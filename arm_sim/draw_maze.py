"""
让机械臂的笔竖直地描出整条迷宫解。

流程:
  1. 用 maze_planner.solve_path 从迷宫照片解出路径(矫正图像素坐标)
  2. 把扫描后的迷宫贴到纸面(21x30cm)上做纹理, 路径按同一映射换算到纸面物理坐标
  3. 笔尖竖直(沿用 arm_kinematics 的两段式 IK)沿路径逐点描画, 红色笔迹逐帧增长
  4. 离屏渲染成 mp4

    conda activate isaac
    python arm_sim/draw_maze.py                  # -> Maze/arm_draw_maze.mp4
    MAZE_FRAMETEST=1 python arm_sim/draw_maze.py # 只校验+取景一帧
"""

import os
import sys
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "arm_sim"))
sys.path.insert(0, os.path.join(REPO, "maze_planner"))

import cv2
from maze_planner import solve_path

# 纸面: 30cm(沿X,前方) x 21cm(沿Y), 放在底座正前方地面上
PAPER_CX, PAPER_CY = 0.22, 0.0
PAPER_SX, PAPER_SY, PAPER_TOP = 0.30, 0.21, 0.002
PEN_Z = PAPER_TOP + 0.001
FPS, W, H = 30, 1280, 720
N_WAYPOINTS = 260
MAZE_IMG = os.path.join(REPO, "maze_planner", "samples", "test_0.jpg")


def img_to_world(px, py, wimg, himg):
    """矫正图像素 (px,py) -> 纸面世界坐标 (与纹理 UV 一致)。"""
    u, v = px / wimg, py / himg
    return (PAPER_CX + (0.5 - v) * PAPER_SX,
            PAPER_CY + (u - 0.5) * PAPER_SY, PEN_Z)


def resample(pts, n):
    """按弧长把折线重采样成 n 个等距点 (顺带平滑台阶)。"""
    P = np.asarray(pts, float)
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    s = np.linspace(0, d[-1], n)
    return np.c_[np.interp(s, d, P[:, 0]), np.interp(s, d, P[:, 1])]


# === 先在 Isaac 之外把迷宫解算好 ===========================================
print("[maze ] 规划迷宫解 ...", flush=True)
_IMG_DIR = os.path.join(REPO, "maze_planner", "outputs", "image")
pts, warped, binary, start_xy, goal_xy = solve_path(MAZE_IMG, auto=True, debug_dir=_IMG_DIR)
himg, wimg = binary.shape[:2]
path_px = resample(pts, N_WAYPOINTS)

# 把迷宫(二值扫描件)存成纹理, 并标注起点(红)/终点(蓝)
tex = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
cv2.circle(tex, (int(start_xy[0]), int(start_xy[1])), 10, (0, 0, 220), -1)
cv2.circle(tex, (int(goal_xy[0]), int(goal_xy[1])), 10, (220, 0, 0), -1)
TEX_PATH = os.path.join(REPO, "arm_sim", "_maze_tex.png")
cv2.imwrite(TEX_PATH, tex)
print(f"[maze ] 路径 {len(pts)} 点 -> 重采样 {N_WAYPOINTS}; 纹理 {wimg}x{himg}", flush=True)


# === 启动 Isaac ============================================================
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.kit.commands
import omni.usd
import imageio
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, Vt
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.sensors.camera import Camera

from arm_kinematics import fk_M5, fk_nib_tail, fk_true_pen, fk_pos, ik, _PEN_T

URDF = os.path.join(REPO, "urdf", "five_dof_arm.urdf")
OUT = os.path.join(REPO, "arm_sim", "video", "arm_draw_maze.mp4")
os.makedirs(os.path.dirname(OUT), exist_ok=True)


def aim_camera(stage, cam_path, eye, target, up=(0, 0, 1)):
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up))
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(cam_path))
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(view.GetInverse())


def make_textured_paper(stage):
    """带迷宫纹理的纸面 quad, 4 角与 img_to_world 的 4 角一致。"""
    z = PAPER_TOP
    A = img_to_world(0, 0, wimg, himg)            # 图左上
    B = img_to_world(wimg, 0, wimg, himg)         # 图右上
    C = img_to_world(wimg, himg, wimg, himg)      # 图右下
    D = img_to_world(0, himg, wimg, himg)         # 图左下
    mesh = UsdGeom.Mesh.Define(stage, "/World/paper")
    mesh.CreatePointsAttr([Gf.Vec3f(*A), Gf.Vec3f(*B), Gf.Vec3f(*C), Gf.Vec3f(*D)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    mesh.SetNormalsInterpolation("vertex")
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([Gf.Vec2f(0, 1), Gf.Vec2f(1, 1), Gf.Vec2f(1, 0), Gf.Vec2f(0, 0)])

    mat = UsdShade.Material.Define(stage, "/World/paperMat")
    pbr = UsdShade.Shader.Define(stage, "/World/paperMat/PBR")
    pbr.CreateIdAttr("UsdPreviewSurface")
    pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    pbr.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    reader = UsdShade.Shader.Define(stage, "/World/paperMat/stReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex = UsdShade.Shader.Define(stage, "/World/paperMat/tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(TEX_PATH)
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb")
    pbr.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(mesh).Bind(mat)


def make_trail(stage):
    pts = UsdGeom.Points.Define(stage, "/World/trail")
    pts.CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.05, 0.05)])
    return pts


def update_trail(pts_prim, pts):
    if len(pts) < 1:
        return
    # 抬高 4mm 画, 避免和纸面纹理 z-fighting; 用密集圆点连成线
    P = [Gf.Vec3f(p[0], p[1], PAPER_TOP + 0.004) for p in pts]
    pts_prim.GetPointsAttr().Set(Vt.Vec3fArray(P))
    pts_prim.GetWidthsAttr().Set([0.004] * len(P))


def main():
    _, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    cfg.merge_fixed_joints = False
    cfg.fix_base = True
    cfg.make_default_prim = True
    cfg.create_physics_scene = True
    cfg.distance_scale = 1.0
    ok, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile", urdf_path=URDF,
        import_config=cfg, get_articulation_root=True)
    print(f"[import] ok={ok}", flush=True)

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/KeyLight"))
    key.CreateIntensityAttr(3000)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-45, 0, 30))
    UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Dome")).CreateIntensityAttr(800)

    make_textured_paper(stage)
    trail = make_trail(stage)
    robot = world.scene.add(Robot(prim_path=prim_path, name="arm"))
    try:
        world.get_physics_context().set_gravity(0.0)
    except Exception:
        pass

    eye, target = (1.93, 1.62, 1.34), (0.15, 0.0, 0.12)
    cam = Camera(prim_path="/World/cam", resolution=(W, H))
    world.reset()
    cam.initialize()
    aim_camera(stage, "/World/cam", eye, target)
    n = robot.num_dof

    # FK 校验
    pen_prim = stage.GetPrimAtPath("/five_dof_arm/pen_tip")
    from pxr import Usd
    q = np.array([0.2, 0.3, -0.3, 0.4, 0.1])
    robot.set_joint_positions(q)
    for _ in range(3):
        world.step(render=False)
    t = UsdGeom.Xformable(pen_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).ExtractTranslation()
    print(f"[fk   ] 与 Isaac pen_tip 误差 = "
          f"{np.linalg.norm(np.array([t[0],t[1],t[2]])-fk_true_pen(q))*1000:.2f} mm", flush=True)

    # 路径 -> 世界目标 -> IK
    targets = [img_to_world(px, py, wimg, himg) for px, py in path_px]
    qtraj, res, tilt = [], [], []
    q_seed = np.array([0.0, 1.0, 1.0, 0.0, 0.0])
    for tgt in targets:
        q_seed, e, ti = ik(np.array(tgt), q_seed)
        qtraj.append(q_seed.copy()); res.append(e); tilt.append(ti)
    res, tilt = np.array(res), np.array(tilt)
    print(f"[ik   ] 位置残差: 最大={res.max()*1000:.2f}mm 均值={res.mean()*1000:.2f}mm | "
          f"笔轴偏离竖直: 最大={tilt.max():.2f}°", flush=True)

    def render_frame():
        for _ in range(10):
            world.step(render=True)
        return np.asarray(cam.get_rgba())[:, :, :3].astype(np.uint8)

    for _ in range(40):
        world.step(render=True)

    if os.environ.get("MAZE_FRAMETEST") == "1":
        os.makedirs(os.path.join(REPO, "arm_sim", "_frames"), exist_ok=True)
        # 把笔分别摆到 路径起点/终点, 看是否压在 红/蓝 标记上 (检验纹理对齐)
        for tag, idx in [("start", 0), ("goal", len(qtraj) - 1)]:
            robot.set_joint_positions(qtraj[idx])
            update_trail(trail, [fk_pos(qtraj[idx])])
            rgb = render_frame()
            p = os.path.join(REPO, "arm_sim", "_frames", f"maze_{tag}.png")
            imageio.imwrite(p, rgb)
            print(f"[frametest] {tag}: pen@{np.round(fk_pos(qtraj[idx]),3)} -> {p}", flush=True)
        return

    writer = imageio.get_writer(OUT, fps=FPS, codec="libx264", quality=8, macro_block_size=8)
    pen_pts = []
    for i in range(len(qtraj)):
        robot.set_joint_positions(qtraj[i])
        pen_pts.append(fk_pos(qtraj[i]))
        update_trail(trail, pen_pts)
        for _ in range(3):              # 多走几步让笔迹同步进 RTX 场景
            world.step(render=True)
        writer.append_data(np.asarray(cam.get_rgba())[:, :, :3].astype(np.uint8))
        if i % 30 == 0:
            print(f"[rec  ] frame {i}/{len(qtraj)}", flush=True)
    writer.close()
    print(f"[done ] -> {OUT}", flush=True)


if __name__ == "__main__":
    import traceback
    try:
        main()
        print("✓ 完成", flush=True)
    except Exception:
        print("✗ 运行异常:", flush=True)
        traceback.print_exc()
    finally:
        simulation_app.close()
