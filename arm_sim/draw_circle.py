"""
让 5-DOF 机械臂的笔尖在底座前方的纸面(21cm x 30cm)上画一个圆。

- 自带 FK (依据 URDF 里给出的齐次变换) + 阻尼最小二乘 IK (仅位置)，
  运行时会先用 Isaac 的真实 pen_tip 位姿校验 FK 是否正确。
- 纯运动学控制 (set_joint_positions)，不依赖 URDF 质量惯量。
- 笔迹用一条红色曲线逐帧增长画在纸上。

    conda activate isaac
    python arm_sim/draw_circle.py                 # -> Maze/arm_draw_circle.mp4
    MAZE_FRAMETEST=1 python arm_sim/draw_circle.py # 校验FK/IK + 取景, 不出整段视频
"""

import os
import numpy as np

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.kit.commands
import omni.usd
import imageio
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, Vt
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.sensors.camera import Camera

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(REPO, "urdf", "five_dof_arm.urdf")
OUT = os.path.join(REPO, "arm_sim", "video", "arm_draw_circle.mp4")
os.makedirs(os.path.dirname(OUT), exist_ok=True)
FPS, W, H = 30, 1280, 720

# 纸面: 30cm(沿X,前方) x 21cm(沿Y), 放在底座正前方的地面上
PAPER_CX, PAPER_CY = 0.22, 0.0
PAPER_SX, PAPER_SY, PAPER_TH = 0.30, 0.21, 0.002
PAPER_TOP = PAPER_TH                       # 纸面上表面 z
# 圆: 画在纸面上
CIRCLE_R = 0.05
PEN_Z = PAPER_TOP + 0.001                  # 笔尖目标平面 (紧贴纸面)


# FK/IK 已抽到 arm_kinematics (纯 numpy, 与 Isaac 无关), 两个脚本共用
from arm_kinematics import (fk_M5, fk_nib_tail, fk_true_pen, fk_pos, ik,
                            _NIB, _TAIL, _PEN_T)


# ---------------------------------------------------------------------------
def aim_camera(stage, cam_path, eye, target, up=(0, 0, 1)):
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up))
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(cam_path))
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(view.GetInverse())


def make_paper(stage):
    cube = UsdGeom.Cube.Define(stage, "/World/paper")
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.97, 0.97, 0.97)])
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(PAPER_CX, PAPER_CY, PAPER_TH / 2))
    xf.AddScaleOp().Set(Gf.Vec3f(PAPER_SX, PAPER_SY, PAPER_TH))


def make_trail(stage):
    crv = UsdGeom.BasisCurves.Define(stage, "/World/trail")
    crv.CreateTypeAttr("linear")
    crv.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.05, 0.05)])
    return crv


def update_trail(crv, pts):
    if len(pts) < 2:
        return
    crv.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    crv.GetCurveVertexCountsAttr().Set([len(pts)])
    crv.GetWidthsAttr().Set([0.004] * len(pts))
    crv.SetWidthsInterpolation("vertex")


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
    print(f"[import] ok={ok} prim_path={prim_path}", flush=True)

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/KeyLight"))
    key.CreateIntensityAttr(3000)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-45, 0, 30))
    UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Dome")).CreateIntensityAttr(800)

    make_paper(stage)
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
    print(f"[robot] num_dof={n} dof_names={robot.dof_names}", flush=True)

    # --- 校验 FK: 和 Isaac 实际 pen_tip 世界坐标对比 -----------------------
    pen_prim = stage.GetPrimAtPath("/five_dof_arm/pen_tip")
    rng = np.random.default_rng(0)
    max_fk_err = 0.0
    for _ in range(4):
        q = rng.uniform(-0.8, 0.8, n)
        robot.set_joint_positions(q)
        for _ in range(3):
            world.step(render=False)
        t = UsdGeom.Xformable(pen_prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()
        isaac_p = np.array([t[0], t[1], t[2]])
        err = np.linalg.norm(isaac_p - fk_true_pen(q))
        max_fk_err = max(max_fk_err, err)
    print(f"[fk   ] 与 Isaac pen_tip 最大误差 = {max_fk_err*1000:.2f} mm", flush=True)

    # --- 规划圆轨迹 + IK ---------------------------------------------------
    n_frames = int(FPS * 8)
    loops = 1.5
    phis = np.linspace(-np.pi/2, -np.pi/2 + loops * 2*np.pi, n_frames)
    qtraj, residuals, tilts = [], [], []
    q_seed = np.array([0.0, 1.0, 1.0, 0.0, 0.0])      # 朝前下方折叠的初始猜测
    for phi in phis:
        tgt = np.array([PAPER_CX + CIRCLE_R*np.cos(phi),
                        PAPER_CY + CIRCLE_R*np.sin(phi), PEN_Z])
        q_seed, res, tilt = ik(tgt, q_seed)
        qtraj.append(q_seed.copy())
        residuals.append(res); tilts.append(tilt)
    residuals, tilts = np.array(residuals), np.array(tilts)
    print(f"[ik   ] 位置残差: 最大={residuals.max()*1000:.2f}mm 均值={residuals.mean()*1000:.2f}mm | "
          f"笔轴偏离竖直: 最大={tilts.max():.2f}° 均值={tilts.mean():.2f}°", flush=True)

    # --- 取景测试: 校验完打一帧就退出 -------------------------------------
    if os.environ.get("MAZE_FRAMETEST") == "1":
        os.makedirs(os.path.join(REPO, "arm_sim", "_frames"), exist_ok=True)
        half = n_frames // 2
        pts = []
        for i in range(half):
            robot.set_joint_positions(qtraj[i])
            pts.append(fk_pos(qtraj[i]))
        update_trail(trail, pts)

        # 诊断: 在三个候选点放彩球, 看哪个落在可见笔尖上
        if os.environ.get("MAZE_MARKERS") == "1":
            q = qtraj[half]
            robot.set_joint_positions(q)
            M5 = fk_M5(q); o, R = M5[:3, 3], M5[:3, :3]
            cand = {
                "nib":  (o + R @ _NIB, (1.0, 0.1, 0.1)),       # 红: 笔尖(-Y端)
                "tail": (o + R @ _TAIL, (0.1, 1.0, 0.1)),      # 绿: 笔尾(+Y端)
            }
            for name, (pos, col) in cand.items():
                s = UsdGeom.Sphere.Define(stage, f"/World/mk_{name}")
                s.CreateRadiusAttr(0.008)
                s.CreateDisplayColorAttr([Gf.Vec3f(*col)])
                UsdGeom.Xformable(s).AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in pos]))
                print(f"[mk   ] {name} = {np.round(pos,3)}", flush=True)

        for _ in range(10):
            world.step(render=True)
        rgb = np.asarray(cam.get_rgba())[:, :, :3].astype(np.uint8)
        p = os.path.join(REPO, "arm_sim", "_frames", "circle_test.png")
        imageio.imwrite(p, rgb)
        print(f"[frametest] mean={rgb.mean():.1f} std={rgb.std():.1f} -> {p}", flush=True)
        return True

    # --- 录制 --------------------------------------------------------------
    for _ in range(30):
        world.step(render=True)
    writer = imageio.get_writer(OUT, fps=FPS, codec="libx264", quality=8, macro_block_size=8)
    pts = []
    for i in range(n_frames):
        robot.set_joint_positions(qtraj[i])
        pts.append(fk_pos(qtraj[i]))
        update_trail(trail, pts)
        world.step(render=True)
        rgb = np.asarray(cam.get_rgba())[:, :, :3].astype(np.uint8)
        writer.append_data(rgb)
        if i % 30 == 0:
            print(f"[rec  ] frame {i}/{n_frames}", flush=True)
    writer.close()
    print(f"[done ] -> {OUT}", flush=True)
    return True


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
