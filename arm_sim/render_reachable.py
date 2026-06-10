"""
根据各关节角度限位，划定笔尖（笔头）的可达区域，渲染一张俯视图并在地面上标出边界。

原理：底盘 b 能转 ±180°（绕底座近一整圈），所以「笔尖能竖直触到纸面高度」的可达区域是
一个以底座为圆心的**圆环**——半径由 s/e/w 限位决定，角度由 b 扫满。于是只需沿一个方向
扫描半径 r，对每个 r 解 IK（目标在纸面高度、笔尽量竖直），检查解出的关节角是否都在真机
限位内，找出可达的 [r_min, r_max]，再在地面画出内外两个边界圆。

输出: assets/reachable_area.png（俯视图，地面上红=外边界、橙=内边界）。

    export OMNI_KIT_ACCEPT_EULA=YES
    conda activate dexbench
    python arm_sim/render_reachable.py
"""

import os
import sys
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "arm_sim"))

from arm_kinematics import ik   # noqa: E402  纯 numpy，Isaac 启动前先算

PEN_Z = 0.003                   # 纸面高度（笔尖目标 z），与 draw_maze 一致
# 真机限位（度）。w 放宽到 ±95 以容忍「画竖直笔需 ~-93°、被 clamp 到 -90」的情况。
LIM = [(-180, 180), (-90, 90), (-90, 90), (-95, 95), (-180, 180)]


def reachable(r, seed):
    """笔尖目标 (r,0,PEN_Z) 能否在限位内、笔接近竖直地到达。返回 (ok, q)。"""
    tgt = np.array([r, 0.0, PEN_Z])
    q, res, tilt = ik(tgt, seed)
    deg = np.degrees(q)
    ok = (res < 1e-3) and (tilt < 20.0) and all(
        lo <= v <= hi for v, (lo, hi) in zip(deg, LIM))
    return ok, q


def compute_reach():
    """扫描半径，返回 (r_min, r_max)。"""
    rs = np.linspace(0.03, 0.70, 400)
    seed = np.array([0.0, 1.0, 1.0, 0.0, 0.0])
    mask = []
    for r in rs:
        ok, q = reachable(r, seed)
        if ok:
            seed = q
        mask.append(ok)
    mask = np.array(mask)
    idx = np.where(mask)[0]
    if len(idx) == 0:
        raise RuntimeError("没有任何可达半径，检查限位/IK")
    r_min, r_max = float(rs[idx[0]]), float(rs[idx[-1]])
    print(f"[reach] 可达半径环: r_min={r_min*100:.1f}cm  r_max={r_max*100:.1f}cm  "
          f"(笔尖在纸面高度 z={PEN_Z*1000:.0f}mm)", flush=True)
    return r_min, r_max


R_MIN, R_MAX = compute_reach()

# ---- 算完可达半径，再启动 Isaac 渲染 ----
from isaacsim import SimulationApp        # noqa: E402
simulation_app = SimulationApp({"headless": True})

import omni.kit.commands                  # noqa: E402
import omni.usd                           # noqa: E402
import imageio                            # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, Vt   # noqa: E402
from isaacsim.core.api import World       # noqa: E402
from isaacsim.core.api.robots import Robot   # noqa: E402
from isaacsim.sensors.camera import Camera   # noqa: E402

URDF = os.path.join(REPO, "urdf", "five_dof_arm.urdf")
OUT = os.path.join(REPO, "assets", "reachable_area.png")
os.makedirs(os.path.dirname(OUT), exist_ok=True)
W, H = 1600, 1600


def aim_camera(stage, cam_path, eye, target, up=(0, 0, 1)):
    if abs(eye[0] - target[0]) < 1e-6 and abs(eye[1] - target[1]) < 1e-6:
        up = (1, 0, 0)
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up))
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(cam_path))
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(view.GetInverse())


def draw_circle(stage, path, radius, z, color, n=160, width=0.006):
    crv = UsdGeom.BasisCurves.Define(stage, path)
    crv.CreateTypeAttr("linear")
    ts = np.linspace(0, 2 * np.pi, n)
    pts = [Gf.Vec3f(float(radius * np.cos(t)), float(radius * np.sin(t)), z) for t in ts]
    crv.CreatePointsAttr(pts)
    crv.CreateCurveVertexCountsAttr([n])
    crv.CreateWidthsAttr([width] * n)
    crv.SetWidthsInterpolation("vertex")
    crv.CreateDisplayColorAttr([Gf.Vec3f(*color)])


# 纸面（与 draw_maze / draw_maze_real 一致）：中心在底座前方 22cm，30cm(X) x 21cm(Y)
PAPER_CX, PAPER_CY, PAPER_SX, PAPER_SY = 0.22, 0.0, 0.30, 0.21


def draw_rect(stage, path, cx, cy, sx, sy, z, color, width=0.005):
    hx, hy = sx / 2.0, sy / 2.0
    corners = [(cx - hx, cy - hy), (cx + hx, cy - hy),
               (cx + hx, cy + hy), (cx - hx, cy + hy), (cx - hx, cy - hy)]
    crv = UsdGeom.BasisCurves.Define(stage, path)
    crv.CreateTypeAttr("linear")
    pts = [Gf.Vec3f(float(x), float(y), z) for x, y in corners]
    crv.CreatePointsAttr(pts)
    crv.CreateCurveVertexCountsAttr([len(pts)])
    crv.CreateWidthsAttr([width] * len(pts))
    crv.SetWidthsInterpolation("vertex")
    crv.CreateDisplayColorAttr([Gf.Vec3f(*color)])


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
    UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Dome")).CreateIntensityAttr(1500)

    robot = world.scene.add(Robot(prim_path=prim_path, name="arm"))
    try:
        world.get_physics_context().set_gravity(0.0)
    except Exception:
        pass

    # 地面上画可达边界：外环红、内环橙
    draw_circle(stage, "/World/reach_outer", R_MAX, 0.004, (0.95, 0.1, 0.1))
    draw_circle(stage, "/World/reach_inner", R_MIN, 0.004, (1.0, 0.6, 0.1))
    # 叠加纸面摆放框（draw_maze 默认位置）
    draw_rect(stage, "/World/paper_frame", PAPER_CX, PAPER_CY,
              PAPER_SX, PAPER_SY, 0.005, (0.1, 0.3, 0.9))

    cam = Camera(prim_path="/World/cam", resolution=(W, H))
    world.reset()
    cam.initialize()
    n = robot.num_dof
    robot.set_joint_positions(np.zeros(n))

    # 俯视：相机放在底座正上方看 XY 平面（eye 与 target 同 x/y，触发 up=(1,0,0)），
    # 高度抬够让外边界圆完整入画。
    eye = (0.0, 0.0, R_MAX * 4.0 + 0.6)
    aim_camera(stage, "/World/cam", eye, (0.0, 0.0, 0.0))
    for _ in range(60):
        world.step(render=True)

    rgb = np.asarray(cam.get_rgba())[:, :, :3].astype(np.uint8)
    imageio.imwrite(OUT, rgb)
    print(f"[done] r_min={R_MIN*100:.1f}cm r_max={R_MAX*100:.1f}cm "
          f"mean={rgb.mean():.1f} -> {OUT}", flush=True)


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
