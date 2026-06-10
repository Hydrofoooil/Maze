"""
渲染「所有关节零位 (q=[0,0,0,0,0])」的机械臂截图，用于实机装配参考。

输出: assets/zero_pose.png（以及若干不同视角的 zero_pose_*.png）。
零位 = URDF 各关节角都为 0 时的姿态，对应说明书里各关节「默认 0 度」的装配基准。

    export OMNI_KIT_ACCEPT_EULA=YES
    conda activate dexbench
    python arm_sim/render_zero_pose.py
"""

import os
import numpy as np

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.kit.commands
import omni.usd
import imageio
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.sensors.camera import Camera

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(REPO, "urdf", "five_dof_arm.urdf")
OUT_DIR = os.path.join(REPO, "assets")
os.makedirs(OUT_DIR, exist_ok=True)
W, H = 1600, 1200

# 多个视角：等距 / 正前方(+X 看向 -X) / 正侧方(+Y 看向 -Y) / 正上方俯视
VIEWS = {
    "zero_pose":      ((1.1, 1.1, 0.8), (0.05, 0.0, 0.25)),   # 3/4 等距（主图）
    "zero_pose_front":((1.4, 0.0, 0.30), (0.0, 0.0, 0.25)),   # 正前方看
    "zero_pose_side": ((0.0, 1.4, 0.30), (0.0, 0.0, 0.25)),   # 正侧方看
    "zero_pose_top":  ((0.05, 0.0, 1.6), (0.05, 0.0, 0.2)),   # 正上方俯视
}


def aim_camera(stage, cam_path, eye, target, up=(0, 0, 1)):
    # 俯视时 up 不能和视线平行，换成 +X
    if abs(eye[0] - target[0]) < 1e-6 and abs(eye[1] - target[1]) < 1e-6:
        up = (1, 0, 0)
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up))
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(cam_path))
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(view.GetInverse())


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
    UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Dome")).CreateIntensityAttr(1200)

    robot = world.scene.add(Robot(prim_path=prim_path, name="arm"))
    try:
        world.get_physics_context().set_gravity(0.0)
    except Exception:
        pass

    cam = Camera(prim_path="/World/cam", resolution=(W, H))
    world.reset()
    cam.initialize()
    n = robot.num_dof
    print(f"[robot] num_dof={n} dof_names={robot.dof_names}", flush=True)

    # 所有关节置零
    robot.set_joint_positions(np.zeros(n))
    for _ in range(60):                 # 预热渲染管线 + 让姿态稳定
        world.step(render=False)

    # 打印零位时笔尖位置（装配参考）
    pen_prim = stage.GetPrimAtPath("/five_dof_arm/pen_tip")
    if pen_prim and pen_prim.IsValid():
        t = UsdGeom.Xformable(pen_prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()
        print(f"[zero] pen_tip @ ({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}) m", flush=True)

    for name, (eye, target) in VIEWS.items():
        aim_camera(stage, "/World/cam", eye, target)
        for _ in range(15):
            world.step(render=True)
        rgb = np.asarray(cam.get_rgba())[:, :, :3].astype(np.uint8)
        out = os.path.join(OUT_DIR, f"{name}.png")
        imageio.imwrite(out, rgb)
        print(f"[done] {name}: mean={rgb.mean():.1f} -> {out}", flush=True)


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
