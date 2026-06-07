"""
录制机械臂运动的可视化视频 (运动学控制，不依赖 URDF 质量惯量)。

导入 urdf/five_dof_arm.urdf，让 5 个关节按平滑的正弦编排运动，
用一台离屏相机逐帧渲染，编码成 mp4 存到项目根目录。

    conda activate isaac
    python arm_sim/record_video.py            # -> Maze/arm_motion.mp4
"""

import os

from isaacsim import SimulationApp

# 关相机窗口但需要渲染，headless 下 RTX 仍可离屏出图
simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.kit.commands
import omni.usd
import imageio
from pxr import Gf, Sdf, UsdGeom, UsdLux
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.sensors.camera import Camera

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(REPO, "urdf", "five_dof_arm.urdf")
OUT = os.path.join(REPO, "arm_motion.mp4")

FPS = 30
DURATION = 8.0          # 秒
W, H = 1280, 720


def aim_camera(stage, cam_path, eye, target, up=(0, 0, 1)):
    """直接给相机 prim 设一个 look-at 变换矩阵 (避开四元数约定的坑)。
    Gf.SetLookAt 给的是 world->camera 视图矩阵，其逆即 camera->world 局部变换。
    USD 相机本地 -Z 朝前，与 SetLookAt 约定一致。"""
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up))
    m = view.GetInverse()
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(cam_path))
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)


def main():
    # --- 导入 URDF ----------------------------------------------------------
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

    # --- 场景: 地面 + 灯光 --------------------------------------------------
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/KeyLight"))
    key.CreateIntensityAttr(3000)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-45, 0, 30))
    UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Dome")).CreateIntensityAttr(800)

    robot = world.scene.add(Robot(prim_path=prim_path, name="arm"))

    # 纯运动学: 关掉重力，避免无质量连杆漂移
    try:
        world.get_physics_context().set_gravity(0.0)
    except Exception as e:
        print("[warn] set_gravity 失败(忽略):", e, flush=True)

    # --- 相机 (3/4 视角, 拉远到能罩住整个工作空间) -------------------------
    # 手臂在原点附近, reach ~0.4m; 运动时末端会大幅扫动, 所以视野要宽。
    # eye 沿 (eye-target) 方向退到 ~1.8 倍距离, 朝向不变, 拍全貌。
    eye, target = (1.85, 1.62, 1.17), (0.05, 0.0, 0.22)
    cam = Camera(prim_path="/World/cam", resolution=(W, H))

    world.reset()
    cam.initialize()
    aim_camera(stage, "/World/cam", eye, target)
    n = robot.num_dof
    print(f"[robot] num_dof={n} dof_names={robot.dof_names}", flush=True)

    # 渲染管线预热 (前几帧可能是空的)
    for _ in range(40):
        world.step(render=True)

    # 取景测试: 几个视角各拍一张，确认能看到手臂
    if os.environ.get("MAZE_FRAMETEST") == "1":
        robot.set_joint_positions(np.array([0.4, 0.6, -0.5, 0.8, 0.3])[:n])
        os.makedirs(os.path.join(REPO, "arm_sim", "_frames"), exist_ok=True)
        poses = {
            "iso":  (eye, target),                          # 实际录制用的机位
            "iso_near": ((1.05, 0.9, 0.75), (0.05, 0.0, 0.22)),  # 旧的近机位对比
        }
        for name, (e, tg) in poses.items():
            aim_camera(stage, "/World/cam", e, tg)
            for _ in range(8):
                world.step(render=True)
            rgb = np.asarray(cam.get_rgba())[:, :, :3].astype(np.uint8)
            p = os.path.join(REPO, "arm_sim", "_frames", f"test_{name}.png")
            imageio.imwrite(p, rgb)
            print(f"[frametest] {name}: mean={rgb.mean():.1f} std={rgb.std():.1f} -> {p}", flush=True)
        return True
        robot.set_joint_positions(np.array([0.4, 0.6, -0.5, 0.8, 0.3])[:n])
        for _ in range(5):
            world.step(render=True)
        os.makedirs(os.path.join(REPO, "arm_sim", "_frames"), exist_ok=True)
        def mat2quat(R):
            w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
            return np.array([w, (R[2, 1] - R[1, 2]) / (4 * w),
                             (R[0, 2] - R[2, 0]) / (4 * w),
                             (R[1, 0] - R[0, 1]) / (4 * w)])

        e = np.array([0.9, 0.6, 0.55]); tg = np.array([0.15, 0.0, 0.2])
        f = tg - e; f /= np.linalg.norm(f)
        r = np.cross(f, [0, 0, 1.]); r /= np.linalg.norm(r)
        u = np.cross(r, f)
        variants = {
            "negz": mat2quat(np.column_stack([r, u, -f])),
            "posz": mat2quat(np.column_stack([r, u, f])),
            "negz_flipru": mat2quat(np.column_stack([-r, u, -f])),
            "yup_negz": mat2quat(np.column_stack([r, -u, f])),
        }
        cam.set_world_pose(e, np.array([1., 0, 0, 0]))
        for name, q in variants.items():
            cam.set_world_pose(e, q)
            for _ in range(8):
                world.step(render=True)
            rgb = np.asarray(cam.get_rgba())[:, :, :3].astype(np.uint8)
            p = os.path.join(REPO, "arm_sim", "_frames", f"test_{name}.png")
            imageio.imwrite(p, rgb)
            print(f"[frametest] {name}: mean={rgb.mean():.1f} std={rgb.std():.1f} -> {p}", flush=True)
        return True

    # --- 运动编排 + 逐帧录制 -----------------------------------------------
    n_frames = int(FPS * DURATION)
    writer = imageio.get_writer(OUT, fps=FPS, codec="libx264",
                                quality=8, macro_block_size=8)
    amp = np.array([0.8, 0.6, 0.9, 1.0, 1.5])      # 各关节幅度(rad)
    freq = np.array([0.15, 0.20, 0.18, 0.12, 0.25])  # 各关节频率(Hz)
    phase = np.array([0.0, 1.0, 2.0, 0.5, 3.0])
    bias = np.array([0.0, 0.5, 0.0, 0.0, 0.0])

    captured = 0
    for f in range(n_frames):
        t = f / FPS
        q = bias + amp * np.sin(2 * np.pi * freq * t + phase)
        robot.set_joint_positions(q[:n])
        world.step(render=True)
        rgba = cam.get_rgba()
        if rgba is None or np.asarray(rgba).size == 0:
            continue
        writer.append_data(np.asarray(rgba)[:, :, :3].astype(np.uint8))
        captured += 1
        if f % 30 == 0:
            print(f"[rec  ] frame {f}/{n_frames}", flush=True)

    writer.close()
    print(f"[done ] 写入 {captured} 帧 -> {OUT}", flush=True)
    return captured > 0


if __name__ == "__main__":
    import traceback
    ok = False
    try:
        ok = main()
        print("✓ 视频已生成" if ok else "✗ 没有捕获到帧", flush=True)
    except Exception:
        print("✗ 运行异常:", flush=True)
        traceback.print_exc()
    finally:
        simulation_app.close()
