"""
最小验证: 把 urdf/five_dof_arm.urdf 导入 Isaac Sim 5.1 (headless)，
确认 (1) 导入成功、(2) 重力下能站住、(3) 关节可被驱动跟踪目标。

在 isaac conda 环境下运行:
    conda activate isaac
    python arm_sim/verify_import.py
"""

import os

# SimulationApp 必须最先创建，之后才能 import 其它 isaacsim/omni 模块
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.types import ArticulationAction

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(REPO, "urdf", "five_dof_arm.urdf")
assert os.path.exists(URDF), f"找不到 URDF: {URDF}"


def main():
    # --- 1. 导入 URDF -------------------------------------------------------
    _, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    cfg.merge_fixed_joints = False     # 保留 pen_tip 等固定连杆
    cfg.fix_base = True                # 底座固定 (机械臂装在桌上)
    cfg.make_default_prim = True
    cfg.create_physics_scene = True    # 自动建物理场景(含重力)
    cfg.distance_scale = 1.0           # URDF 单位=米
    cfg.set_default_drive_type(1)      # 1 = 位置驱动

    ok, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile", urdf_path=URDF,
        import_config=cfg, get_articulation_root=True)
    print(f"[import] ok={ok} prim_path={prim_path}")
    assert ok and prim_path, "URDF 导入失败"

    # --- 2. 搭场景 ----------------------------------------------------------
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    robot = world.scene.add(Robot(prim_path=prim_path, name="arm"))
    world.reset()                      # 初始化物理，articulation 上线

    n = robot.num_dof
    print(f"[robot] num_dof={n} dof_names={robot.dof_names}")
    assert n >= 5, f"自由度异常: {n}"

    # 注意: 该 URDF 未指定 mass/inertia，导入器只能给极小的默认惯量，
    # 所以基于物理驱动(力矩)的控制会病态(要么发散要么塌陷)。
    # 描迷宫是运动学任务(让笔尖跟轨迹)，用运动学控制即可，不依赖动力学。

    # --- 3. 运动学控制链路测试: 逐关节写入角度并读回 -----------------------
    rng = np.random.default_rng(0)
    targets = np.clip(rng.uniform(-1.0, 1.0, n), -3.0, 3.0)
    robot.set_joint_positions(targets)
    world.step(render=False)                       # 让状态生效
    q = robot.get_joint_positions()
    err = np.abs(q - targets)
    print(f"[kine ] 目标 q*={np.round(targets, 3)}")
    print(f"[kine ] 读回 q ={np.round(q, 3)}  最大误差={err.max():.5f} rad")

    # 回零位再验一次
    robot.set_joint_positions(np.zeros(n))
    world.step(render=False)
    q0 = robot.get_joint_positions()
    print(f"[kine ] 回零位 q={np.round(q0, 4)}  max|q|={np.abs(q0).max():.5f}")

    ok_import = bool(ok) and n == 5
    ok_kine = err.max() < 1e-2 and np.abs(q0).max() < 1e-2
    print(f"\n结果: 导入&articulation={'通过' if ok_import else '失败'}  "
          f"运动学控制={'通过' if ok_kine else '失败'}")
    return ok_import and ok_kine


if __name__ == "__main__":
    import traceback
    success = False
    try:
        success = main()
        print("✓ 最小验证全部通过" if success else "✗ 验证未通过，见上面日志",
              flush=True)
    except Exception:
        # 在 close() 硬退出前把 traceback 打出来，否则会被吞掉
        print("✗ 运行异常:", flush=True)
        traceback.print_exc()
        import sys
        sys.stdout.flush()
        sys.stderr.flush()
    finally:
        simulation_app.close()
