"""
真机定点探针：诊断「模型 vs 实物」（C 层）的运动学偏差。

用途：你在交互提示里输入一个基座系世界坐标（米），脚本用 arm_kinematics.ik
求解 5 关节角，打印诊断信息，并把真实机械臂移动到该点停住，让你肉眼/用尺
比对「指定坐标」与「笔尖实际落点」的偏差。

为什么需要它（而不是纯 numpy 的 IK→FK round-trip）：
  在纯 numpy 里 fk_pos(ik(target)) 是「自证」的——IK 的目标函数就是最小化
  ‖fk_pos(q)-target‖，所以 round-trip 只能暴露：优化器不收敛、关节限位被
  clamp、目标超出工作空间。它无法发现「模型本身算错」（变换矩阵符号、_NIB
  笔尖偏移标定误差、舵机标定）。真机提供了 arm_kinematics.py 之外的地面真值，
  所以「指定坐标 → 真机移过去 → 你看实际落点」才能暴露这些建模偏差。

输入：交互式逐点输入  x y  或  x y z （单位米，基座系）。
  - 只给 x y 时，z 缺省用纸面 PEN_Z=0.003。
  - 不经过 img_to_world：隔离纯运动学+硬件，不掺图像映射误差。
  - 输入 q / 空行 / Ctrl-C 退出（退出前抬笔回安全姿态）。

点间运动：抬笔 → 平移 → 落下。
  先在目标正上方 (z+LIFT) 求解并移动到目标上空，再垂直落到目标 z。看完按
  回车，抬笔再去下一个点。绝不贴纸横拖（会划纸）。

真机前提：
  - 下位机 robot_server 已启动，9001 端口在线（与 draw_maze_real.py 相同）。
  - 一张纸平放在底座正前方，便于你比对落点。
  - 笔已固定、笔尖朝下。

复用现有模块，不重写：arm_kinematics.ik、RobotClient、robot_config、
draw_maze_real 的 LIMITS_DEG / check_limits / prepend_prep_start_points 约定。
"""

import os
import sys
import argparse

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "arm_sim"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from arm_kinematics import ik, fk_pos          # noqa: E402
from robot_client import RobotClient           # noqa: E402
from robot_config import ROBOT_HOST, ROBOT_PORT, DEFAULT_SPD, DEFAULT_ACC, DEFAULT_DT  # noqa: E402

# 纸面接触高度（与 draw_maze_real.py 一致：PAPER_TOP 0.002 + 0.001）
PEN_Z = 0.003
# 抬笔高度（米）：点间平移时在目标正上方多高
LIFT = 0.030

# 关节顺序与限位（URDF joint_1..5 = 真机 b/s/e/w/h），与 draw_maze_real.py 一致。
JOINT_ORDER = ["b", "s", "e", "w", "h"]
LIMITS_DEG = {"b": (-180, 180), "s": (-90, 90), "e": (-90, 90),
              "w": (-90, 90), "h": (-45, 45)}

# IK 种子：朝前下方折叠的初始猜测（与 draw_maze_real.py 一致）
Q_SEED_DEFAULT = np.array([0.0, 1.0, 1.0, 0.0, 0.0])


def q_to_point(q):
    """rad 关节解 -> {b,s,e,w,h} 度。"""
    deg = np.degrees(q)
    return {k: float(deg[i]) for i, k in enumerate(JOINT_ORDER)}


def check_limits_point(point):
    """检查单个点是否超限，返回 (是否超限, 超限关节列表)。"""
    over = []
    for k in JOINT_ORDER:
        lo, hi = LIMITS_DEG[k]
        if not (lo <= point[k] <= hi):
            over.append(k)
    return bool(over), over


def solve_and_report(target, q_seed):
    """对目标点求 IK，打印诊断信息。返回 (q, point, ok)。
    ok=False 表示残差过大或超限，调用方应要求用户确认。"""
    q, res_m, tilt_deg = ik(target, q_seed)
    # 用求得的 q 正算回笔尖位置，显式展示模型内部一致性（A 层自检）
    fk_back = fk_pos(q)
    res_mm = res_m * 1000.0
    point = q_to_point(q)
    over, over_joints = check_limits_point(point)

    print(f"  目标(米)      : x={target[0]:.4f} y={target[1]:.4f} z={target[2]:.4f}", flush=True)
    print(f"  FK回算笔尖(米): x={fk_back[0]:.4f} y={fk_back[1]:.4f} z={fk_back[2]:.4f}", flush=True)
    print(f"  关节角(度)    : " +
          "  ".join(f"{k}={point[k]:+7.2f}" for k in JOINT_ORDER), flush=True)
    print(f"  位置残差      : {res_mm:.3f} mm   笔轴偏离竖直: {tilt_deg:.2f}°", flush=True)

    ok = True
    if res_mm > 1.0:
        print(f"  [警告] 位置残差 {res_mm:.3f}mm 偏大（目标可能超出工作空间或优化未收敛）", flush=True)
        ok = False
    if over:
        for k in over_joints:
            lo, hi = LIMITS_DEG[k]
            print(f"  [警告] 关节 {k}={point[k]:+.2f}° 超限[{lo},{hi}]，下位机会 clamp，落点会失真", flush=True)
        ok = False
    if tilt_deg > 10.0:
        print(f"  [提示] 笔轴偏离竖直 {tilt_deg:.2f}°（腕关节受限时常见，落点仍可用但笔偏斜）", flush=True)

    return q, point, ok


def goto_target(robot, target, q_seed, spd, acc, dt):
    """抬笔→平移→落下，把笔尖移动到 target。
    先在目标正上方 (z+LIFT) 求解并移动，再垂直落到目标 z。
    返回 (q_at_target, point_at_target, ok) 供下一点作种子；发送失败返回 None。"""
    above = np.array([target[0], target[1], target[2] + LIFT])

    # 1) 在目标上方求解
    print("[上方] 求解目标正上方 (z+{:.0f}mm)：".format(LIFT * 1000), flush=True)
    q_above, pt_above, ok_above = solve_and_report(above, q_seed)

    # 2) 在目标本身求解
    print("[落点] 求解目标点：", flush=True)
    q_at, pt_at, ok_at = solve_and_report(target, q_above)

    if not (ok_above and ok_at):
        ans = input("  >> 上方/落点解有警告，仍要发送吗？(y/N) ").strip().lower()
        if ans != "y":
            print("  已跳过该点（未发送）。", flush=True)
            return None

    # 3) 先发上方点，再发落点（开环按 dt 间隔，单点用 joint 即可）
    print(f"[send] 移动到目标上方 -> 垂直落下 (spd={spd}, acc={acc})", flush=True)
    try:
        robot.joint(**pt_above, spd=spd, acc=acc)
        robot.joint(**pt_at, spd=spd, acc=acc)
    except OSError as ex:
        print(f"  [错误] 发送失败：{ex}", flush=True)
        return None
    print("  已落到目标点。比对实际落点后回车继续。", flush=True)
    return q_at, pt_at, ok_at


def lift_pen(robot, q_seed, spd, acc):
    """从当前位置抬笔到安全高度（用最近一次的 x,y，z 抬到 PEN_Z+LIFT）。
    q_seed 为最近解，用其 FK 取当前 x,y。"""
    cur = fk_pos(q_seed)
    above = np.array([cur[0], cur[1], PEN_Z + LIFT])
    q_above, _, _ = ik(above, q_seed)
    try:
        robot.joint(**q_to_point(q_above), spd=spd, acc=acc)
    except OSError as ex:
        print(f"  [错误] 抬笔失败：{ex}", flush=True)


def parse_coord(line):
    """解析一行输入为 (x,y,z) 米。接受 'x y' 或 'x y z'；z 缺省 PEN_Z。
    返回 np.array 或 None（无法解析）。"""
    parts = line.replace(",", " ").split()
    try:
        vals = [float(p) for p in parts]
    except ValueError:
        return None
    if len(vals) == 2:
        return np.array([vals[0], vals[1], PEN_Z])
    if len(vals) == 3:
        return np.array(vals)
    return None


def main():
    global LIFT
    ap = argparse.ArgumentParser(description="真机定点探针：指定笔尖世界坐标→IK→真机移过去看落点")
    ap.add_argument("--send", action="store_true",
                    help="真正下发到机械臂（默认 dry-run：只求解+打印，不碰硬件）")
    ap.add_argument("--host", default=ROBOT_HOST, help="下位机地址（默认取自 robot_config）")
    ap.add_argument("--port", type=int, default=ROBOT_PORT, help="下位机端口（默认取自 robot_config）")
    ap.add_argument("--dt", type=float, default=DEFAULT_DT, help="相邻点时间间隔(s)")
    ap.add_argument("--spd", type=int, default=DEFAULT_SPD, help="关节角速度(°/s)")
    ap.add_argument("--acc", type=int, default=DEFAULT_ACC, help="关节角加速度")
    ap.add_argument("--lift", type=float, default=LIFT * 1000, metavar="MM",
                    help=f"抬笔高度(mm)，默认 {LIFT * 1000:.0f}")
    args = ap.parse_args()

    LIFT = args.lift / 1000.0

    robot = None
    if args.send:
        print(f"[send] 连接 {args.host}:{args.port} ...", flush=True)
        robot = RobotClient(host=args.host, port=args.port)
        try:
            print("[send] ping:", robot.ping(), flush=True)
        except OSError as ex:
            print(f"[错误] 无法连接下位机 {args.host}:{args.port}：{ex}", flush=True)
            print("       请确认 robot_server 已启动、9001 端口在线。退出。", flush=True)
            return
        # 进入绘图姿态：先让腕/笔旋转关节到位（与 draw_maze_real 预备点一致）
        print("[prep] 进入绘图姿态 (w=-90, h=-45) ...", flush=True)
        try:
            robot.joint(b=0, s=0, e=0, w=-90, h=0, spd=args.spd, acc=args.acc)
            robot.joint(b=0, s=0, e=0, w=-90, h=-45, spd=args.spd, acc=args.acc)
        except OSError as ex:
            print(f"[错误] 预备姿态发送失败：{ex}", flush=True)
            return
    else:
        print("[dry-run] 只求解+打印，不连硬件。确认无误后加 --send。", flush=True)

    print("\n输入笔尖世界坐标（米）：'x y' 或 'x y z'（z 缺省 {:.3f}）。".format(PEN_Z), flush=True)
    print("输入 q / 空行 / Ctrl-C 退出。\n", flush=True)

    q_seed = Q_SEED_DEFAULT.copy()
    try:
        while True:
            try:
                line = input("目标> ").strip()
            except EOFError:
                break
            if line == "" or line.lower() == "q":
                break
            target = parse_coord(line)
            if target is None:
                print("  无法解析。请输入 'x y' 或 'x y z'（米）。", flush=True)
                continue

            if not args.send:
                # dry-run：只对落点求解打印，不发送
                q, _, _ = solve_and_report(target, q_seed)
                q_seed = q
                continue

            result = goto_target(robot, target, q_seed, args.spd, args.acc, args.dt)
            if result is not None:
                q_at, _, _ = result
                input("  [回车] 看完落点后继续，将抬笔再等下一个点 ...")
                lift_pen(robot, q_at, args.spd, args.acc)
                q_seed = q_at  # 用上一解作下一点种子，保持连续
    except KeyboardInterrupt:
        print("\n中断。", flush=True)
    finally:
        if args.send and robot is not None:
            print("[退出] 抬笔回安全姿态 ...", flush=True)
            lift_pen(robot, q_seed, args.spd, args.acc)
    print("结束。", flush=True)


if __name__ == "__main__":
    main()



