"""读取已规划的关节轨迹 JSON，下发给真实机械臂执行（不做规划）。

职责单一：只「读 JSON → 校验限位 → dry-run / 下发实机」。
轨迹由 plan_maze.py 生成（手动点选纸角 → 规划 → IK → 存 outputs/trajectory/trajectory.json）。
本文件同时充当「轨迹相关共享定义」的来源（常量 / load / save / check_limits），plan_maze 复用它。

默认 dry-run（只读+校验，不碰硬件）；--send 才真发。
首次上真机建议先 --max-points 5 试探，确认笔落点/方向无误再发全程。

关节对应 URDF joint_1..5 = 真机 b/s/e/w/h；w 画竖直笔 IK 需约 -93°，会被下位机 clamp 到 -90
（笔约 3° 恒定倾斜，画线无碍）。真机运行前：下位机 robot_server 已启动、纸已就位、笔尖朝下。

    python arm_real_client/draw_maze_real.py                       # dry-run 校验默认轨迹
    python arm_real_client/draw_maze_real.py --send --max-points 5 # 先发前 5 点试探
    python arm_real_client/draw_maze_real.py --send                # 发全程
    python arm_real_client/draw_maze_real.py --traj 别的.json --send
"""

import os
import sys
import json
import argparse

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_client import RobotClient           # noqa: E402
from robot_config import ROBOT_HOST, ROBOT_PORT, DEFAULT_SPD, DEFAULT_ACC, DEFAULT_DT  # noqa: E402

# 规划产物目录（plan_maze.py 写，本脚本读）
OUT_DIR = os.path.join(REPO, "maze_planner", "outputs")
TRAJ_FILE = os.path.join(OUT_DIR, "trajectory", "trajectory.json")  # 关节轨迹
IMAGE_DIR = os.path.join(OUT_DIR, "image")                          # 中间图 + planned.png
PLANNED_FILE = os.path.join(IMAGE_DIR, "planned.png")              # 轨迹投影结果图

# 关节角限位（度），URDF joint_1..5 = b/s/e/w/h。
# w 保守按 ±90：画竖直笔 IK 需约 -93°，超出由下位机 clamp 到 -90（笔约 3° 恒定倾斜）。
JOINT_ORDER = ["b", "s", "e", "w", "h"]
LIMITS_DEG = {"b": (-180, 180), "s": (-90, 90), "e": (-90, 90),
              "w": (-90, 90), "h": (-180, 180)}


def save_trajectory(points, meta, path=TRAJ_FILE):
    """把关节轨迹(points: [{b,s,e,w,h}...]) + 元信息写成 JSON（覆盖）。供 plan_maze 复用。"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump({**meta, "points": points}, f, ensure_ascii=False, indent=2)
    print(f"[traj] 已保存 {len(points)} 点轨迹 -> {path}", flush=True)


def load_trajectory(path=TRAJ_FILE):
    """从 JSON 读关节轨迹，返回 (points, meta)。"""
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    points = data.pop("points")
    print(f"[traj] 读取 {len(points)} 点轨迹 <- {path}", flush=True)
    return points, data


def check_limits(points):
    """打印各关节角度范围 vs 真机限位，返回是否有超限。"""
    deg = np.array([[p[k] for k in JOINT_ORDER] for p in points])
    print("[limit] 各关节角度范围 vs 真机限位:", flush=True)
    over = False
    for i, k in enumerate(JOINT_ORDER):
        lo, hi = LIMITS_DEG[k]
        mn, mx = deg[:, i].min(), deg[:, i].max()
        bad = not (mn >= lo and mx <= hi)
        over = over or bad
        print(f"  {k}: [{mn:8.2f}, {mx:8.2f}]   限位[{lo:4d},{hi:4d}]"
              f"{'   <<< 超限!' if bad else ''}", flush=True)
    return over


def main():
    ap = argparse.ArgumentParser(description="读关节轨迹 JSON 并下发真实机械臂")
    ap.add_argument("--traj", default=TRAJ_FILE,
                    help="关节轨迹 JSON 路径（默认 outputs/trajectory/trajectory.json，由 plan_maze.py 生成）")
    ap.add_argument("--max-points", type=int, default=None, metavar="N",
                    help="只发轨迹的前 N 个点（首次上真机试探用）")
    ap.add_argument("--send", action="store_true",
                    help="真正下发到机械臂（默认 dry-run，只读+校验不碰硬件）")
    ap.add_argument("--host", default=ROBOT_HOST, help="下位机地址（默认取自 robot_config）")
    ap.add_argument("--port", type=int, default=ROBOT_PORT, help="下位机端口（默认取自 robot_config）")
    ap.add_argument("--dt", type=float, default=DEFAULT_DT, help="相邻点时间间隔(s)，默认取自 robot_config")
    ap.add_argument("--spd", type=int, default=DEFAULT_SPD, help="关节角速度(°/s)，默认取自 robot_config")
    ap.add_argument("--acc", type=int, default=DEFAULT_ACC, help="关节角加速度，默认取自 robot_config")
    args = ap.parse_args()

    if not os.path.exists(args.traj):
        print(f"[错误] 找不到轨迹文件: {args.traj}", flush=True)
        print("       请先用 plan_maze.py 规划生成，例如:", flush=True)
        print("       python arm_real_client/plan_maze.py --img 你的迷宫.jpg", flush=True)
        return

    points, meta = load_trajectory(args.traj)
    if meta:
        print(f"[traj] 元信息: {meta}", flush=True)

    if args.max_points is not None:
        points = points[:args.max_points]
        print(f"[traj] 只取前 {len(points)} 个点（--max-points）", flush=True)

    over = check_limits(points)

    if not args.send:
        print(f"[dry-run] 仅读取+校验，未下发。{len(points)} 点 x dt={args.dt}s "
              f"≈ {len(points) * args.dt:.0f}s。确认无误后加 --send。", flush=True)
        return

    if over:
        print("[警告] 轨迹超出真机限位，下位机会 clamp，笔迹会失真。", flush=True)

    print(f"[send] 连接 {args.host}:{args.port} ...", flush=True)
    robot = RobotClient(host=args.host, port=args.port)
    print("[send] ping:", robot.ping(), flush=True)
    resp = robot.trajectory(points, dt=args.dt, traj_id="maze",
                            spd=args.spd, acc=args.acc)
    print("[send] trajectory:", resp, flush=True)
    print("[send] status:", robot.status(), flush=True)


if __name__ == "__main__":
    main()
