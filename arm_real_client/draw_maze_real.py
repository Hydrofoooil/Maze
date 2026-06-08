"""
把 maze_planner 规划的迷宫路径转成关节轨迹，下发给真实机械臂。

完整链路（复用现有模块，不重新造轮子）：
  maze_planner.solve_path(迷宫照片)  -> 路径(矫正图像素坐标)
  -> 按弧长重采样 -> img_to_world      -> 纸面物理坐标(与 arm_sim/draw_maze.py 完全一致)
  -> arm_kinematics.ik 逐点求关节角(rad)
  -> 转角度 + 映射到 b/s/e/w/h + 限位检查
  -> 存成轨迹文件(outputs/trajectory/trajectory.json，覆盖旧的)
  -> arm_real_client.RobotClient.trajectory 下发

两种运行方式：
  - 规划模式(默认)：现场规划 -> 存轨迹文件 -> dry-run / 下发
  - 读取模式(--from-file)：直接读已存的轨迹文件下发，跳过规划+IK

默认是 dry-run：只规划+校验+存文件，不连硬件、不动机械臂。加 --send 才真发。
首次上真机建议先 --max-points 1（或几）试探，确认笔落点/方向无误再发全程。

关节对应（URDF joint_1..5 = 真机 b/s/e/w/h），单位换算 rad -> deg：
  本机的夹爪已被拆除、换成与笔固定的连接件，所以第 5 关节(h)现在是「笔旋转关节」，
  和仿真 URDF 的 joint_5 完全一致 —— 因此 5-DOF IK 的解可直接使用，h 不再是 ±45 夹爪。
  ⚠ 下位机 robot_server.py 的软限位 h 必须放宽（已改），否则笔旋转会被 clamp。

真机运行前提：
  - 下位机 serial_bridge.py / robot_server.py 已启动，9001 portproxy 已配置
  - 一张纸平放在底座正前方，中心距底座 --paper-cx 米、尺寸 30x21cm（与仿真一致）
  - 笔已固定在末端连接件上、笔尖朝下
"""

import os
import sys
import json
import argparse

import numpy as np
import cv2

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "maze_planner"))
sys.path.insert(0, os.path.join(REPO, "arm_sim"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from maze_planner import solve_path           # noqa: E402
from arm_kinematics import ik                  # noqa: E402
from robot_client import RobotClient           # noqa: E402
from robot_config import ROBOT_HOST, ROBOT_PORT, DEFAULT_SPD, DEFAULT_ACC  # noqa: E402

# 规划产物目录（每次覆盖）
OUT_DIR = os.path.join(REPO, "maze_planner", "outputs")
TRAJ_FILE = os.path.join(OUT_DIR, "trajectory", "trajectory.json")  # 关节轨迹
IMAGE_DIR = os.path.join(OUT_DIR, "image")                          # debug 中间图
PLANNED_FILE = os.path.join(IMAGE_DIR, "planned.png")              # 轨迹投影在矫正迷宫上的结果图

# 纸面尺寸：与 arm_sim/draw_maze.py 一致（30cm x 21cm）。中心前后位置可由 --paper-cx 调
PAPER_SX, PAPER_SY, PAPER_TOP = 0.30, 0.21, 0.002
PEN_Z = PAPER_TOP + 0.001

# 关节角限位，单位度。URDF joint_1..5 依次对应 b/s/e/w/h。
#   h 已从 ±45 夹爪改装为笔旋转关节，按舵机行程放宽到 ±180（与 URDF 一致）。
# w 保守按文档 ±90：IK 需约 -93°，超出会被下位机 clamp 到 -90（笔约 3° 恒定倾斜）。
# 确认手腕舵机物理能转过 ±90 后，可把这里和 robot_server 一起放宽到 ±95。
JOINT_ORDER = ["b", "s", "e", "w", "h"]
LIMITS_DEG = {"b": (-180, 180), "s": (-90, 90), "e": (-90, 90),
              "w": (-90, 90), "h": (-180, 180)}


def img_to_world(px, py, wimg, himg, paper_cx, paper_cy):
    """矫正图像素 (px,py) -> 纸面世界坐标 (与 draw_maze.py 的纹理 UV 一致)。"""
    u, v = px / wimg, py / himg
    return (paper_cx + (0.5 - v) * PAPER_SX,
            paper_cy + (u - 0.5) * PAPER_SY, PEN_Z)


def resample(pts, n):
    """按弧长把折线重采样成 n 个等距点。"""
    P = np.asarray(pts, float)
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    s = np.linspace(0, d[-1], n)
    return np.c_[np.interp(s, d, P[:, 0]), np.interp(s, d, P[:, 1])]


def plan_joint_traj(img, n_waypoints, paper_cx, paper_cy, debug_dir):
    """迷宫照片 -> 关节角轨迹(rad, N x 5)。返回 (qtraj, 位置残差, 笔轴偏离角)。
    debug_dir: 各步骤中间图保存目录（覆盖上一次）。"""
    print(f"[maze] 规划路径: {img}", flush=True)
    pts, warped, binary, start_xy, goal_xy = solve_path(img, auto=True, debug_dir=debug_dir)
    himg, wimg = binary.shape[:2]
    path_px = resample(pts, n_waypoints)
    print(f"[maze] 原始 {len(pts)} 点 -> 重采样 {n_waypoints}; 矫正图 {wimg}x{himg}; "
          f"纸面中心 cx={paper_cx} cy={paper_cy}", flush=True)
    print(f"[maze] 中间图 -> {debug_dir}", flush=True)

    # 把实际下发的轨迹投影到矫正(裁剪)后的迷宫彩色图上，保存为结果图
    vis = warped.copy()
    for i in range(1, len(path_px)):
        p0 = (int(path_px[i - 1][0]), int(path_px[i - 1][1]))
        p1 = (int(path_px[i][0]), int(path_px[i][1]))
        cv2.line(vis, p0, p1, (255, 0, 255), 3)
    cv2.circle(vis, (int(start_xy[0]), int(start_xy[1])), 10, (0, 0, 220), -1)  # 起点红
    cv2.circle(vis, (int(goal_xy[0]), int(goal_xy[1])), 10, (220, 0, 0), -1)    # 终点蓝
    os.makedirs(os.path.dirname(PLANNED_FILE), exist_ok=True)
    cv2.imwrite(PLANNED_FILE, vis)
    print(f"[plan] 轨迹投影图 -> {PLANNED_FILE}", flush=True)

    qtraj, res, tilt = [], [], []
    q_seed = np.array([0.0, 1.0, 1.0, 0.0, 0.0])   # 朝前下方折叠的初始猜测
    for px, py in path_px:
        tgt = np.array(img_to_world(px, py, wimg, himg, paper_cx, paper_cy))
        q_seed, e, ti = ik(tgt, q_seed)
        qtraj.append(q_seed.copy())
        res.append(e)
        tilt.append(ti)
    return np.array(qtraj), np.array(res), np.array(tilt)


def save_trajectory(points, meta, path=TRAJ_FILE):
    """把关节轨迹(points: [{b,s,e,w,h}...]) + 元信息写成 JSON（覆盖）。"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump({**meta, "points": points}, f, ensure_ascii=False, indent=2)
    print(f"[traj] 已保存 {len(points)} 点轨迹 -> {path}", flush=True)


def load_trajectory(path=TRAJ_FILE):
    """从 JSON 读关节轨迹，返回 (points, meta)。"""
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    points = data.pop("points")
    print(f"[traj] 从文件读取 {len(points)} 点轨迹 <- {path}", flush=True)
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
    ap = argparse.ArgumentParser(description="把迷宫规划路径下发给真实机械臂")
    ap.add_argument("--img",
                    default=os.path.join(REPO, "maze_planner", "samples", "test_0.jpg"),
                    help="迷宫照片（默认 samples/test_0.jpg）")
    ap.add_argument("--n-waypoints", type=int, default=120, help="轨迹点数")
    ap.add_argument("--paper-cx", type=float, default=0.22,
                    help="纸面中心距底座的前向距离(m)，物理上即纸张摆放位置")
    ap.add_argument("--paper-cy", type=float, default=0.0, help="纸面中心横向偏移(m)")
    ap.add_argument("--from-file", nargs="?", const=TRAJ_FILE, default=None,
                    metavar="JSON",
                    help="不重新规划，直接读已存轨迹文件下发（默认读 outputs/trajectory/trajectory.json）")
    ap.add_argument("--max-points", type=int, default=None, metavar="N",
                    help="只发轨迹的前 N 个点（首次上真机试探用）")
    ap.add_argument("--send", action="store_true",
                    help="真正下发到机械臂（默认 dry-run，只规划+校验不碰硬件）")
    ap.add_argument("--host", default=ROBOT_HOST, help="下位机地址（默认取自 robot_config）")
    ap.add_argument("--port", type=int, default=ROBOT_PORT, help="下位机端口（默认取自 robot_config）")
    ap.add_argument("--dt", type=float, default=0.3, help="相邻点时间间隔(s)")
    ap.add_argument("--spd", type=int, default=DEFAULT_SPD, help="关节角速度(°/s)，默认取自 robot_config")
    ap.add_argument("--acc", type=int, default=DEFAULT_ACC, help="关节角加速度，默认取自 robot_config")
    args = ap.parse_args()

    if args.from_file is not None:
        points, meta = load_trajectory(args.from_file)
    else:
        qtraj, res, tilt = plan_joint_traj(args.img, args.n_waypoints,
                                           args.paper_cx, args.paper_cy, IMAGE_DIR)
        print(f"[ik] 位置残差: 最大={res.max() * 1000:.2f}mm 均值={res.mean() * 1000:.2f}mm | "
              f"笔轴偏离竖直: 最大={tilt.max():.2f}° 均值={tilt.mean():.2f}°", flush=True)
        deg = np.degrees(qtraj)             # (N,5)，列依次为 b,s,e,w,h
        points = [{k: float(row[i]) for i, k in enumerate(JOINT_ORDER)} for row in deg]
        save_trajectory(points, {
            "source_image": args.img,
            "paper_cx": args.paper_cx, "paper_cy": args.paper_cy,
            "n_waypoints": args.n_waypoints,
            "dt": args.dt, "spd": args.spd, "acc": args.acc,
            "ik_residual_mm_max": float(res.max() * 1000),
            "tilt_deg_max": float(tilt.max()),
            "n_points": len(points),
        })

    if args.max_points is not None:
        points = points[:args.max_points]
        print(f"[traj] 只取前 {len(points)} 个点（--max-points）", flush=True)

    over = check_limits(points)

    if not args.send:
        print(f"[dry-run] 仅规划+校验，未下发。预计 {len(points)} 点 x dt={args.dt}s "
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
