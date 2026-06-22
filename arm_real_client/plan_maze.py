"""手动点选纸角 → 规划迷宫路径 → IK → 生成关节轨迹 JSON（供 draw_maze_real.py 执行）。

把「规划」从「执行」中分离：本脚本只负责
  迷宫照片 -(手动点选4角 / 自动检测 / 给坐标)-> 重建+A*路径 -> 纸面物理坐标
  -> arm_kinematics.ik 解 5-DOF 关节角 -> 存 trajectory.json (+ 中间图 + planned.png)。
不碰硬件。随后用 `draw_maze_real.py --send` 读这个 JSON 下发实机。

选角方式（默认手动点选，最可靠）：
  默认       : 弹窗手动点选 4 角（左键点 4 个角，u/退格撤销，回车确认，Esc 取消）——需图形界面/X11
  --auto     : 自动检测纸角（无头环境用）
  --corners  : 直接给原图坐标 "x1,y1 x2,y2 x3,y3 x4,y4"

    conda activate maze
    python arm_real_client/plan_maze.py --img maze.jpg --paper-cx 0.33     # 手动点选
    python arm_real_client/plan_maze.py --img maze.jpg --auto              # 自动检测纸角
"""

import os
import sys
import argparse

import numpy as np
import cv2

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "maze_planner"))
sys.path.insert(0, os.path.join(REPO, "arm_sim"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from maze_planner import solve_path, parse_corners      # noqa: E402
from arm_kinematics import ik                            # noqa: E402
from draw_maze_real import (                             # noqa: E402  复用执行端的共享定义
    TRAJ_FILE, IMAGE_DIR, PLANNED_FILE, JOINT_ORDER, check_limits, save_trajectory)

# 纸面尺寸（与 arm_sim/draw_maze.py 一致，30cm x 21cm）；前后中心位置由 --paper-cx 调
PAPER_SX, PAPER_SY, PAPER_TOP = 0.30, 0.21, 0.002
PEN_Z = PAPER_TOP + 0.001


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


def plan_joint_traj(img, n_waypoints, paper_cx, paper_cy, debug_dir,
                    manual=True, auto=False, corners=None):
    """迷宫照片 -> 关节角轨迹(rad, N x 5)。返回 (qtraj, 位置残差, 笔轴偏离角)。
    选角：corners 优先 > auto > manual（弹窗）。中间图/planned 图存到 debug_dir。"""
    mode = "corners" if corners is not None else ("auto自动检测" if auto else "manual弹窗点选")
    print(f"[maze] 规划路径: {img}  (选角方式: {mode})", flush=True)
    pts, warped, binary, start_xy, goal_xy = solve_path(
        img, manual=manual, auto=auto, corners=corners, debug_dir=debug_dir)
    himg, wimg = binary.shape[:2]
    path_px = resample(pts, n_waypoints)
    print(f"[maze] 原始 {len(pts)} 点 -> 重采样 {n_waypoints}; 矫正图 {wimg}x{himg}; "
          f"纸面中心 cx={paper_cx} cy={paper_cy}", flush=True)
    print(f"[maze] 中间图 -> {debug_dir}", flush=True)

    # 把规划轨迹投影到矫正(裁剪)后的迷宫彩色图上，存结果图
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


def main():
    ap = argparse.ArgumentParser(description="手动点选纸角→规划迷宫→生成关节轨迹 JSON")
    ap.add_argument("--img",
                    default=os.path.join(REPO, "maze_planner", "samples", "test_0.jpg"),
                    help="迷宫照片（默认 samples/test_0.jpg）")
    ap.add_argument("--n-waypoints", type=int, default=120, help="轨迹点数")
    ap.add_argument("--paper-cx", type=float, default=0.33,
                    help="纸面中心距底座的前向距离(m)，物理上即纸张摆放位置")
    ap.add_argument("--paper-cy", type=float, default=0.0, help="纸面中心横向偏移(m)")
    ap.add_argument("--auto", action="store_true",
                    help="自动检测纸角（默认是弹窗手动点选 4 角，需图形界面）")
    ap.add_argument("--corners", metavar='"x1,y1 x2,y2 x3,y3 x4,y4"',
                    help="直接给原图坐标的 4 个角点，跳过点选")
    args = ap.parse_args()

    corners = parse_corners(args.corners)
    qtraj, res, tilt = plan_joint_traj(
        args.img, args.n_waypoints, args.paper_cx, args.paper_cy, IMAGE_DIR,
        manual=(not args.auto), auto=args.auto, corners=corners)

    print(f"[ik] 位置残差: 最大={res.max() * 1000:.2f}mm 均值={res.mean() * 1000:.2f}mm | "
          f"笔轴偏离竖直: 最大={tilt.max():.2f}° 均值={tilt.mean():.2f}°", flush=True)

    deg = np.degrees(qtraj)             # (N,5)，列依次为 b,s,e,w,h
    points = [{k: float(row[i]) for i, k in enumerate(JOINT_ORDER)} for row in deg]
    save_trajectory(points, {
        "source_image": args.img,
        "paper_cx": args.paper_cx, "paper_cy": args.paper_cy,
        "n_waypoints": args.n_waypoints,
        "ik_residual_mm_max": float(res.max() * 1000),
        "tilt_deg_max": float(tilt.max()),
        "n_points": len(points),
    })

    over = check_limits(points)
    if over:
        print("[警告] 规划轨迹有关节超出真机限位，执行时下位机会 clamp（笔迹会失真）。", flush=True)
    print("[done] 规划完成。接着用 draw_maze_real.py 下发：", flush=True)
    print("       python arm_real_client/draw_maze_real.py --send --max-points 5  # 先试探", flush=True)
    print("       python arm_real_client/draw_maze_real.py --send                 # 发全程", flush=True)


if __name__ == "__main__":
    main()
