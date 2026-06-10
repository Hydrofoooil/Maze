import argparse
import json
import os
import sys
import time
from typing import Any, Dict, List

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "arm_real_client"))
sys.path.insert(0, os.path.join(REPO, "arm_sim"))

from arm_kinematics import ik  # noqa: E402
from force_bias_model import fit, save  # noqa: E402
from robot_client import RobotClient  # noqa: E402
from robot_config import ROBOT_HOST, ROBOT_PORT  # noqa: E402


JOINT_KEYS = ("b", "s", "e", "w", "h")
TRAJ_FILE = os.path.join(
    REPO, "maze_planner", "outputs", "trajectory", "trajectory.json"
)


def load_points(path: str) -> List[Dict[str, Any]]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    points = data.get("points", [])
    if not points:
        raise ValueError(f"no points in {path}")
    return points


def q_to_point(q_rad: np.ndarray, spd: int, acc: int) -> Dict[str, float]:
    deg = np.degrees(q_rad)
    point = {joint: float(deg[i]) for i, joint in enumerate(JOINT_KEYS)}
    point["spd"] = spd
    point["acc"] = acc
    return point


def stable_state(robot: RobotClient, settle: float, samples: int, dt: float):
    time.sleep(max(0.0, settle))
    vals = []
    for _ in range(max(1, samples)):
        resp = robot.state()
        st = resp.get("raw") if resp.get("ok") else None
        if st is not None:
            vals.append(st)
        time.sleep(max(0.0, dt))
    if not vals:
        raise RuntimeError("failed to read state during calibration")
    return vals


def main():
    ap = argparse.ArgumentParser(
        description="Collect no-contact samples and fit tau_bias(q) model."
    )
    ap.add_argument("--trajectory", default=TRAJ_FILE,
                    help="带 x/y/z 的轨迹 JSON，默认 outputs/trajectory/trajectory.json")
    ap.add_argument("--out", default=os.path.join(
        REPO, "maze_planner", "outputs", "trajectory",
        "force_bias_model.json"),
        help="输出姿态零偏模型 JSON")
    ap.add_argument("--host", default=ROBOT_HOST)
    ap.add_argument("--port", type=int, default=ROBOT_PORT)
    ap.add_argument("--lift", type=float, default=0.008,
                    help="相对轨迹 z 抬高距离(m)，默认 8mm，必须确保不接触纸")
    ap.add_argument("--max-points", type=int, default=None,
                    help="最多使用多少个标定姿态")
    ap.add_argument("--stride", type=int, default=1,
                    help="轨迹采样步长，默认每个点都采")
    ap.add_argument("--settle", type=float, default=0.8,
                    help="每个姿态发出后等待稳定时间(s)")
    ap.add_argument("--samples-per-pose", type=int, default=5,
                    help="每个姿态采集状态帧数")
    ap.add_argument("--sample-dt", type=float, default=0.05,
                    help="状态采样间隔(s)")
    ap.add_argument("--spd", type=int, default=25,
                    help="标定移动速度(°/s)")
    ap.add_argument("--acc", type=int, default=20,
                    help="标定移动加速度")
    ap.add_argument("--ridge", type=float, default=1e-6,
                    help="岭回归正则，默认 1e-6")
    ap.add_argument("--dry-run", action="store_true",
                    help="只计算标定姿态，不下发到机械臂")
    args = ap.parse_args()

    points = load_points(args.trajectory)
    points = points[::max(1, args.stride)]
    if args.max_points is not None:
        points = points[:args.max_points]
    if not all(all(k in p for k in ("x", "y", "z")) for p in points):
        raise ValueError("trajectory must be regenerated so every point has x/y/z")

    robot = RobotClient(host=args.host, port=args.port, timeout=8)
    q_seed = np.radians([float(points[0][j]) for j in JOINT_KEYS])
    samples = []

    print(f"[calib] poses={len(points)} lift={args.lift * 1000:.1f}mm "
          f"out={args.out}", flush=True)
    if not args.dry_run:
        print("[calib] ping:", robot.ping(), flush=True)

    for i, point in enumerate(points, start=1):
        target = np.array([
            float(point["x"]),
            float(point["y"]),
            float(point["z"]) + float(args.lift),
        ], dtype=float)
        q_seed, err, tilt = ik(target, q_seed, debug=False)
        cmd = q_to_point(q_seed, args.spd, args.acc)
        print(f"[calib] {i}/{len(points)} target_z={target[2]:.5f} "
              f"ik_err={err * 1000:.2f}mm tilt={tilt:.2f} cmd={cmd}",
              flush=True)

        if args.dry_run:
            continue

        resp = robot.joint(**cmd)
        if not resp.get("ok"):
            raise RuntimeError(f"joint command failed: {resp}")
        samples.extend(stable_state(robot, args.settle,
                                    args.samples_per_pose,
                                    args.sample_dt))

    if args.dry_run:
        print("[calib] dry-run only; no model written", flush=True)
        return

    model = fit(samples, ridge=args.ridge)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    save(model, args.out)
    print(f"[calib] saved model -> {args.out}", flush=True)
    print(f"[calib] n_samples={model['n_samples']} rmse_ncm={model['rmse_ncm']}",
          flush=True)


if __name__ == "__main__":
    main()
