"""
真实机械臂下位机服务（Windows 直连串口版）。

直接跑在下位机 Windows 上：机械臂经 USB-C 接 Windows、识别为 COM4，本服务用 pyserial
直接读写 COM4，并通过 TCP 对上位机提供 ping/joint/trajectory/status/state/stop 接口。

相比旧的「WSL robot_server + Windows serial_bridge + portproxy」三层架构，这里把串口和
TCP 服务合并到 Windows 一个进程，去掉了 WSL<->Windows 的 TCP 中转和 portproxy，延迟更低，
也不再有 bridge 卡死/重连那些坑。serial_bridge.py 不再需要。

运行（下位机 Windows，先 pip install pyserial）：
    python robot_server.py
上位机经 tailscale IP:9001 连接（见 arm_real_client/robot_config.py，端口沿用 9001）。
记得 Windows 防火墙放行 9001；portproxy 不再需要，可删。
"""

import socket
import json
import time
import math
import threading
import argparse
import os
import sys
from collections import deque
from typing import Dict, Any, List, Optional

import numpy as np
import serial

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "arm_sim"))

from arm_kinematics import fk_pos, ik, jacobian_pos  # noqa: E402
from force_bias_model import load as load_bias_model, predict as predict_bias  # noqa: E402

# ---- 对上位机的 TCP 服务 ----
SERVER_HOST = "127.0.0.1"
SERVER_PORT = 9001          # 直接监听对外端口（不再经 portproxy 9001->9000）

# ---- 机械臂串口 ----
SERIAL_PORT = "COM10"
BAUD = 115200

LIMITS = {
    "b": (-180.0, 180.0),
    "s": (-90.0, 90.0),
    "e": (-90.0, 90.0),
    # 手腕舵机文档行程 ±90。画竖直笔的 IK 解需约 -93°，超 ±90 在此 clamp 到 -90
    # （笔约 3° 恒定倾斜，画线可接受），避免顶机械挡块堵转。确认能转过 ±90 可放宽。
    "w": (-90.0, 90.0),
    # 夹爪已拆除、改装为与笔固定的旋转连接件，按舵机行程放宽（原装夹爪为 ±45）。
    "h": (-180.0, 180.0),
}

DEFAULT_SPD = 10
DEFAULT_ACC = 10
DEFAULT_INIT_SPD = 25
DEFAULT_INIT_ACC = 20
DEFAULT_XYZ_SPD = 0.25
DEFAULT_ELASTIC_ON = {"b": 60, "s": 100, "e": 35, "w": 35, "h": 35}
DEFAULT_ELASTIC_OFF = {"b": 1000, "s": 1000, "e": 1000, "w": 1000, "h": 1000}
STREAM_HZ = 20.0
STREAM_DT = 1.0 / STREAM_HZ
STATE_QUERY_TIMEOUT = 0.02
PLOT_WINDOW_SEC = 5.0
FORCE_Z_DEFAULT = {
    "enabled": False,
    "target": 0.7,
    "gain": 0.0005,
    "z_min": -0.002,
    "z_max": 0.004,
    "z_step": 0.00015,
    "alpha": 0.2,
    "max_force": 5.0,
    "safety_stop": False,
    "sign": 1.0,
    "bias_samples": 20,
    "tolerance": 0.15,
}

# 到位轮询参数（闭环：发一个点 -> 轮询 T=105 直到角度收敛=停止 -> 再发下一个）
ANGLE_TOL_DEG = 2.0      # 仅诊断显示用（绝对角度误差）；到位判据用下面的「角度收敛」
POLL_INTERVAL = 0.05     # 两次状态查询的间隔(s)
POINT_TIMEOUT = 12.0     # 单点最大等待(s)，超时则放弃等待、继续下一点
                         # 注意 spd 单位 °/s：spd=10 走 90° 要 9s，大角度移动需放宽此值或调大 spd
MIN_SETTLE = 0.8         # 目标≈当前(几乎不动)时，至少等这么久再判定到位
STABLE_DELTA = 0.6       # 相邻两次状态各关节变化都 < 此值，视为"这一刻没在动"
STABLE_NEEDED = 4        # 连续这么多次"没在动"判定为已停止到位（× POLL_INTERVAL ≈ 确认时长）
JOINT_KEYS = ("b", "s", "e", "w", "h")
CHECK_KEYS = ("b", "s", "e", "w")   # 到位判据只看这 4 个；h(笔旋转件)返回字段 t 零位存疑、画图非关键
FORCE_JOINT_IDXS = np.array([0, 2, 3, 4], dtype=int)  # torS is invalid; use b/e/w/h only.
TORQUE_NCM_TO_NM = 0.01
_RAD2DEG = 180.0 / math.pi

state_lock = threading.Lock()
exec_lock = threading.Lock()
stop_event = threading.Event()
telemetry_stop_event = threading.Event()
ser_lock = threading.Lock()         # 串口读写串行化（trajectory 线程与 state 请求可能并发）

server_state = {
    "status": "idle",        # idle / running / done / stopped / error
    "traj_id": None,
    "current_index": 0,
    "total_points": 0,
    "last_error": None,
    "last_result": None,
}

ser: Optional[serial.Serial] = None
plotter = None
telemetry_thread = None
force_z_config = dict(FORCE_Z_DEFAULT)
force_torque_bias = np.zeros(5, dtype=float)
force_bias_model = None
init_motion_config = {"spd": DEFAULT_INIT_SPD, "acc": DEFAULT_INIT_ACC}
motion_config = {
    "mode": "joint",
    "xyz_axis_map": ("x", "y", "z"),
    "xyz_offset": np.zeros(3, dtype=float),
    "xyz_scale": np.ones(3, dtype=float) * 1000.0,
    "xyz_spd": DEFAULT_XYZ_SPD,
}


class RealtimePlotter:
    def __init__(self, window_sec=PLOT_WINDOW_SEC):
        self.window_sec = float(window_sec)
        self.lock = threading.Lock()
        self.series = {
            "target_joints": deque(maxlen=2000),
            "feedback_joints": deque(maxlen=2000),
            "xyz": deque(maxlen=2000),
            "torque": deque(maxlen=2000),
            "tip_force": deque(maxlen=2000),
        }
        self.colors = {
            "b": "#d62728",
            "s": "#1f77b4",
            "e": "#2ca02c",
            "w": "#ff7f0e",
            "h": "#9467bd",
        }
        self.zoom = {"joints": 1.0, "xyz": 1.0, "torque": 1.0, "force": 1.0}
        self.active_tab = "joints"
        self.ready = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def add(self, kind, t, data):
        with self.lock:
            self.series[kind].append((float(t), dict(data)))

    def _run(self):
        try:
            import tkinter as tk
            from tkinter import ttk
        except Exception as exc:
            print(f"[PLOT] disabled: {exc}", flush=True)
            return

        try:
            self.tk = tk
            self.root = tk.Tk()
            self.root.title("Robot telemetry - 20Hz / 5s window")
            controls = ttk.Frame(self.root)
            controls.pack(fill="x")
            self.stop_button = ttk.Button(
                controls,
                text="Stop receiving",
                command=self.toggle_receiving,
            )
            self.stop_button.pack(side="left", padx=8, pady=6)
            notebook = ttk.Notebook(self.root)
            notebook.pack(fill="both", expand=True)
            self.canvases = {}
            for kind, title in (
                ("joints", "Joint angles"),
                ("xyz", "XYZ"),
                ("torque", "Torque"),
                ("force", "Tip force"),
            ):
                frame = ttk.Frame(notebook)
                canvas = tk.Canvas(frame, width=900, height=420, bg="white")
                canvas.pack(fill="both", expand=True)
                canvas.bind("<Enter>", lambda _e, k=kind: self._set_active_tab(k))
                canvas.bind("<MouseWheel>", self._on_mousewheel)
                canvas.bind("<Button-4>", self._on_mousewheel)
                canvas.bind("<Button-5>", self._on_mousewheel)
                notebook.add(frame, text=title)
                self.canvases[kind] = canvas
            self.ready.set()
            self.root.after(50, self._redraw)
            self.root.mainloop()
        except Exception as exc:
            print(f"[PLOT] disabled: {exc}", flush=True)

    def _redraw(self):
        now = time.perf_counter()
        with self.lock:
            data = {k: list(v) for k, v in self.series.items()}
        self._draw_joint_canvas(self.canvases["joints"],
                                data["target_joints"],
                                data["feedback_joints"], now)
        self._draw_series_canvas(self.canvases["xyz"], data["xyz"], now,
                                 "XYZ feedback", ("x", "y", "z"),
                                 {"x": "#d62728", "y": "#1f77b4", "z": "#2ca02c"})
        self._draw_series_canvas(self.canvases["torque"], data["torque"], now,
                                 "Torque feedback",
                                 ("torB", "torS", "torE", "torW", "torH"),
                                 {
                                     "torB": "#d62728",
                                     "torS": "#1f77b4",
                                     "torE": "#2ca02c",
                                     "torW": "#ff7f0e",
                                     "torH": "#9467bd",
                                 })
        self._draw_series_canvas(self.canvases["force"], data["tip_force"], now,
                                 "Tip force",
                                 ("fz", "fz_filtered", "target",
                                  "z_offset_mm", "rejected"),
                                 {
                                     "fz": "#d62728",
                                     "fz_filtered": "#1f77b4",
                                     "target": "#2ca02c",
                                     "z_offset_mm": "#9467bd",
                                     "rejected": "#8c564b",
                                 },
                                 kind="force")
        self.root.after(50, self._redraw)

    def toggle_receiving(self):
        if telemetry_stop_event.is_set():
            start_telemetry_receiver()
            self.stop_button.config(text="Stop receiving")
        else:
            telemetry_stop_event.set()
            self.stop_button.config(text="Resume receiving")

    def _set_active_tab(self, kind):
        self.active_tab = kind

    def _on_mousewheel(self, event):
        kind = self.active_tab
        direction = 1
        if hasattr(event, "delta") and event.delta:
            direction = 1 if event.delta > 0 else -1
        elif getattr(event, "num", None) == 5:
            direction = -1
        factor = 0.8 if direction > 0 else 1.25
        self.zoom[kind] = min(20.0, max(0.2, self.zoom[kind] * factor))

    def _draw_grid(self, canvas, x0, y0, x1, y1, y_min, y_max):
        for i in range(6):
            x = x0 + i * (x1 - x0) / 5
            canvas.create_line(x, y0, x, y1, fill="#f1f1f1")
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            yy = y1 - frac * (y1 - y0)
            val = y_min + frac * (y_max - y_min)
            canvas.create_line(x0, yy, x1, yy, fill="#e8e8e8")
            canvas.create_text(4, yy, anchor="w", text=f"{val:.1f}",
                               fill="#666666")

    def _axis_scale(self, kind, vals):
        y_min, y_max = min(vals), max(vals)
        if abs(y_max - y_min) < 1e-6:
            y_min -= 1.0
            y_max += 1.0
        center = 0.5 * (y_min + y_max)
        half = 0.5 * (y_max - y_min) * 1.15 * self.zoom[kind]
        return center - half, center + half

    def _draw_base(self, canvas, now, title, visible_list, keys, row, nrows,
                   kind):
        w = max(canvas.winfo_width(), 200)
        h = max(canvas.winfo_height(), 160)
        top_pad, bottom_pad, gap = 28, 28, 18
        avail_h = h - top_pad - bottom_pad - gap * (nrows - 1)
        plot_h = max(70, avail_h / max(1, nrows))
        margin_l, margin_r = 54, 18
        x0 = margin_l
        x1 = w - margin_r
        y0 = top_pad + row * (plot_h + gap)
        y1 = y0 + plot_h

        canvas.create_rectangle(x0, y0, x1, y1, outline="#cccccc")
        canvas.create_text(x0, y0 - 16, anchor="nw", text=title,
                           fill="#333333")

        if not any(visible_list):
            canvas.create_text((x0 + x1) / 2, (y0 + y1) / 2,
                               text="waiting for data", fill="#777777")
            return None

        vals = []
        for visible in visible_list:
            for _, d in visible:
                vals.extend(float(d[k]) for k in keys if k in d)
        if not vals:
            canvas.create_text((x0 + x1) / 2, (y0 + y1) / 2,
                               text="waiting for data", fill="#777777")
            return None

        y_min, y_max = self._axis_scale(kind, vals)
        self._draw_grid(canvas, x0, y0, x1, y1, y_min, y_max)

        t_min = now - self.window_sec

        def sx(t):
            return x0 + (t - t_min) / self.window_sec * (x1 - x0)

        def sy(v):
            return y1 - (float(v) - y_min) / (y_max - y_min) * (y1 - y0)

        return sx, sy, h, x0

    def _visible(self, samples, now):
        t_min = now - self.window_sec
        return [(t, d) for t, d in samples if t >= t_min]

    def _draw_series_canvas(self, canvas, samples, now, title, keys, colors,
                            kind=None):
        canvas.delete("all")
        visible = self._visible(samples, now)
        nrows = len(keys)
        draw_kind = kind or ("xyz" if title.startswith("XYZ") else "torque")
        for idx, key in enumerate(keys):
            base = self._draw_base(canvas, now, f"{title}: {key}",
                                   [visible], (key,), idx, nrows,
                                   draw_kind)
            if base is None:
                continue
            sx, sy, h, x0 = base
            pts = []
            for t, d in visible:
                if key in d:
                    pts.extend((sx(t), sy(d[key])))
            if len(pts) >= 4:
                canvas.create_line(*pts, fill=colors[key], width=2)

    def _draw_joint_canvas(self, canvas, target_samples, feedback_samples, now):
        canvas.delete("all")
        target_visible = self._visible(target_samples, now)
        feedback_visible = self._visible(feedback_samples, now)
        nrows = len(JOINT_KEYS)
        for idx, key in enumerate(JOINT_KEYS):
            base = self._draw_base(canvas, now, f"Joint {key}: target + feedback",
                                   [target_visible, feedback_visible], (key,),
                                   idx, nrows, "joints")
            if base is None:
                continue
            sx, sy, h, x0 = base
            for samples, dash in ((target_visible, None), (feedback_visible, (4, 3))):
                pts = []
                for t, d in samples:
                    if key in d:
                        pts.extend((sx(t), sy(d[key])))
                if len(pts) >= 4:
                    opts = {"fill": self.colors[key], "width": 2}
                    if dash is not None:
                        opts["dash"] = dash
                    canvas.create_line(*pts, **opts)
            lx = x0 + 10
            y_legend = max(10, (idx + 1) * max(70, (h - 56) / nrows) + 2)
            canvas.create_line(lx, y_legend, lx + 20, y_legend,
                               fill=self.colors[key], width=2)
            canvas.create_text(lx + 24, y_legend, anchor="w",
                               text="target", fill="#333333")
            canvas.create_line(lx + 90, y_legend, lx + 110, y_legend,
                               fill=self.colors[key], width=2, dash=(4, 3))
            canvas.create_text(lx + 114, y_legend, anchor="w",
                               text="feedback", fill="#333333")


def start_plotter():
    global plotter
    if plotter is None:
        plotter = RealtimePlotter()
    return plotter


def telemetry_worker():
    p = start_plotter()
    next_t = time.perf_counter()
    print(f"[RX20] telemetry receiving at {STREAM_HZ:.1f}Hz", flush=True)
    while not telemetry_stop_event.is_set():
        now = time.perf_counter()
        st = query_state(timeout=STATE_QUERY_TIMEOUT)
        if st is not None:
            p.add("feedback_joints", now, _state_to_deg(st))
            p.add("xyz", now, {
                "x": float(st.get("x", 0.0)),
                "y": float(st.get("y", 0.0)),
                "z": float(st.get("z", 0.0)),
            })
            p.add("torque", now, {
                "torB": float(st.get("torB", 0.0)),
                "torS": float(st.get("torS", 0.0)),
                "torE": float(st.get("torE", 0.0)),
                "torW": float(st.get("torW", 0.0)),
                "torH": float(st.get("torH", 0.0)),
            })
            if force_z_config.get("enabled", False):
                try:
                    p.add("tip_force", now, {
                        "fz": float(force_z_config["sign"] *
                                    estimate_tip_fz(_state_to_q_rad(st), st)),
                        "target": float(force_z_config["target"]),
                    })
                except Exception as exc:
                    print(f"[FORCE] telemetry estimate skipped: {exc}", flush=True)
        next_t += STREAM_DT
        wait_s = next_t - time.perf_counter()
        if wait_s > 0:
            telemetry_stop_event.wait(wait_s)
        else:
            next_t = time.perf_counter()
    print("[RX20] telemetry receiving stopped; plot frozen", flush=True)


def start_telemetry_receiver():
    global telemetry_thread
    if telemetry_thread is None or not telemetry_thread.is_alive():
        telemetry_stop_event.clear()
        telemetry_thread = threading.Thread(target=telemetry_worker, daemon=True)
        telemetry_thread.start()
    return telemetry_thread


def open_serial():
    global ser
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.05)
    time.sleep(2)
    print(f"[OK] Opened serial {SERIAL_PORT} @ {BAUD}", flush=True)


def set_state(**kwargs):
    with state_lock:
        server_state.update(kwargs)


def get_state():
    with state_lock:
        return dict(server_state)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(x)))


def mapped_robot_xyz(world_xyz: np.ndarray) -> np.ndarray:
    """Map our meter-based world xyz to the robot firmware XYZ command frame."""
    values = {
        "x": float(world_xyz[0]),
        "y": float(world_xyz[1]),
        "z": float(world_xyz[2]),
    }
    mapped = []
    for token in motion_config["xyz_axis_map"]:
        sign = -1.0 if token.startswith("-") else 1.0
        axis = token[1:] if token.startswith("-") else token
        mapped.append(sign * values[axis])
    return motion_config["xyz_offset"] + motion_config["xyz_scale"] * \
        np.asarray(mapped, dtype=float)


def make_xyz_cmd(world_xyz: np.ndarray, spd=None) -> Dict[str, Any]:
    xyz = mapped_robot_xyz(np.asarray(world_xyz, dtype=float))
    return {
        "T": 104,
        "x": float(xyz[0]),
        "y": float(xyz[1]),
        "z": float(xyz[2]),
        "spd": float(spd if spd is not None else motion_config["xyz_spd"]),
    }


def make_arm_cmd(point: Dict[str, Any], global_spd=None, global_acc=None) -> Dict[str, Any]:
    cmd = {"T": 122}
    for joint, (lo, hi) in LIMITS.items():
        value = point.get(joint, 0.0)
        cmd[joint] = clamp(value, lo, hi)
    cmd["spd"] = point.get("spd", global_spd if global_spd is not None else DEFAULT_SPD)
    cmd["acc"] = point.get("acc", global_acc if global_acc is not None else DEFAULT_ACC)
    return cmd


# ----------------------------------------------------------------------------
# 串口收发
# ----------------------------------------------------------------------------
def _extract_state(buf: bytes) -> Optional[Dict[str, Any]]:
    """从字节缓冲里按行找出机械臂状态返回（T==1051）。"""
    for line in buf.split(b"\n"):
        line = line.strip()
        if not line:
            continue
        try:
            obj = json.loads(line.decode("utf-8", "ignore"))
        except (ValueError, UnicodeDecodeError):
            continue
        if isinstance(obj, dict) and obj.get("T") == 1051:
            return obj
    return None


def _state_to_deg(st: Dict[str, Any]) -> Dict[str, float]:
    """把 T=1051 状态返回（弧度）换算成 b/s/e/w/h（度）。
    注意：返回里第 5 关节字段名是 't'（说明书 6.7），对应控制指令里的 h。
    另：s(肩,步进电机)返回符号与指令相反，但实际动作方向正确——这里不修正符号，
    到位判据用「角度收敛(是否还在变)」，不受符号影响。"""
    return {
        "b": float(st.get("b", 0.0)) * _RAD2DEG,
        "s": float(st.get("s", 0.0)) * _RAD2DEG,
        "e": float(st.get("e", 0.0)) * _RAD2DEG,
        "w": float(st.get("w", 0.0)) * _RAD2DEG,
        "h": float(st.get("t", 0.0)) * _RAD2DEG,
    }


def _state_to_q_rad(st: Dict[str, Any]) -> np.ndarray:
    deg = _state_to_deg(st)
    return np.radians([deg[joint] for joint in JOINT_KEYS])


def _torque_vec(st: Dict[str, Any]) -> np.ndarray:
    return np.array([
        float(st.get("torB", 0.0)),
        float(st.get("torS", 0.0)),
        float(st.get("torE", 0.0)),
        float(st.get("torW", 0.0)),
        float(st.get("torH", 0.0)),
    ], dtype=float)


def predict_force_torque_bias(q_rad: np.ndarray) -> np.ndarray:
    if force_bias_model is not None:
        return predict_bias(force_bias_model, q_rad)
    return force_torque_bias


def _q_rad_to_point(q: np.ndarray) -> Dict[str, float]:
    deg = np.degrees(q)
    return {joint: float(deg[i]) for i, joint in enumerate(JOINT_KEYS)}


def _point_to_q_rad(point: Dict[str, Any]) -> np.ndarray:
    return np.radians([float(point[joint]) for joint in JOINT_KEYS])


def _point_target_xyz(point: Dict[str, Any], fallback_q: np.ndarray) -> np.ndarray:
    if all(k in point for k in ("x", "y", "z")):
        return np.array([float(point["x"]), float(point["y"]), float(point["z"])],
                        dtype=float)
    return fk_pos(fallback_q)


def estimate_tip_fz(q_rad: np.ndarray, st: Dict[str, Any]) -> float:
    """Estimate only pen-tip vertical force from torque feedback.

    Servo torque feedback is reported in N*cm. Convert to N*m before applying
    tau_i ~= Jz_i * Fz. torS is known invalid, so the least-squares estimate
    uses only b/e/w/h.
    """
    tau_ext_nm = (_torque_vec(st) - predict_force_torque_bias(q_rad)) * \
        TORQUE_NCM_TO_NM
    jz = jacobian_pos(q_rad)[2, :]
    jz_valid = jz[FORCE_JOINT_IDXS]
    tau_valid = tau_ext_nm[FORCE_JOINT_IDXS]
    denom = float(jz_valid @ jz_valid)
    if denom < 1e-12:
        return 0.0
    return float((jz_valid @ tau_valid) / denom)


def calibrate_force_torque_bias(samples: int):
    global force_torque_bias
    vals = []
    samples = max(1, int(samples))
    print(f"[FORCE] calibrating torque bias with {samples} samples", flush=True)
    for _ in range(samples):
        st = query_state(timeout=0.2)
        if st is not None:
            vals.append(_torque_vec(st))
        time.sleep(0.03)
    if vals:
        force_torque_bias = np.mean(np.vstack(vals), axis=0)
        print(f"[FORCE] torque bias N*cm={force_torque_bias.round(4).tolist()} "
              f"(torS ignored for Fz)",
              flush=True)
    else:
        force_torque_bias = np.zeros(5, dtype=float)
        print("[FORCE] warning: no state during bias calibration; using zero bias",
              flush=True)


class ForceZController:
    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.z_offset = 0.0
        self.fz_filtered = 0.0
        self.last_t = None
        self.last_reject_warn_t = 0.0

    def update(self, q_rad: np.ndarray, st: Dict[str, Any],
               allow_z_update: bool) -> Dict[str, float]:
        now = time.perf_counter()
        dt = POLL_INTERVAL if self.last_t is None else max(1e-3, now - self.last_t)
        self.last_t = now

        fz = float(self.cfg["sign"] * estimate_tip_fz(q_rad, st))
        err = self.fz_filtered - float(self.cfg["target"])
        dz = 0.0
        rejected = False
        if allow_z_update:
            if abs(fz) > float(self.cfg["max_force"]):
                rejected = True
                msg = f"reject implausible stable raw Fz: {fz:.3f} N"
                if self.cfg.get("safety_stop", False):
                    raise RuntimeError(msg)
                if now - self.last_reject_warn_t > 1.0:
                    print(f"[FORCE] warning: {msg}", flush=True)
                    self.last_reject_warn_t = now
                return {
                    "fz": fz,
                    "fz_filtered": self.fz_filtered,
                    "target": float(self.cfg["target"]),
                    "force_error": err,
                    "z_update_enabled": False,
                    "measurement_rejected": True,
                    "z_step_mm": 0.0,
                    "z_offset": self.z_offset,
                    "z_offset_mm": self.z_offset * 1000.0,
                }
            alpha = float(self.cfg["alpha"])
            self.fz_filtered = (1.0 - alpha) * self.fz_filtered + alpha * fz
            err = self.fz_filtered - float(self.cfg["target"])
            dz = float(self.cfg["gain"]) * err * dt
            dz = clamp(dz, -float(self.cfg["z_step"]), float(self.cfg["z_step"]))
            self.z_offset = clamp(
                self.z_offset + dz,
                float(self.cfg["z_min"]),
                float(self.cfg["z_max"]),
            )
        return {
            "fz": fz,
            "fz_filtered": self.fz_filtered,
            "target": float(self.cfg["target"]),
            "force_error": err,
            "z_update_enabled": bool(allow_z_update),
            "measurement_rejected": rejected,
            "z_step_mm": dz * 1000.0,
            "z_offset": self.z_offset,
            "z_offset_mm": self.z_offset * 1000.0,
        }

    def force_ready(self) -> bool:
        return abs(self.fz_filtered - float(self.cfg["target"])) <= \
            float(self.cfg["tolerance"])


def send_serial(cmd: Dict[str, Any]):
    """把一条指令写入串口（线程安全）。"""
    msg = json.dumps(cmd, separators=(",", ":")) + "\n"
    with ser_lock:
        ser.write(msg.encode("utf-8"))


def make_elastic_cmd(enabled: bool, values: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Build T=112 elastic-adaptive output command from the product manual."""
    defaults = DEFAULT_ELASTIC_ON if enabled else DEFAULT_ELASTIC_OFF
    src = values or defaults
    cmd = {"T": 112, "mode": 1 if enabled else 0}
    for joint in JOINT_KEYS:
        value = int(src.get(joint, defaults[joint]))
        cmd[joint] = int(clamp(value, 0, 1000))
    return cmd


def configure_elastic_adaptive(enabled: bool, values: Optional[Dict[str, Any]] = None):
    cmd = make_elastic_cmd(enabled, values)
    print(f"[ELASTIC] {'enable' if enabled else 'disable'} adaptive output: {cmd}",
          flush=True)
    send_serial(cmd)


def query_state(timeout: float = 0.5) -> Optional[Dict[str, Any]]:
    """发 {"T":105} 查询，读回 T=1051 状态（角度为弧度）。失败/超时返回 None。"""
    with ser_lock:
        try:
            ser.reset_input_buffer()        # 清掉上一次的残留，确保读到本次查询的返回
            ser.write((json.dumps({"T": 105}, separators=(",", ":")) + "\n").encode("utf-8"))
            buf = b""
            start = time.time()
            while time.time() - start < timeout:
                data = ser.read(512)        # ser timeout=0.05，无数据最多阻塞 0.05s
                if data:
                    buf += data
                    st = _extract_state(buf)
                    if st is not None:
                        return st
        except serial.SerialException as e:
            print(f"[SERIAL] query error: {e}", flush=True)
    return None


def send_point_and_wait(cmd: Dict[str, Any], dt_after: float = 0.0) -> bool:
    """发一个控制点，再轮询 T=105 直到到位或超时。

    到位判据：检测「角度收敛」——b/s/e/w 连续若干次几乎不再变化即认定机械臂已停止。
    这比 move 字段(实测停止后仍可能=1)和绝对角度比对(返回 s 符号与指令相反)都可靠，
    只看「是否还在动」，不受返回符号/零位/稳态误差影响。h(笔旋转件)不纳入。
    返回 True=到位，False=超时或被 stop 打断。"""
    target = {j: float(cmd[j]) for j in CHECK_KEYS}
    send_serial(cmd)
    start = time.time()
    next_log = 1.0
    prev = None
    moved = False
    stable = 0
    n_state = 0
    last_diff = None
    last_move = None
    while time.time() - start < POINT_TIMEOUT:
        if stop_event.is_set():
            return False
        st = query_state()
        elapsed = time.time() - start
        if st is None:
            if stop_event.wait(POLL_INTERVAL):
                return False
            continue
        n_state += 1
        last_move = st.get("move")
        cur = _state_to_deg(st)
        last_diff = {j: cur[j] - target[j] for j in CHECK_KEYS}
        err = max(abs(v) for v in last_diff.values())
        # 「角度收敛」判到位：相邻两次各关节变化都很小、连续若干次 => 已停止。
        if prev is not None:
            delta = max(abs(cur[j] - prev[j]) for j in CHECK_KEYS)
            if delta >= STABLE_DELTA:
                moved = True
                stable = 0
            else:
                stable += 1
        prev = cur
        if stable >= STABLE_NEEDED and (moved or elapsed >= MIN_SETTLE):
            if dt_after > 0:
                stop_event.wait(dt_after)
            return not stop_event.is_set()
        if elapsed >= next_log:
            print(f"[POLL] t={elapsed:.1f}s move={last_move} stable={stable} "
                  f"err={err:.1f}deg diff={ {j: round(last_diff[j], 1) for j in CHECK_KEYS} }",
                  flush=True)
            next_log = elapsed + 1.0
        if stop_event.wait(POLL_INTERVAL):
            return False
    # 超时诊断
    if n_state == 0:
        print(f"[POLL] 超时：{POINT_TIMEOUT}s 内未读到任何 T=1051 状态返回", flush=True)
    else:
        print(f"[POLL] 超时：收到 {n_state} 次状态但角度始终未收敛(可能一直抖动)，"
              f"最后误差 max={max(abs(v) for v in last_diff.values()):.1f}deg", flush=True)
    return False


def send_cmd_and_wait_stable(cmd: Dict[str, Any], label="CMD",
                             dt_after: float = 0.0) -> bool:
    """Send a non-joint command, then wait until feedback joint angles settle."""
    send_serial(cmd)
    start = time.time()
    next_log = 1.0
    prev = None
    moved = False
    stable = 0
    n_state = 0
    last_move = None

    while time.time() - start < POINT_TIMEOUT:
        if stop_event.is_set():
            return False
        st = query_state()
        elapsed = time.time() - start
        if st is None:
            if stop_event.wait(POLL_INTERVAL):
                return False
            continue
        n_state += 1
        last_move = st.get("move")
        cur = _state_to_deg(st)
        if prev is not None:
            delta = max(abs(cur[j] - prev[j]) for j in CHECK_KEYS)
            if delta >= STABLE_DELTA:
                moved = True
                stable = 0
            else:
                stable += 1
        prev = cur
        if stable >= STABLE_NEEDED and (moved or elapsed >= MIN_SETTLE):
            if dt_after > 0:
                stop_event.wait(dt_after)
            return not stop_event.is_set()
        if elapsed >= next_log:
            print(f"[POLL-{label}] t={elapsed:.1f}s move={last_move} "
                  f"stable={stable}", flush=True)
            next_log = elapsed + 1.0
        if stop_event.wait(POLL_INTERVAL):
            return False

    if n_state == 0:
        print(f"[POLL-{label}] timeout: no T=1051 state in {POINT_TIMEOUT}s",
              flush=True)
    else:
        print(f"[POLL-{label}] timeout: state received but not stable",
              flush=True)
    return False


def send_cartesian_point_force_adaptive(point: Dict[str, Any],
                                        q_seed: np.ndarray,
                                        spd,
                                        acc,
                                        adapter: ForceZController,
                                        p: Optional[RealtimePlotter] = None):
    """Hold target x/y, adapt z from torque-estimated pen force, then online IK."""
    base_xyz = _point_target_xyz(point, q_seed)
    q_cmd = np.array(q_seed, dtype=float)
    prev = None
    moved = False
    stable = 0
    n_state = 0
    last_diag = None
    next_log = 1.0
    last_sent_deg = None
    last_send_t = 0.0
    start = time.time()

    while time.time() - start < POINT_TIMEOUT:
        if stop_event.is_set():
            return q_cmd, False

        st = query_state()
        elapsed = time.time() - start
        if st is None:
            if elapsed < 0.1 and last_sent_deg is None:
                target_xyz = base_xyz.copy()
                target_xyz[2] += adapter.z_offset
                if motion_config["mode"] == "xyz":
                    cmd = make_xyz_cmd(target_xyz)
                else:
                    q_cmd, _, _ = ik(target_xyz, q_cmd, debug=False)
                    cmd = make_arm_cmd(_q_rad_to_point(q_cmd), global_spd=spd,
                                       global_acc=acc)
                send_serial(cmd)
                last_sent_deg = None if motion_config["mode"] == "xyz" else \
                    np.array([cmd[j] for j in JOINT_KEYS], dtype=float)
                last_send_t = time.time()
            if stop_event.wait(POLL_INTERVAL):
                return q_cmd, False
            continue

        n_state += 1
        q_feedback = _state_to_q_rad(st)
        cur = _state_to_deg(st)
        if prev is not None:
            delta = max(abs(cur[j] - prev[j]) for j in CHECK_KEYS)
            if delta >= STABLE_DELTA:
                moved = True
                stable = 0
            else:
                stable += 1
        prev = cur
        force_update_enabled = stable >= STABLE_NEEDED and \
            (moved or elapsed >= MIN_SETTLE)

        try:
            force_q = q_feedback if motion_config["mode"] == "xyz" else q_cmd
            last_diag = adapter.update(force_q, st, force_update_enabled)
        except RuntimeError as exc:
            print(f"[FORCE] safety stop: {exc}", flush=True)
            stop_event.set()
            return q_cmd, False
        if p is not None:
            p.add("tip_force", time.perf_counter(), {
                "fz": last_diag["fz"],
                "fz_filtered": last_diag["fz_filtered"],
                "target": last_diag["target"],
                "z_offset_mm": last_diag["z_offset_mm"],
                "rejected": 1.0 if last_diag.get("measurement_rejected") else 0.0,
            })

        target_xyz = base_xyz.copy()
        target_xyz[2] += adapter.z_offset
        if motion_config["mode"] == "xyz":
            cmd = make_xyz_cmd(target_xyz)
            ik_err = 0.0
            target_deg = None
            q_cmd = q_feedback
        else:
            q_cmd, ik_err, _ = ik(target_xyz, q_cmd, debug=False)
            cmd = make_arm_cmd(_q_rad_to_point(q_cmd), global_spd=spd,
                               global_acc=acc)
            target_deg = np.array([cmd[j] for j in JOINT_KEYS], dtype=float)

        now = time.time()
        if motion_config["mode"] == "xyz":
            should_send = now - last_send_t >= 0.25 or last_send_t == 0.0
        else:
            should_send = last_sent_deg is None or \
                np.max(np.abs(target_deg - last_sent_deg)) >= 0.05 or \
                now - last_send_t >= 0.25
        if should_send:
            send_serial(cmd)
            if p is not None and motion_config["mode"] != "xyz":
                p.add("target_joints", time.perf_counter(),
                      {k: float(cmd[k]) for k in JOINT_KEYS})
            last_sent_deg = target_deg
            last_send_t = now

        limit_hit = abs(adapter.z_offset - float(adapter.cfg["z_min"])) < 1e-9 or \
            abs(adapter.z_offset - float(adapter.cfg["z_max"])) < 1e-9
        if force_update_enabled and (adapter.force_ready() or limit_hit):
            return q_cmd, True

        if elapsed >= next_log:
            diag = last_diag or {}
            print("[FORCE] "
                  f"t={elapsed:.1f}s stable={stable} "
                  f"z_update={int(diag.get('z_update_enabled', False))} "
                  f"rejected={int(diag.get('measurement_rejected', False))} "
                  f"raw_fz={diag.get('fz', 0.0):+.3f} "
                  f"ctrl_fz={diag.get('fz_filtered', 0.0):+.3f} "
                  f"target={adapter.cfg['target']:.3f} "
                  f"z_offset={adapter.z_offset * 1000.0:+.2f}mm "
                  f"ik_err={ik_err * 1000.0:.2f}mm",
                  flush=True)
            next_log = elapsed + 1.0

        if stop_event.wait(POLL_INTERVAL):
            return q_cmd, False

    if n_state == 0:
        print(f"[FORCE] timeout: no T=1051 state in {POINT_TIMEOUT}s", flush=True)
    else:
        diag = last_diag or {}
        print("[FORCE] timeout: "
              f"raw_fz={diag.get('fz', 0.0):+.3f}, "
              f"ctrl_fz={diag.get('fz_filtered', 0.0):+.3f}, "
              f"target={adapter.cfg['target']:.3f}, "
              f"z_offset={adapter.z_offset * 1000.0:+.2f}mm",
              flush=True)
    return q_cmd, False


# ----------------------------------------------------------------------------
# 轨迹执行
# ----------------------------------------------------------------------------
def _execute_trajectory_worker_legacy(traj: Dict[str, Any]):
    points: List[Dict[str, Any]] = traj.get("points", [])
    dt = STREAM_DT
    traj_id = traj.get("traj_id", "unnamed")
    spd = traj.get("spd", DEFAULT_SPD)
    acc = traj.get("acc", DEFAULT_ACC)

    if not exec_lock.acquire(blocking=False):
        set_state(status="error", last_error="another trajectory is running")
        return

    # 关键：每条新轨迹开始时，必须清除上一次 stop
    stop_event.clear()

    try:
        set_state(status="running", traj_id=traj_id, current_index=0,
                  total_points=len(points), last_error=None, last_result=None)

        # 探测状态查询是否可用：可用则「发点->轮询到位->下一个」，否则回退定时(dt)
        use_poll = query_state() is not None
        mode = "poll-until-reached" if use_poll else "timed(dt)"
        print(f"[EXEC] traj_id={traj_id}, n_points={len(points)}, mode={mode}, "
              f"timeout={POINT_TIMEOUT}s, dt={dt}", flush=True)
        if not use_poll:
            print("[WARN] 状态查询无返回，回退为定时模式（无法确认到位）", flush=True)

        for i, point in enumerate(points):
            if stop_event.is_set():
                print(f"[STOP] trajectory stopped before point {i + 1}", flush=True)
                set_state(status="stopped", current_index=i,
                          last_result={"stopped": True, "at": i})
                return

            cmd = make_arm_cmd(point, global_spd=spd, global_acc=acc)
            print(f"[SEND] {i + 1}/{len(points)} {cmd}", flush=True)

            if use_poll:
                reached = send_point_and_wait(cmd, dt_after=dt)
                set_state(current_index=i + 1)
                if stop_event.is_set():
                    print(f"[STOP] trajectory stopped while waiting point {i + 1}", flush=True)
                    set_state(status="stopped", current_index=i + 1,
                              last_result={"stopped": True, "at": i + 1})
                    return
                if reached:
                    print(f"[REACHED] point {i + 1}/{len(points)}", flush=True)
                else:
                    print(f"[WARN] point {i + 1} 未在 {POINT_TIMEOUT}s 内到位，继续下一点", flush=True)
            else:
                send_serial(cmd)
                set_state(current_index=i + 1)
                print(f"[WAIT] after point {i + 1}, dt={dt}", flush=True)
                if stop_event.wait(dt):
                    print(f"[STOP] trajectory stopped during wait after point {i + 1}", flush=True)
                    set_state(status="stopped", current_index=i + 1,
                              last_result={"stopped": True, "at": i + 1})
                    return

        print(f"[DONE] traj_id={traj_id}", flush=True)
        set_state(status="done", current_index=len(points),
                  last_result={"ok": True, "traj_id": traj_id, "executed_points": len(points)})

    except serial.SerialException as e:
        print(f"[ERR] trajectory failed (serial): {e}", flush=True)
        set_state(status="error", last_error=str(e))
    except Exception as e:
        print(f"[ERR] trajectory failed: {e}", flush=True)
        set_state(status="error", last_error=str(e))
    finally:
        exec_lock.release()


def execute_trajectory_worker_reached(traj: Dict[str, Any]):
    points: List[Dict[str, Any]] = traj.get("points", [])
    dt = STREAM_DT
    traj_id = traj.get("traj_id", "unnamed")
    spd = traj.get("spd", DEFAULT_SPD)
    acc = traj.get("acc", DEFAULT_ACC)
    init_spd = init_motion_config["spd"]
    init_acc = init_motion_config["acc"]

    if not exec_lock.acquire(blocking=False):
        set_state(status="error", last_error="another trajectory is running")
        return

    stop_event.clear()

    try:
        set_state(status="running", traj_id=traj_id, current_index=0,
                  total_points=len(points), last_error=None, last_result=None)
        p = start_plotter()
        adaptive = bool(force_z_config.get("enabled", False))
        adapter = ForceZController(force_z_config) if adaptive else None
        motion_mode = motion_config["mode"]
        print(f"[EXEC] traj_id={traj_id}, n_points={len(points)}, "
              f"mode=send-one-wait-reached:{motion_mode}"
              f"{'+force-z-adaptive' if adaptive else ''}", flush=True)
        if adaptive and not all(all(k in pt for k in ("x", "y", "z")) for pt in points):
            print("[FORCE] warning: trajectory has no xyz targets; "
                  "falling back to FK of joint points", flush=True)

        first_cmd = make_arm_cmd(points[0], global_spd=init_spd,
                                 global_acc=init_acc)
        st0 = query_state(timeout=0.5)
        if st0 is not None:
            current = _state_to_deg(st0)
        else:
            current = {k: 0.0 for k in JOINT_KEYS}

        wrist_point = {k: current.get(k, 0.0) for k in JOINT_KEYS}
        wrist_point["w"] = first_cmd["w"]
        wrist_cmd = make_arm_cmd(wrist_point, global_spd=init_spd,
                                 global_acc=init_acc)
        print(f"[PREP] wrist first to avoid camera: {wrist_cmd}", flush=True)
        if not send_point_and_wait(wrist_cmd, dt_after=0.0):
            set_state(status="stopped" if stop_event.is_set() else "error",
                      current_index=0,
                      last_error="wrist preposition failed")
            return

        if stop_event.is_set():
            set_state(status="stopped", current_index=0,
                      last_result={"stopped": True, "at": "wrist-prep"})
            return

        if motion_mode == "xyz":
            first_xyz = _point_target_xyz(points[0], _point_to_q_rad(first_cmd))
            first_move_cmd = make_xyz_cmd(first_xyz)
            print(f"[PREP-XYZ] move to first point and wait: "
                  f"world={np.round(first_xyz, 5).tolist()} cmd={first_move_cmd}",
                  flush=True)
            first_reached = send_cmd_and_wait_stable(first_move_cmd, label="XYZ")
        else:
            first_move_cmd = first_cmd
            print(f"[PREP] move to first point and wait: {first_cmd}", flush=True)
            first_reached = send_point_and_wait(first_cmd, dt_after=0.0)
        if not first_reached:
            set_state(status="stopped" if stop_event.is_set() else "error",
                      current_index=0,
                      last_error="first point preposition failed")
            return
        st_first = query_state(timeout=0.2)
        q_seed = _state_to_q_rad(st_first) if st_first is not None else \
            _point_to_q_rad(first_cmd)
        set_state(current_index=1)

        if len(points) <= 1:
            print(f"[DONE] traj_id={traj_id} (first point only)", flush=True)
            set_state(status="done", current_index=1,
                      last_result={"ok": True, "traj_id": traj_id,
                                   "executed_points": 1})
            return

        print("[EXEC] first point reached; start point-by-point drawing", flush=True)
        for stream_idx, point in enumerate(points[1:], start=1):
            if stop_event.is_set():
                print(f"[STOP] trajectory stopped before point {stream_idx + 1}", flush=True)
                set_state(status="stopped", current_index=stream_idx,
                          last_result={"stopped": True, "at": stream_idx})
                return

            if adapter is not None:
                base_xyz = _point_target_xyz(point, q_seed)
                print(f"[SEND-FORCE] {stream_idx + 1}/{len(points)} "
                      f"xy=({base_xyz[0]:.5f},{base_xyz[1]:.5f}) "
                      f"base_z={base_xyz[2]:.5f}", flush=True)
                q_seed, reached = send_cartesian_point_force_adaptive(
                    point, q_seed, spd, acc, adapter, p)
            else:
                if motion_mode == "xyz":
                    target_xyz = _point_target_xyz(point, q_seed)
                    cmd = make_xyz_cmd(target_xyz)
                    print(f"[SEND-XYZ] {stream_idx + 1}/{len(points)} "
                          f"world={np.round(target_xyz, 5).tolist()} cmd={cmd}",
                          flush=True)
                    reached = send_cmd_and_wait_stable(cmd, label="XYZ")
                    st_after = query_state(timeout=0.2)
                    if st_after is not None:
                        q_seed = _state_to_q_rad(st_after)
                else:
                    cmd = make_arm_cmd(point, global_spd=spd, global_acc=acc)
                    now = time.perf_counter()
                    target = {k: float(cmd[k]) for k in JOINT_KEYS}
                    print(f"[SEND] {stream_idx + 1}/{len(points)} {cmd}",
                          flush=True)
                    p.add("target_joints", now, target)
                    reached = send_point_and_wait(cmd, dt_after=0.0)
                    q_seed = _point_to_q_rad(cmd)
            set_state(current_index=stream_idx + 1)
            if stop_event.is_set():
                print(f"[STOP] trajectory stopped while waiting point {stream_idx + 1}",
                      flush=True)
                set_state(status="stopped", current_index=stream_idx + 1,
                          last_result={"stopped": True, "at": stream_idx + 1})
                return
            if reached:
                print(f"[REACHED] point {stream_idx + 1}/{len(points)}", flush=True)
            else:
                print(f"[WARN] point {stream_idx + 1} not reached before timeout; "
                      f"continuing", flush=True)

        print(f"[DONE] traj_id={traj_id}", flush=True)
        set_state(status="done", current_index=len(points),
                  last_result={"ok": True, "traj_id": traj_id,
                               "executed_points": len(points)})

    except serial.SerialException as e:
        print(f"[ERR] trajectory failed (serial): {e}", flush=True)
        set_state(status="error", last_error=str(e))
    except Exception as e:
        print(f"[ERR] trajectory failed: {e}", flush=True)
        set_state(status="error", last_error=str(e))
    finally:
        exec_lock.release()


def start_trajectory(req: Dict[str, Any]) -> Dict[str, Any]:
    points = req.get("points", [])
    traj_id = req.get("traj_id", "unnamed")

    if not points:
        return {"ok": False, "error": "empty trajectory"}
    if exec_lock.locked():
        return {"ok": False, "error": "another trajectory is running", "state": get_state()}

    t = threading.Thread(target=execute_trajectory_worker_reached, args=(req,), daemon=True)
    t.start()
    return {"ok": True, "msg": "trajectory accepted", "traj_id": traj_id,
            "n_points": len(points),
            "mode": f"send-one-wait-reached:{motion_config['mode']}"
                    f"{'+force-z-adaptive' if force_z_config.get('enabled') else ''}"}


def handle_request(req: Dict[str, Any]) -> Dict[str, Any]:
    req_type = req.get("type")

    if req_type == "trajectory":
        return start_trajectory(req)

    if req_type == "joint":
        if exec_lock.locked():
            return {"ok": False, "error": "trajectory is running; stop it first"}
        cmd = make_arm_cmd(req)
        print(f"[JOINT] {cmd}", flush=True)
        send_serial(cmd)
        return {"ok": True, "sent": cmd}

    if req_type == "elastic":
        if exec_lock.locked():
            return {"ok": False, "error": "trajectory is running; stop it first"}
        enabled = bool(req.get("enabled", req.get("mode", 1)))
        values = {j: req[j] for j in JOINT_KEYS if j in req}
        cmd = make_elastic_cmd(enabled, values or None)
        print(f"[ELASTIC] request: {cmd}", flush=True)
        send_serial(cmd)
        return {"ok": True, "sent": cmd}

    if req_type == "state":
        st = query_state()
        return {"ok": st is not None, "raw": st, "server_state": get_state()}

    if req_type == "status":
        return {"ok": True, "server_state": get_state()}

    if req_type == "stop":
        stop_event.set()
        return {"ok": True, "msg": "stop requested"}

    if req_type == "ping":
        return {"ok": True, "msg": "pong"}

    return {"ok": False, "error": f"unknown request type: {req_type}"}


def recv_json_line(conn: socket.socket) -> Dict[str, Any]:
    buf = b""
    while True:
        chunk = conn.recv(4096)
        if not chunk:
            raise ConnectionError("client disconnected before newline")
        buf += chunk
        if b"\n" in buf:
            line, _ = buf.split(b"\n", 1)
            return json.loads(line.decode("utf-8"))


def parse_args():
    ap = argparse.ArgumentParser(description="Robot TCP server with direct serial control")
    group = ap.add_mutually_exclusive_group()
    group.add_argument("--elastic-adaptive", action="store_true",
                       help="启动后发送 T=112 mode=1，开启弹力自适应输出")
    group.add_argument("--disable-elastic-adaptive", action="store_true",
                       help="启动后发送 T=112 mode=0，关闭弹力自适应输出")
    ap.add_argument("--init-spd", type=int, default=DEFAULT_INIT_SPD,
                    help="初始化阶段关节角速度(°/s)，默认 25")
    ap.add_argument("--init-acc", type=int, default=DEFAULT_INIT_ACC,
                    help="初始化阶段关节角加速度，默认 20")
    ap.add_argument("--motion-mode", choices=("joint", "xyz"), default="joint",
                    help="轨迹执行方式：joint 使用 T=122，xyz 使用 T=104")
    ap.add_argument("--xyz-axis-map", default="x,y,z",
                    help="XYZ 模式轴映射：机器人 cmd x,y,z 分别取我们的哪些轴，支持负号，如 x,-y,z")
    ap.add_argument("--xyz-scale", default="1000,1000,1000",
                    help="XYZ 模式轴缩放，默认米->毫米：1000,1000,1000")
    ap.add_argument("--xyz-offset", default="0,0,0",
                    help="XYZ 模式命令坐标偏移，作用在缩放后，默认 0,0,0")
    ap.add_argument("--xyz-spd", type=float, default=DEFAULT_XYZ_SPD,
                    help="XYZ 模式 T=104 的 spd 字段，默认 0.25")
    ap.add_argument("--elastic-b", type=int, default=None,
                    help="弹力自适应 b 关节输出值，默认 60")
    ap.add_argument("--elastic-s", type=int, default=None,
                    help="弹力自适应 s 关节输出值，默认 100")
    ap.add_argument("--elastic-e", type=int, default=None,
                    help="弹力自适应 e 关节输出值，默认 35")
    ap.add_argument("--elastic-w", type=int, default=None,
                    help="弹力自适应 w 关节输出值，默认 35")
    ap.add_argument("--elastic-h", type=int, default=None,
                    help="弹力自适应 h 关节输出值，默认 35")
    ap.add_argument("--force-z-adaptive", action="store_true",
                    help="绘制阶段根据力矩反馈估计笔尖受力，并动态调整 z")
    ap.add_argument("--force-target", type=float, default=FORCE_Z_DEFAULT["target"],
                    help="目标笔尖 z 向受力估计值(N)，默认 0.7")
    ap.add_argument("--force-k", type=float, default=FORCE_Z_DEFAULT["gain"],
                    help="z 导纳增益，单位 m/(N*s)，默认 0.0005")
    ap.add_argument("--force-z-min", type=float, default=FORCE_Z_DEFAULT["z_min"],
                    help="z_offset 最小值(m)，默认 -0.002")
    ap.add_argument("--force-z-max", type=float, default=FORCE_Z_DEFAULT["z_max"],
                    help="z_offset 最大值(m)，默认 0.004")
    ap.add_argument("--force-z-step", type=float, default=FORCE_Z_DEFAULT["z_step"],
                    help="单次 z_offset 最大变化(m)，默认 0.00015")
    ap.add_argument("--force-alpha", type=float, default=FORCE_Z_DEFAULT["alpha"],
                    help="力估计低通滤波系数，默认 0.2")
    ap.add_argument("--force-max", type=float, default=FORCE_Z_DEFAULT["max_force"],
                    help="估计笔尖 Fz 绝对值报警阈值(N)，默认 5.0")
    ap.add_argument("--force-safety-stop", action="store_true",
                    help="Fz 超过 --force-max 时停止轨迹；默认只报警不停机")
    ap.add_argument("--force-sign", type=float, choices=(-1.0, 1.0),
                    default=FORCE_Z_DEFAULT["sign"],
                    help="z 向力符号，若压力越大估计值越负则设为 -1")
    ap.add_argument("--force-bias-samples", type=int,
                    default=FORCE_Z_DEFAULT["bias_samples"],
                    help="启动时空载力矩零偏采样数，默认 20")
    ap.add_argument("--force-bias-model", default=None,
                    help="姿态相关静态力矩零偏模型 JSON；加载后使用 tau_bias(q)")
    ap.add_argument("--force-tolerance", type=float,
                    default=FORCE_Z_DEFAULT["tolerance"],
                    help="认为笔尖受力到达目标的容差，默认 0.15")
    return ap.parse_args()


def elastic_values_from_args(args) -> Dict[str, int]:
    values = {}
    for joint in JOINT_KEYS:
        arg_value = getattr(args, f"elastic_{joint}")
        values[joint] = DEFAULT_ELASTIC_ON[joint] if arg_value is None else arg_value
    return values


def parse_float3(text: str, name: str) -> np.ndarray:
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 3:
        raise ValueError(f"{name} must have 3 comma-separated numbers")
    return np.array([float(p) for p in parts], dtype=float)


def parse_axis_map(text: str):
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 3:
        raise ValueError("--xyz-axis-map must have 3 comma-separated axes")
    valid = {"x", "y", "z", "-x", "-y", "-z"}
    for part in parts:
        if part not in valid:
            raise ValueError(f"invalid xyz axis token: {part}")
    return tuple(parts)


def motion_config_from_args(args) -> Dict[str, Any]:
    return {
        "mode": args.motion_mode,
        "xyz_axis_map": parse_axis_map(args.xyz_axis_map),
        "xyz_offset": parse_float3(args.xyz_offset, "--xyz-offset"),
        "xyz_scale": parse_float3(args.xyz_scale, "--xyz-scale"),
        "xyz_spd": float(args.xyz_spd),
    }


def force_z_config_from_args(args) -> Dict[str, Any]:
    cfg = dict(FORCE_Z_DEFAULT)
    cfg.update({
        "enabled": bool(args.force_z_adaptive),
        "target": float(args.force_target),
        "gain": float(args.force_k),
        "z_min": float(args.force_z_min),
        "z_max": float(args.force_z_max),
        "z_step": float(args.force_z_step),
        "alpha": float(args.force_alpha),
        "max_force": float(args.force_max),
        "safety_stop": bool(args.force_safety_stop),
        "sign": float(args.force_sign),
        "bias_samples": int(args.force_bias_samples),
        "tolerance": float(args.force_tolerance),
    })
    if cfg["z_min"] > cfg["z_max"]:
        raise ValueError("--force-z-min must be <= --force-z-max")
    if not 0.0 < cfg["alpha"] <= 1.0:
        raise ValueError("--force-alpha must be in (0, 1]")
    return cfg


def main():
    global force_z_config, force_bias_model, init_motion_config, motion_config
    args = parse_args()
    force_z_config = force_z_config_from_args(args)
    motion_config = motion_config_from_args(args)
    if args.force_bias_model:
        force_bias_model = load_bias_model(args.force_bias_model)
        print(f"[FORCE] loaded posture bias model: {args.force_bias_model} "
              f"n={force_bias_model.get('n_samples')} "
              f"rmse={force_bias_model.get('rmse_ncm')}", flush=True)
    init_motion_config = {
        "spd": max(1, int(args.init_spd)),
        "acc": max(1, int(args.init_acc)),
    }
    open_serial()
    print(f"[INIT] pre-draw motion spd={init_motion_config['spd']} "
          f"acc={init_motion_config['acc']}", flush=True)
    print("[MOTION] "
          f"mode={motion_config['mode']} "
          f"xyz_axis_map={motion_config['xyz_axis_map']} "
          f"xyz_scale={motion_config['xyz_scale'].tolist()} "
          f"xyz_offset={motion_config['xyz_offset'].tolist()} "
          f"xyz_spd={motion_config['xyz_spd']}",
          flush=True)
    if args.elastic_adaptive:
        configure_elastic_adaptive(True, elastic_values_from_args(args))
    elif args.disable_elastic_adaptive:
        configure_elastic_adaptive(False)
    if force_z_config["enabled"]:
        print(f"[FORCE] z-adaptive enabled: {force_z_config}", flush=True)
        if force_bias_model is None:
            calibrate_force_torque_bias(force_z_config["bias_samples"])
        else:
            print("[FORCE] using posture bias model; skip constant bias calibration",
                  flush=True)
    start_plotter()
    start_telemetry_receiver()
    print(f"[OK] robot_server (Windows direct-serial) listening on {SERVER_HOST}:{SERVER_PORT}",
          flush=True)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((SERVER_HOST, SERVER_PORT))
    server.listen(5)

    while True:
        conn, addr = server.accept()
        print(f"[CLIENT] connected: {addr}", flush=True)
        try:
            req = recv_json_line(conn)
            print(f"[REQ] {req}", flush=True)
            resp = handle_request(req)
        except Exception as e:
            resp = {"ok": False, "error": str(e)}
            print(f"[ERR] {e}", flush=True)
        try:
            conn.sendall((json.dumps(resp, separators=(",", ":")) + "\n").encode("utf-8"))
        finally:
            conn.close()
            print("[CLIENT] disconnected", flush=True)


if __name__ == "__main__":
    main()
