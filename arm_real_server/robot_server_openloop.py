"""
真实机械臂下位机服务（Windows 直连串口版）—— 开环定时版。

与 robot_server.py 的唯一区别：trajectory 执行是**开环定时**的——按固定 `dt` 逐点发送，
不查询状态、不等到位。点足够密时机械臂会平滑地连续流过这些点，更适合画连续曲线
（不像闭环到位那样走走停停）；代价是不保证每点精确到达。

其余与 robot_server.py 一致：直接跑在下位机 Windows 上、用 pyserial 直读写 COM4、监听 9001、
对上位机提供 ping/joint/trajectory/status/state/stop 接口。无需 WSL / serial_bridge / portproxy。

运行（下位机 Windows，先 pip install pyserial）：
    python robot_server_openloop.py
"""

import socket
import json
import time
import math
import threading
from typing import Dict, Any, List, Optional

import serial

# ---- 对上位机的 TCP 服务 ----
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 9001

# ---- 机械臂串口 ----
SERIAL_PORT = "COM4"
BAUD = 115200

LIMITS = {
    "b": (-180.0, 180.0),
    "s": (-90.0, 90.0),
    "e": (-90.0, 90.0),
    "w": (-90.0, 90.0),     # 画竖直笔需约 -93°，超 ±90 在此 clamp 到 -90（笔约 3° 恒定倾斜）
    "h": (-180.0, 180.0),   # 夹爪已拆、改装为与笔固定的旋转件（原装夹爪 ±45）
}

DEFAULT_SPD = 10
DEFAULT_ACC = 10

JOINT_KEYS = ("b", "s", "e", "w", "h")
_RAD2DEG = 180.0 / math.pi

state_lock = threading.Lock()
exec_lock = threading.Lock()
stop_event = threading.Event()
ser_lock = threading.Lock()         # 串口读写串行化

server_state = {
    "status": "idle",        # idle / running / done / stopped / error
    "traj_id": None,
    "current_index": 0,
    "total_points": 0,
    "last_error": None,
    "last_result": None,
}

ser: Optional[serial.Serial] = None


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
    """T=1051 状态返回（弧度）-> b/s/e/w/h（度）。第 5 关节返回字段名是 't'。"""
    return {
        "b": float(st.get("b", 0.0)) * _RAD2DEG,
        "s": float(st.get("s", 0.0)) * _RAD2DEG,
        "e": float(st.get("e", 0.0)) * _RAD2DEG,
        "w": float(st.get("w", 0.0)) * _RAD2DEG,
        "h": float(st.get("t", 0.0)) * _RAD2DEG,
    }


def send_serial(cmd: Dict[str, Any]):
    """把一条指令写入串口（线程安全）。"""
    msg = json.dumps(cmd, separators=(",", ":")) + "\n"
    with ser_lock:
        ser.write(msg.encode("utf-8"))


def query_state(timeout: float = 0.5) -> Optional[Dict[str, Any]]:
    """发 {"T":105} 查询，读回 T=1051 状态（角度为弧度）。失败/超时返回 None。"""
    with ser_lock:
        try:
            ser.reset_input_buffer()
            ser.write((json.dumps({"T": 105}, separators=(",", ":")) + "\n").encode("utf-8"))
            buf = b""
            start = time.time()
            while time.time() - start < timeout:
                data = ser.read(512)
                if data:
                    buf += data
                    st = _extract_state(buf)
                    if st is not None:
                        return st
        except serial.SerialException as e:
            print(f"[SERIAL] query error: {e}", flush=True)
    return None


# ----------------------------------------------------------------------------
# 轨迹执行（开环：按 dt 定时逐点发送，不查询状态、不等到位）
# ----------------------------------------------------------------------------
def execute_trajectory_worker(traj: Dict[str, Any]):
    points: List[Dict[str, Any]] = traj.get("points", [])
    dt = float(traj.get("dt", 0.1))
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
        print(f"[EXEC] traj_id={traj_id}, n_points={len(points)}, dt={dt} (open-loop timed)",
              flush=True)

        for i, point in enumerate(points):
            if stop_event.is_set():
                print(f"[STOP] trajectory stopped before point {i + 1}", flush=True)
                set_state(status="stopped", current_index=i,
                          last_result={"stopped": True, "at": i})
                return

            cmd = make_arm_cmd(point, global_spd=spd, global_acc=acc)
            print(f"[SEND] {i + 1}/{len(points)} {cmd}", flush=True)
            send_serial(cmd)
            set_state(current_index=i + 1)

            if stop_event.wait(dt):   # 定时等待 dt（可被 stop 打断）
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


def start_trajectory(req: Dict[str, Any]) -> Dict[str, Any]:
    points = req.get("points", [])
    dt = float(req.get("dt", 0.1))
    traj_id = req.get("traj_id", "unnamed")

    if not points:
        return {"ok": False, "error": "empty trajectory"}
    if dt <= 0:
        return {"ok": False, "error": "dt must be positive"}
    if exec_lock.locked():
        return {"ok": False, "error": "another trajectory is running", "state": get_state()}

    t = threading.Thread(target=execute_trajectory_worker, args=(req,), daemon=True)
    t.start()
    return {"ok": True, "msg": "trajectory accepted", "traj_id": traj_id,
            "n_points": len(points), "dt": dt}


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


def main():
    open_serial()
    print(f"[OK] robot_server OPEN-LOOP (Windows direct-serial) listening on "
          f"{SERVER_HOST}:{SERVER_PORT}", flush=True)

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
