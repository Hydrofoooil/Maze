import socket
import json
import time
import threading
from typing import Dict, Any, List, Optional

SERVER_HOST = "0.0.0.0"
SERVER_PORT = 9000

BRIDGE_HOST = "127.0.0.1"
BRIDGE_PORT = 9100

LIMITS = {
    "b": (-180.0, 180.0),
    "s": (-90.0, 90.0),
    "e": (-90.0, 90.0),
    "w": (-90.0, 90.0),
    "h": (-45.0, 45.0),
}

DEFAULT_SPD = 10
DEFAULT_ACC = 10

state_lock = threading.Lock()
exec_lock = threading.Lock()
stop_event = threading.Event()

server_state = {
    "status": "idle",        # idle / running / done / stopped / error
    "traj_id": None,
    "current_index": 0,
    "total_points": 0,
    "last_error": None,
    "last_result": None,
}


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


def send_to_bridge(cmd: Dict[str, Any]) -> str:
    msg = json.dumps(cmd, separators=(",", ":")) + "\n"
    print(f"[BRIDGE] connecting to {BRIDGE_HOST}:{BRIDGE_PORT}")

    sock = socket.create_connection((BRIDGE_HOST, BRIDGE_PORT), timeout=3)
    try:
        sock.settimeout(1.0)
        sock.sendall(msg.encode("utf-8"))
        print("[BRIDGE] sent and shutdown")

        # 告诉 serial_bridge：我这边已经写完，不再继续发
        try:
            sock.shutdown(socket.SHUT_WR)
        except OSError:
            pass

    finally:
        sock.close()

    print("[BRIDGE] closed")
    return ""


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
        set_state(
            status="running",
            traj_id=traj_id,
            current_index=0,
            total_points=len(points),
            last_error=None,
            last_result=None,
        )

        print(f"[EXEC] traj_id={traj_id}, n_points={len(points)}, dt={dt}", flush=True)

        for i, point in enumerate(points):
            if stop_event.is_set():
                print(f"[STOP] trajectory stopped before point {i + 1}", flush=True)
                set_state(
                    status="stopped",
                    current_index=i,
                    last_result={"stopped": True, "at": i},
                )
                return

            cmd = make_arm_cmd(point, global_spd=spd, global_acc=acc)

            print(f"[SEND] {i + 1}/{len(points)} {cmd}", flush=True)
            send_to_bridge(cmd)

            set_state(current_index=i + 1)

            print(f"[WAIT] after point {i + 1}, dt={dt}", flush=True)

            if stop_event.wait(dt):
                print(f"[STOP] trajectory stopped during wait after point {i + 1}", flush=True)
                set_state(
                    status="stopped",
                    current_index=i + 1,
                    last_result={"stopped": True, "at": i + 1},
                )
                return

        print(f"[DONE] traj_id={traj_id}", flush=True)
        set_state(
            status="done",
            current_index=len(points),
            last_result={
                "ok": True,
                "traj_id": traj_id,
                "executed_points": len(points),
            },
        )

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

    return {
        "ok": True,
        "msg": "trajectory accepted",
        "traj_id": traj_id,
        "n_points": len(points),
        "dt": dt,
    }


def handle_request(req: Dict[str, Any]) -> Dict[str, Any]:
    req_type = req.get("type")

    if req_type == "trajectory":
        return start_trajectory(req)

    if req_type == "joint":
        if exec_lock.locked():
            return {"ok": False, "error": "trajectory is running; stop it first"}

        cmd = make_arm_cmd(req)
        print(f"[JOINT] {cmd}")
        send_to_bridge(cmd)
        return {"ok": True, "sent": cmd}

    if req_type == "state":
        raw = send_to_bridge({"T": 105})
        return {"ok": True, "raw": raw, "server_state": get_state()}

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
    print(f"[OK] robot_server listening on {SERVER_HOST}:{SERVER_PORT}")
    print(f"[OK] serial bridge target: {BRIDGE_HOST}:{BRIDGE_PORT}")

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((SERVER_HOST, SERVER_PORT))
    server.listen(5)

    while True:
        conn, addr = server.accept()
        print(f"[CLIENT] connected: {addr}")

        try:
            req = recv_json_line(conn)
            print(f"[REQ] {req}")
            resp = handle_request(req)

        except Exception as e:
            resp = {"ok": False, "error": str(e)}
            print(f"[ERR] {e}")

        try:
            msg = json.dumps(resp, separators=(",", ":")) + "\n"
            conn.sendall(msg.encode("utf-8"))
        finally:
            conn.close()
            print("[CLIENT] disconnected")


if __name__ == "__main__":
    main()