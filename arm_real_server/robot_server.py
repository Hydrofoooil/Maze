import socket
import json
import time
import math
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
    # 手腕舵机文档行程 ±90。画竖直笔理论需约 -93°，超出部分在此 clamp 到 -90，
    # 笔产生约 3° 恒定倾斜（画线可接受），避免顶机械挡块堵转。
    # 若确认手腕舵机物理能转过 ±90，可放宽到 ±95 换取笔完全竖直。
    "w": (-90.0, 90.0),
    # 夹爪已拆除、改装为与笔固定的旋转连接件，按舵机行程放宽（原装夹爪为 ±45）
    "h": (-180.0, 180.0),
}

DEFAULT_SPD = 10
DEFAULT_ACC = 10

# 到位轮询参数（闭环执行：发一个点 -> 轮询 T=105 直到到位 -> 再发下一个）
ANGLE_TOL_DEG = 2.0      # 各关节与目标偏差都 < 此值(度) 即认定到达目标
POLL_INTERVAL = 0.1      # 轮询状态的间隔(s)
POINT_TIMEOUT = 12.0     # 单点最大等待(s)，超时则放弃等待、继续下一点
                         # 注意 spd 单位 °/s：spd=10 走 90° 要 9s，大角度移动需放宽此值或调大 spd
MIN_SETTLE = 0.8         # 目标≈当前(几乎不动)时，至少等这么久再判定到位
STABLE_DELTA = 0.6       # 相邻两次状态各关节变化都 < 此值，视为"这一刻没在动"
STABLE_NEEDED = 4        # 连续这么多次"没在动"判定为已停止到位（× POLL_INTERVAL ≈ 确认时长）
JOINT_KEYS = ("b", "s", "e", "w", "h")
CHECK_KEYS = ("b", "s", "e", "w")   # 到位判据只看这 4 个；h(笔旋转件)返回字段 t 零位存疑、画图非关键
_RAD2DEG = 180.0 / math.pi

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


def _connect_bridge(timeout: float = 2.0, retries: int = 8, backoff: float = 0.05):
    """连接 serial_bridge，带重试。bridge 一次只服务一个连接，连接交替时偶尔需要
    等它回到 accept，重试可吸收偶发的 Connection refused。全部失败则抛最后一次异常。"""
    last = None
    for _ in range(retries):
        try:
            return socket.create_connection((BRIDGE_HOST, BRIDGE_PORT), timeout=timeout)
        except OSError as e:
            last = e
            time.sleep(backoff)
    raise last


def send_to_bridge(cmd: Dict[str, Any]) -> str:
    msg = json.dumps(cmd, separators=(",", ":")) + "\n"
    print(f"[BRIDGE] connecting to {BRIDGE_HOST}:{BRIDGE_PORT}")

    sock = _connect_bridge(timeout=3)
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


def query_state(timeout: float = 1.0) -> Optional[Dict[str, Any]]:
    """向机械臂发 {"T":105} 查询当前状态，读回 T=1051 返回（角度为弧度）。失败/超时返回 None。
    与 send_to_bridge 不同：这里发完不 shutdown 写端，必须保持连接读返回。"""
    try:
        sock = _connect_bridge()
    except OSError as e:
        print(f"[QUERY] connect failed: {e}", flush=True)
        return None
    try:
        sock.settimeout(timeout)
        sock.sendall((json.dumps({"T": 105}, separators=(",", ":")) + "\n").encode("utf-8"))
        buf = b""
        start = time.time()
        while time.time() - start < timeout:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                break
            if not chunk:
                break
            buf += chunk
            st = _extract_state(buf)
            if st is not None:
                return st
    except OSError:
        return None
    finally:
        try:
            sock.close()
        except OSError:
            pass
    return None


def _state_to_deg(st: Dict[str, Any]) -> Dict[str, float]:
    """把 T=1051 状态返回（弧度）换算成 b/s/e/w/h（度）。
    注意：返回里第 5 关节字段名是 't'（见说明书 6.7），对应控制指令里的 h。"""
    return {
        "b": float(st.get("b", 0.0)) * _RAD2DEG,
        "s": float(st.get("s", 0.0)) * _RAD2DEG,
        "e": float(st.get("e", 0.0)) * _RAD2DEG,
        "w": float(st.get("w", 0.0)) * _RAD2DEG,
        "h": float(st.get("t", 0.0)) * _RAD2DEG,
    }


def send_point_and_wait(cmd: Dict[str, Any], dt_after: float = 0.0) -> bool:
    """连一次 bridge：先发控制点，再在【同一连接】上轮询 T=105 直到到位或超时。
    每个点只占用一次 bridge 连接（不像「发点 + 反复重连查询」那样高频建连，
    会压垮串行处理的 serial_bridge）。

    到位判据：检测「角度收敛」——b/s/e/w 连续若干次几乎不再变化即认定机械臂已停止。
    这比 move 字段(实测停止后仍可能=1)和绝对角度比对(返回 s 符号与指令相反)都可靠，
    只看「是否还在动」，不受返回符号/零位/稳态误差影响。h(笔旋转件)不纳入。
    返回 True=到位，False=超时或被 stop 打断。"""
    target = {j: float(cmd[j]) for j in CHECK_KEYS}
    try:
        sock = _connect_bridge()
    except OSError as e:
        print(f"[BRIDGE] connect failed: {e}", flush=True)
        return False
    try:
        sock.settimeout(0.3)
        sock.sendall((json.dumps(cmd, separators=(",", ":")) + "\n").encode("utf-8"))
        start = time.time()
        next_query = 0.0
        next_log = 1.0
        buf = b""
        n_state = 0
        prev = None
        moved = False
        stable = 0
        last_diff = None
        last_move = None
        while time.time() - start < POINT_TIMEOUT:
            if stop_event.is_set():
                return False
            elapsed = time.time() - start
            if elapsed >= next_query:
                try:
                    sock.sendall((json.dumps({"T": 105}, separators=(",", ":")) + "\n").encode("utf-8"))
                except OSError:
                    return False
                next_query = elapsed + POLL_INTERVAL
            try:
                data = sock.recv(4096)
            except socket.timeout:
                continue
            if not data:
                break
            buf += data
            st = _extract_state(buf)
            if st is not None:
                buf = b""
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
        # 超时诊断
        if n_state == 0:
            print(f"[POLL] 超时：{POINT_TIMEOUT}s 内未读到任何 T=1051 状态返回", flush=True)
        else:
            print(f"[POLL] 超时：收到 {n_state} 次状态但角度始终未收敛(可能一直抖动)，"
                  f"最后误差 max={max(abs(v) for v in last_diff.values()):.1f}deg", flush=True)
        return False
    except OSError as e:
        print(f"[BRIDGE] 通信中断（bridge 可能断开）: {e}", flush=True)
        return False
    finally:
        try:
            sock.close()
        except OSError:
            pass


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

        # 探测状态查询是否可用：可用则「发点->轮询到位->下一个」，否则回退定时(dt)
        use_poll = query_state() is not None
        mode = "poll-until-reached" if use_poll else "timed(dt)"
        print(f"[EXEC] traj_id={traj_id}, n_points={len(points)}, mode={mode}, "
              f"tol={ANGLE_TOL_DEG}deg, timeout={POINT_TIMEOUT}s, dt={dt}", flush=True)
        if not use_poll:
            print("[WARN] 状态查询无返回，回退为定时模式（无法确认到位）", flush=True)

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

            if use_poll:
                # 发点 + 在同一连接上轮询到位（每点只连一次 bridge）
                reached = send_point_and_wait(cmd, dt_after=dt)
                set_state(current_index=i + 1)
                if stop_event.is_set():
                    print(f"[STOP] trajectory stopped while waiting point {i + 1}", flush=True)
                    set_state(
                        status="stopped",
                        current_index=i + 1,
                        last_result={"stopped": True, "at": i + 1},
                    )
                    return
                if reached:
                    print(f"[REACHED] point {i + 1}/{len(points)}", flush=True)
                else:
                    print(f"[WARN] point {i + 1} 未在 {POINT_TIMEOUT}s 内到位，继续下一点", flush=True)
            else:
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
