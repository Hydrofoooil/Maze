import socket
import threading
import serial
import time
import traceback

SERIAL_PORT = "COM4"
BAUD = 115200

# 监听本机所有地址，WSL 可以通过 Windows host IP 连接
TCP_HOST = "0.0.0.0"
TCP_PORT = 9100

# 连接空闲超过此秒数则主动断开：避免某个半开连接把 recv 永久阻塞、导致 bridge
# 再也回不到 accept、后续连接全部 Connection refused。
IDLE_TIMEOUT = 30.0


def open_serial():
    """打开串口；机械臂运动拉扯线缆/供电波动导致 COM 口瞬断后，可用它重连。"""
    s = serial.Serial(SERIAL_PORT, BAUD, timeout=0.05)
    time.sleep(2)
    print(f"[OK] Opened serial {SERIAL_PORT} @ {BAUD}")
    return s


ser = open_serial()

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((TCP_HOST, TCP_PORT))
server.listen(16)   # backlog 调大：连接交替时让新连接排队，避免偶发 refused
print(f"[OK] Listening on {TCP_HOST}:{TCP_PORT}")


def handle_conn(conn, addr):
    """处理一个 robot_server 连接：TCP<->串口双向转发，直到连接断开或空闲超时。"""
    print(f"[OK] Connected by {addr}")
    stop_evt = threading.Event()

    # 串口 -> TCP：把机械臂返回（如 T=105 的状态）转发回 robot_server。
    def serial_to_tcp(c=conn, ev=stop_evt):
        while not ev.is_set():
            try:
                data = ser.read(4096)
                if data:
                    c.sendall(data)
            except Exception:
                break

    t = threading.Thread(target=serial_to_tcp, daemon=True)
    t.start()

    conn.settimeout(IDLE_TIMEOUT)
    try:
        while True:
            try:
                data = conn.recv(4096)
            except socket.timeout:
                print("[INFO] idle timeout, closing connection")
                break
            if not data:
                print("[INFO] Client disconnected")
                break
            print("[TCP -> COM4]", data)
            ser.write(data)
    finally:
        stop_evt.set()
        t.join(timeout=1.0)
        try:
            conn.close()
        except Exception:
            pass


# 顶层循环：任何一个连接/串口异常都只打印 traceback、不让进程退出。
# 串口失效（SerialException）时尝试重连，避免机械臂猛动拉断线后整个 bridge 挂掉。
while True:
    try:
        print("[INFO] Waiting for WSL robot_server connection...")
        conn, addr = server.accept()
        handle_conn(conn, addr)
    except serial.SerialException:
        print("[ERR] 串口异常，2s 后尝试重连 COM 口:")
        traceback.print_exc()
        try:
            ser.close()
        except Exception:
            pass
        time.sleep(2.0)
        while True:
            try:
                ser = open_serial()
                break
            except Exception:
                print("[ERR] 重连串口失败，2s 后重试 ...", flush=True)
                time.sleep(2.0)
    except Exception:
        print("[ERR] 连接处理异常（进程继续运行）:")
        traceback.print_exc()
        time.sleep(0.5)
