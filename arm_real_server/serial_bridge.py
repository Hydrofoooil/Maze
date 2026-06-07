import socket
import threading
import serial
import time

SERIAL_PORT = "COM4"
BAUD = 115200

# 监听本机所有地址，WSL 可以通过 Windows host IP 连接
TCP_HOST = "0.0.0.0"
TCP_PORT = 9100

ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.05)
time.sleep(2)

print(f"[OK] Opened serial {SERIAL_PORT} @ {BAUD}")
print(f"[OK] Listening on {TCP_HOST}:{TCP_PORT}")

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((TCP_HOST, TCP_PORT))
server.listen(1)

while True:
    print("[INFO] Waiting for WSL robot_server connection...")
    conn, addr = server.accept()
    print(f"[OK] Connected by {addr}")

    running = True

    def serial_to_tcp():
        nonlocal_running = True
        while nonlocal_running:
            try:
                data = ser.read(4096)
                if data:
                    conn.sendall(data)
            except Exception as e:
                print("[ERR] serial_to_tcp:", e)
                break

    t = threading.Thread(target=serial_to_tcp, daemon=True)
    t.start()

    try:
        while True:
            data = conn.recv(4096)
            if not data:
                print("[INFO] Client disconnected")
                break
            print("[TCP -> COM4]", data)
            ser.write(data)
    except Exception as e:
        print("[ERR] tcp_to_serial:", e)

    try:
        conn.close()
    except Exception:
        pass