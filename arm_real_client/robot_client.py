import socket
import json
import time

from robot_config import ROBOT_HOST, ROBOT_PORT


class RobotClient:
    def __init__(self, host=ROBOT_HOST, port=ROBOT_PORT, timeout=5):
        self.host = host
        self.port = port
        self.timeout = timeout

    def request(self, payload):
        msg = json.dumps(payload, separators=(",", ":")) + "\n"

        with socket.create_connection((self.host, self.port), timeout=self.timeout) as sock:
            sock.sendall(msg.encode("utf-8"))
            data = sock.recv(4096)

        return json.loads(data.decode("utf-8"))

    def ping(self):
        return self.request({"type": "ping"})

    def status(self):
        return self.request({"type": "status"})

    def state(self):
        return self.request({"type": "state"})

    def stop(self):
        return self.request({"type": "stop"})

    def joint(self, b=0, s=0, e=0, w=0, h=0, spd=10, acc=10):
        return self.request({
            "type": "joint",
            "b": b,
            "s": s,
            "e": e,
            "w": w,
            "h": h,
            "spd": spd,
            "acc": acc,
        })

    def trajectory(self, points, dt=0.1, traj_id="traj", spd=10, acc=10):
        return self.request({
            "type": "trajectory",
            "traj_id": traj_id,
            "dt": dt,
            "spd": spd,
            "acc": acc,
            "points": points,
        })


if __name__ == "__main__":
    robot = RobotClient()

    print("ping:", robot.ping())

    points = [
        {"b": 0, "s": 0, "e": 0, "w": 0, "h": 0},
        {"b": 10, "s": 0, "e": 0, "w": 0, "h": 0},
        {"b": 0, "s": 0, "e": 0, "w": 0, "h": 0},
    ]

    print("trajectory:", robot.trajectory(
        points,
        dt=1.0,
        traj_id="upper_machine_test",
        spd=10,
        acc=10,
    ))

    time.sleep(0.5)
    print("status:", robot.status())
