from robot_client import RobotClient

robot = RobotClient(host="10.196.101.150", port=9001)

points = [
    {"b": 10, "s": 0, "e": 0, "w": 0, "h": 0},
    {"b": 20, "s": 0, "e": 0, "w": 0, "h": 0},
    {"b": 10, "s": 0, "e": 0, "w": 0, "h": 0},
    {"b": 0, "s": 0, "e": 0, "w": 0, "h": 0},
]

print(robot.trajectory(points, dt=1.0, traj_id="test", spd=10, acc=10))
print(robot.status())