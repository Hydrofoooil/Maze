from robot_client import RobotClient

robot = RobotClient()

points = [
    {"b": 0, "s": 0, "e": 0, "w": 0, "h": 0},
    {"b": 30, "s": 0, "e": 0, "w": 0, "h": 0},
    {"b": 45, "s": 0, "e": 0, "w": 0, "h": 0},
    {"b": 0, "s": 0, "e": 0, "w": 0, "h": 0},
]

print(robot.trajectory(points, dt=1.0, traj_id="test", spd=10, acc=10))
print(robot.status())