"""一键急停：给下位机发 stop，立即阻止后续轨迹点继续下发。

    python arm_real_client/estop.py

注意这是「软停」：
  - 机械臂会停在当前/下一个目标点并**保持姿态**（舵机扭矩仍在，不会瘫软掉落）。
  - 它阻止的是「后续点」；已经发给主控板的当前目标点不会被撤回（轨迹点通常很密，
    所以基本是立即停住）。
  - 它不会断电。真正的硬急停请直接断 12V 电源。
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_client import RobotClient   # noqa: E402

if __name__ == "__main__":
    r = RobotClient()
    print("[ESTOP] 发送 stop ->", r.stop())
    try:
        print("[ESTOP] 当前状态 ->", r.status())
    except Exception as e:
        print("[ESTOP] status 查询失败:", e)
