"""下位机连接 + 运动参数配置（唯一真源）。

连接地址和运动参数默认值都集中在此一处，所有上位机脚本（robot_client / test_client /
draw_maze_real）都从这里导入。改这里一处即可全局生效，避免散落写死、漏改导致不一致。
"""

import os


ROBOT_HOST = os.environ.get("ROBOT_HOST", "127.0.0.1")  # 同一台电脑默认连本机
ROBOT_PORT = int(os.environ.get("ROBOT_PORT", "9001"))  # 下位机 robot_server 监听端口

# 运动参数默认值：spd 角速度(°/s)、acc 角加速度、dt 相邻轨迹点的时间间隔(s)。
# 改这里，所有上位机入口统一生效；命令行/调用处显式传参仍可临时覆盖（如 --acc 5 / --dt 0.1）。
# dt 语义：开环版(robot_server_openloop)是「点间隔」——越小线越连续越快；
#          闭环版(robot_server)是「到位后额外停留时长」。
DEFAULT_SPD = 5
DEFAULT_ACC = 10
DEFAULT_DT = 0.1
