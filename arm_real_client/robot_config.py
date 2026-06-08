"""下位机连接 + 运动参数配置（唯一真源）。

连接地址和运动参数默认值都集中在此一处，所有上位机脚本（robot_client / test_client /
draw_maze_real）都从这里导入。改这里一处即可全局生效，避免散落写死、漏改导致不一致。
"""

ROBOT_HOST = "100.127.110.20"   # 下位机地址（tailscale 固定 IP）
ROBOT_PORT = 9001               # 下位机 robot_server 监听端口

# 运动参数默认值：spd 角速度(°/s)、acc 角加速度。改这里，所有上位机入口统一生效；
# 命令行/调用处显式传参仍可临时覆盖（如 draw_maze_real.py --acc 5）。
DEFAULT_SPD = 10
DEFAULT_ACC = 5
