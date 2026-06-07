"""下位机连接配置（唯一真源）。

IP / 端口集中在此一处，所有上位机脚本（robot_client / test_client / draw_maze_real）
都从这里导入。改下位机地址时只动这一个文件，避免散落写死、漏改导致不一致。
"""

ROBOT_HOST = "100.127.110.20"   # 下位机地址（Windows portproxy 对外 IP）
ROBOT_PORT = 9001               # 下位机端口（portproxy 对外，转发到 WSL robot_server 9000）
