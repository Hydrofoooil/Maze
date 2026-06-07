# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个「迷宫自动解算 + 机械臂仿真绘制」的端到端系统：
1. **maze_planner**：从手机/相机拍到的迷宫照片，重建出纯黑白扫描件，检测红(起)/蓝(终)标记，用 A* 规划路径。
2. **arm_sim**：在 NVIDIA Isaac Sim 里加载 5-DOF 机械臂 URDF，用解析正运动学(FK) + 阻尼最小二乘逆运动学(IK) 驱动笔尖沿规划路径描画，离屏渲染成 mp4。
3. **arm_real**：通过上下位机 TCP 链路把规划好的关节轨迹下发给**真实机械臂**执行。上位机（本机）跑 `arm_real_client`（`draw_maze_real.py` 串起「迷宫规划→IK→关节轨迹→下发」全流程），下位机（Windows+WSL）跑 `arm_real_server`。

## 运行环境

两个模块各用一个 conda 环境，**职责不同，OpenCV 版本也故意不同**：

| 环境 | 用途 | OpenCV | 算力 |
|---|---|---|---|
| `maze` | maze_planner 独立运行（含手动点选 GUI） | `opencv-python`（Qt5 GUI，**非 headless**） | 纯 CPU |
| `dexbench` | arm_sim 仿真（Isaac Sim 5.1） | `opencv-python-headless`（无 GUI） | 需 GPU |

- `maze` 环境创建：`conda env create -f maze_planner/environment.yml`（python3.10 + numpy + opencv-python 4.13，Qt5 GUI）。它的 OpenCV 带窗口支持，所以能弹窗手动点选纸角。
- `dexbench` 环境已预装 Isaac Sim 5.1 及全部依赖（opencv-python-headless、imageio、numpy），路径 `/data/maoting/miniconda3/envs/dexbench`。**不要往这个环境装非 headless 的 opencv**，会和 Isaac Sim 的依赖冲突。
- 跑 Isaac Sim 需要接受 EULA：`export OMNI_KIT_ACCEPT_EULA=YES`（已写进 Slurm 脚本）。

⚠️ 两个环境的 OpenCV 不能混用：headless 版跑 GUI 会报「rebuild with GTK+ support」；非 headless 版装进 Isaac 环境会依赖冲突。服务器无头渲染用 headless，本地手动点选用带 GUI 的。

## 常用命令

所有命令都在仓库根目录 `Maze/` 下运行。

### maze_planner

```bash
# 合成测试照片（含透视倾斜 + 不均匀光照 + 红/蓝标记）
python maze_planner/make_sample.py

# 重建 + 路径规划（默认弹窗手动点 4 个纸角，需要显示器）
python maze_planner/maze_planner.py INPUT.jpg -o OUTPUT.png

# 用 --auto 跳过弹窗（适合 CI / SSH 无头环境）
python maze_planner/maze_planner.py INPUT.jpg -o OUTPUT.png --auto --debug maze_planner/outputs/debug

# 典型自测（合成图 + 自动检测）
python maze_planner/make_sample.py
python maze_planner/maze_planner.py maze_planner/samples/maze_photo.jpg \
       -o maze_planner/outputs/planned.png --auto --debug maze_planner/outputs/debug

# 只做重建
python maze_planner/maze_scanner.py INPUT.jpg -o scanned.png --auto
```

### 远程手动点选：X11 转发 + tmux 缓存坑

maze_planner.py 默认弹窗手动点选 4 角（`pick_corners_interactive`），远程 SSH 使用时：

1. 用带 X11 转发的 ssh 登录：`ssh -X`（或 `-Y`）。集群 sshd 已开 `X11Forwarding yes`。
2. **若在 tmux 里**：tmux server 创建时把环境变量冻结了，attach 后已有 pane 的 `DISPLAY` 仍是旧值（空）。需从 tmux 缓存刷新到当前 shell：
   ```bash
   export DISPLAY=$(tmux show-environment | sed -n 's/^DISPLAY=//p')
   ```
3. 测链路：`xset q`（能返回信息）或 `xeyes`（本地能弹窗）即通。cookie 在 `~/.Xauthority`，`XAUTHORITY` 不用单独设。
4. 跑：`conda activate maze && python maze_planner/maze_planner.py <img> -o out.png`

- maze_planner 是纯 CPU 任务，直接在登录节点 `master` 跑即可，**不必 srun**（避免登录节点→计算节点二次转发）。
- Qt 的 `QFontDatabase: Cannot find font directory` 是无害警告，不影响点选。
- 无图形界面时一律加 `--auto`（自动检测纸角）跳过 GUI。

### arm_sim（dexbench 环境 + GPU，走 Slurm）

```bash
# 提交迷宫绘制仿真作业（自动解算路径 + IK + 渲染，draw_maze.py 内部用 auto=True 不弹 GUI）
sbatch arm_sim/run_draw_maze.sh
#   日志: arm_sim/logs/draw_maze_<JOBID>.log    视频: arm_sim/video/arm_draw_maze.mp4

# 手动交互式跑（需先在 GPU 节点上）：
export OMNI_KIT_ACCEPT_EULA=YES
conda activate dexbench
python arm_sim/draw_maze.py              # -> arm_sim/video/arm_draw_maze.mp4
python arm_sim/draw_circle.py           # -> arm_sim/video/arm_draw_circle.mp4
python arm_sim/record_video.py          # -> arm_sim/video/arm_motion.mp4

# 快速校验取景（FK/IK 正确性 + 一帧截图，不出整段视频）
MAZE_FRAMETEST=1 python arm_sim/draw_circle.py
MAZE_FRAMETEST=1 python arm_sim/draw_maze.py
```

### arm_real（实机控制，完整部署见 README「四、实机控制」）

```bash
# 下位机 Windows：起串口桥（监听 9100，写 COM4@115200）
python serial_bridge.py
# 下位机 WSL：起 robot server（监听 9000，转发到 bridge 9100）
python3 robot_server.py
# 下位机 Windows：portproxy 把对外 9001 → WSL 9000（管理员 PowerShell，一次性）
netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=9001 connectaddress=127.0.0.1 connectport=9000

# 上位机（本机）：发一条测试轨迹
cd arm_real_client && python test_client.py

# 上位机：从迷宫规划到实机绘制（draw_maze_real.py，默认 dry-run 只规划+校验不动机械臂）
conda activate maze
python arm_real_client/draw_maze_real.py                                    # 规划→存轨迹/中间图/planned图 + 校验
python arm_real_client/draw_maze_real.py --send --from-file --max-points 5  # 首次：从轨迹文件只发前 5 点试探
python arm_real_client/draw_maze_real.py --send --from-file                 # 确认后从轨迹文件发全程
```

## 代码架构

### maze_planner 模块

两个文件有明确的上下游关系：`maze_scanner.py` 是纯图像处理管线，`maze_planner.py` 调用它并在上面做路径规划。

**`maze_scanner.py`** — 图像 → 扫描件  
5 步流水线：`find_paper_contour`（自动检测纸角） → `four_point_transform`（透视矫正） → `enhance`（背景除法去阴影 + CLAHE） → `binarize`（自适应阈值） → `clean`（连通域去噪）。  
对外暴露 `scan(path, ...)` 函数，返回 `(binary, warped)`——二值扫描件和透视矫正后的彩色图。

**`maze_planner.py`** — 扫描件 → 路径  
调用 `scan()` 后，`detect_markers(warped)` 从彩色矫正图里用 HSV 阈值找红(起)/蓝(终)标记质心；`build_occupancy(binary, markers, ...)` 把二值图转成 bool 占用栅格（抹掉标记黑块 → 闭运算补墙缝 → 膨胀给机器人让路 → 降采样到 `grid_max` 格）；`astar(occ, start, goal)` 做 8 邻接 A*（禁对角穿缝）；最终把路径画回彩色图。

关键参数：
- `--inflate`：机器人半径（栅格格数，默认 1），调大离墙更远但易堵死通道
- `--close`：补墙缝闭运算核（默认 5），手绘墙缝隙大时调大
- `--grid-max`：搜索栅格最长边（默认 400），值小搜索快但精度低
- 排查路径规划问题看 `outputs/image/6_occupancy.png`（白 = 可走格）；`--debug` 默认输出到 `outputs/image`，每次覆盖

### arm_sim 模块

**`arm_kinematics.py`** — 纯 numpy FK/IK，不依赖 Isaac  
按 `urdf/five_dof_arm.urdf` 的链接定义硬编码了 5 个关节的齐次变换 `_JOINTS`。`fk_M5(q)` 返回 link_5 坐标系到 world 的 4×4 矩阵；`fk_nib_tail(q)` 由此算出笔尖/笔尾世界坐标（偏移量 `_NIB`/`_TAIL` 通过 PCA 从 link_5.stl 获得）。`ik(target, q0)` 两阶段阻尼最小二乘：先收敛位置，再加「笔轴竖直向下」约束，返回 `(q, 位置残差, 笔轴偏离角)`。

**`draw_circle.py` / `draw_maze.py` / `record_video.py`** — Isaac Sim 脚本  
三个脚本结构相同：启动 `SimulationApp(headless=True)` → 加载 URDF → 建场景（地面+灯光+纸面/纹理） → 规划轨迹（IK） → 逐帧 `set_joint_positions` + `cam.get_rgba()` → `imageio` 写 mp4 到 `arm_sim/video/`。  
`draw_maze.py` 在启动 Isaac 之前先在 maze 模块里解算路径并生成 `arm_sim/_maze_tex.png` 纹理贴到纸面，路径点用 `img_to_world()` 从像素坐标映射到纸面物理坐标；中间图输出到 `maze_planner/outputs/image`。

坐标系约定：纸面位于 `(PAPER_CX=0.22, PAPER_CY=0.0)` 正前方地面，30cm×21cm，笔尖目标 z = 纸面上表面 + 1mm。

**`urdf/five_dof_arm.urdf`** — 机械臂描述  
5 个旋转关节，每个关节的 origin/axis 直接对应 `arm_kinematics.py` 里的 `_JOINTS` 硬编码参数。修改 URDF 后必须同步更新 `_JOINTS`，并用 `MAZE_FRAMETEST=1` 脚本验证 FK 误差 < 0.1mm。

### arm_real 模块（真实机械臂控制）

上下位机 TCP 链路把关节轨迹下发到实机，详细部署与排错见 README「四、实机控制」。链路：
`client →(9001 portproxy)→ robot_server(9000) →(9100)→ serial_bridge → COM4 → 主控板`。

**`arm_real_client/robot_config.py`** — 下位机连接配置（唯一真源）  
`ROBOT_HOST` / `ROBOT_PORT` 集中在此，robot_client / test_client / draw_maze_real 都从这里导入。**改下位机 IP 只动这一个文件**，别在各脚本里写死。

**`arm_real_client/robot_client.py`** — 上位机（本机）客户端  
`RobotClient` 类，`host`/`port` 默认取自 `robot_config`，方法 `ping/joint/trajectory/status/state/stop`。每个请求是一条单行 JSON、一来一回的短连接。`test_client.py` 是发轨迹的最小示例。

**`arm_real_client/draw_maze_real.py`** — 端到端：迷宫照片 → 实机绘制  
复用 `maze_planner.solve_path`（路径）+ `arm_kinematics.ik`（5-DOF 解关节角）+ `RobotClient`（下发）。关节映射 q[0..4](rad)→b/s/e/w/h(deg)，第 5 关节是改装后的笔旋转件（非夹爪）。规划产物（每次覆盖）：关节轨迹 `outputs/trajectory/trajectory.json`、中间图与轨迹投影结果图 `outputs/image/`（含 `planned.png`）。默认 dry-run（只规划+校验不碰硬件）；`--send` 真发、`--max-points N` 首次试探、`--from-file` 直接读轨迹文件发送。只需 `maze` 环境（不依赖 GPU/Isaac）。

**`arm_real_server/robot_server.py`** — 下位机 WSL 端，监听 9000  
`handle_request` 分发请求；`make_arm_cmd` 做关节限幅并转成底层 `T=122` 串口指令；trajectory 在独立线程**逐点闭环执行**：`send_point_and_wait` 发一个点后、在同一条 bridge 连接上轮询 `T=105` 状态，检测「角度收敛」（b/s/e/w 连续几次几乎不再变=机械臂已停）才发下一个；探测不到状态返回时回退按 `dt` 定时。状态存 `server_state`（idle/running/done/stopped/error），`stop_event` 中断（只阻止后续点，非物理急停）。  
限幅 `LIMITS`：b±180 / s±90 / e±90 / w±90 / **h±180**（夹爪已拆、改装为与笔固定的旋转件，故放宽；原装夹爪 ±45）。IK 解画竖直笔需手腕约 -93°，超 ±90 被 clamp 到 -90（笔约 3° 恒定倾斜）。

**实测坑（决定了为何用「角度收敛」判到位）：**
- `s`(肩，步进电机)的 `T=105` 返回**符号与指令相反**，但**实际动作方向正确**（只是读数符号反，画图不受影响，无需改 draw_maze_real）。
- `move` 字段**不可靠**：机械臂停了仍可能 `=1`，不能用来判到位。
- `e` 在 0° 附近有 ~3.5° 稳态误差（重力下垂）。
- `serial_bridge` 需保活：`recv` 空闲超时 + 串口断开自动重连 + 顶层异常不退出（机械臂猛动曾拉断 USB / 卡死 bridge）。

改此文件后需手动同步到下位机。

**`arm_real_server/serial_bridge.py`** — 下位机 Windows 端，监听 9100  
把收到的 JSON 写入 `COM4@115200`。WSL 不能直接访问 USB 串口，所以靠这个 bridge 中转。当前是「每个控制点建一次 TCP 连接、发完即断」。

⚠️ `arm_real_server/` 的两个文件是要**部署到下位机**的（robot_server → WSL `~/robot_arm/`，serial_bridge → Windows 桌面），仓库里只是存档，改完需手动同步过去。
