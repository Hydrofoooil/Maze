# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个「迷宫自动解算 + 机械臂仿真绘制」的端到端系统：
1. **maze_planner**：从手机/相机拍到的迷宫照片，重建出纯黑白扫描件，检测红(起)/蓝(终)标记，用 A* 规划路径。
2. **arm_sim**：在 NVIDIA Isaac Sim 里加载 5-DOF 机械臂 URDF，用解析正运动学(FK) + 阻尼最小二乘逆运动学(IK) 驱动笔尖沿规划路径描画，离屏渲染成 mp4。

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
#   日志: arm_sim/logs/draw_maze_<JOBID>.log    视频: arm_draw_maze.mp4

# 手动交互式跑（需先在 GPU 节点上）：
export OMNI_KIT_ACCEPT_EULA=YES
conda activate dexbench
python arm_sim/draw_maze.py              # -> arm_draw_maze.mp4
python arm_sim/draw_circle.py           # -> arm_draw_circle.mp4
python arm_sim/record_video.py          # -> arm_motion.mp4

# 快速校验取景（FK/IK 正确性 + 一帧截图，不出整段视频）
MAZE_FRAMETEST=1 python arm_sim/draw_circle.py
MAZE_FRAMETEST=1 python arm_sim/draw_maze.py
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
- 排查路径规划问题看 `debug/6_occupancy.png`（白 = 可走格）

### arm_sim 模块

**`arm_kinematics.py`** — 纯 numpy FK/IK，不依赖 Isaac  
按 `urdf/five_dof_arm.urdf` 的链接定义硬编码了 5 个关节的齐次变换 `_JOINTS`。`fk_M5(q)` 返回 link_5 坐标系到 world 的 4×4 矩阵；`fk_nib_tail(q)` 由此算出笔尖/笔尾世界坐标（偏移量 `_NIB`/`_TAIL` 通过 PCA 从 link_5.stl 获得）。`ik(target, q0)` 两阶段阻尼最小二乘：先收敛位置，再加「笔轴竖直向下」约束，返回 `(q, 位置残差, 笔轴偏离角)`。

**`draw_circle.py` / `draw_maze.py` / `record_video.py`** — Isaac Sim 脚本  
三个脚本结构相同：启动 `SimulationApp(headless=True)` → 加载 URDF → 建场景（地面+灯光+纸面/纹理） → 规划轨迹（IK） → 逐帧 `set_joint_positions` + `cam.get_rgba()` → `imageio` 写 mp4。  
`draw_maze.py` 在启动 Isaac 之前先在 maze 模块里解算路径并生成 `arm_sim/_maze_tex.png` 纹理贴到纸面，路径点用 `img_to_world()` 从像素坐标映射到纸面物理坐标。

坐标系约定：纸面位于 `(PAPER_CX=0.22, PAPER_CY=0.0)` 正前方地面，30cm×21cm，笔尖目标 z = 纸面上表面 + 1mm。

**`urdf/five_dof_arm.urdf`** — 机械臂描述  
5 个旋转关节，每个关节的 origin/axis 直接对应 `arm_kinematics.py` 里的 `_JOINTS` 硬编码参数。修改 URDF 后必须同步更新 `_JOINTS`，并用 `MAZE_FRAMETEST=1` 脚本验证 FK 误差 < 0.1mm。
