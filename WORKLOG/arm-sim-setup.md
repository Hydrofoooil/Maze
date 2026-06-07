# arm-sim-setup

## 当前状态（始终覆盖更新）

### 项目理解
- 仿真脚本 `arm_sim/draw_maze.py` 在 Isaac Sim 启动前先用 maze_planner 解算路径，再启动 SimulationApp(headless=True)，IK 驱动笔尖描画，离屏渲染 mp4
- Isaac Sim 5.1.0.0 安装在 conda `dexbench` 环境下（`/data/maoting/miniconda3/envs/dexbench`）
- EULA 通过 `OMNI_KIT_ACCEPT_EULA=YES` 环境变量解决（参考 `/data/maoting/dexaby/env.sh`）
- 所有依赖（opencv-python-headless, imageio, numpy）都在 dexbench 环境里
- API 兼容性已确认：`URDFCreateImportConfig` / `URDFParseAndImportFile` / `isaacsim.core.api` / `isaacsim.sensors.camera` 全部可用

### 进行中的任务
- **已完成**。Slurm 作业 1474 (master 节点, RTX 5880 Ada) 正常跑完
- 输出 `/data/maoting/Maze/arm_draw_maze.mp4`（4.8 MB）已生成

### 关键文件索引
- `arm_sim/draw_maze.py` — 主仿真脚本（迷宫解算 + IK + 渲染）
- `arm_sim/arm_kinematics.py` — 纯 numpy FK/IK，不依赖 Isaac
- `arm_sim/run_draw_maze.sh` — Slurm 提交脚本，使用 dexbench conda 环境
- `maze_planner/samples/test_0.jpg` — 输入迷宫照片
- `arm_draw_maze.mp4` — 预期输出路径

<!-- 最后更新: Claude 2026-06-08 01:11 -->

---

## 变更历史（只追加，不修改）

### [2026-06-08 01:09][Claude] 环境调研 + Slurm 脚本初建

**做了什么**
- 调研发现没有单独的 `maze`/`isaac` conda 环境，`dexbench` 环境已含 Isaac Sim 5.1 及所有依赖
- 确认 EULA 绕过方式：`OMNI_KIT_ACCEPT_EULA=YES`（kit_app.py 第19行）
- 确认 API 兼容：IsaacSim 5.1 的 URDF importer、robots、sensors.camera 接口不变
- 创建 `arm_sim/run_draw_maze.sh`，使用 dexbench 环境，申请 1 GPU，`--auto` 模式跳过交互选角

**关键决策与发现**
- `draw_maze.py` 已经写死用 `test_0.jpg` + `auto=True`，无需改脚本
- Slurm partition = `gpu`，集群有 master/node01/node02/node04，node04 已有一个 dexbench 作业在跑
