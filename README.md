# Maze

把摄像头拍到的「白纸黑笔迷宫」照片重建成扫描件，做起点→终点路径规划，
并可在 NVIDIA Isaac Sim 里让 5-DOF 机械臂用笔沿规划路径描画、渲染成视频。
- 起点用**红笔**标记，终点用**蓝笔**标记。

## 一、重建 (maze_scanner.py)
1. 确定纸张四角（**默认手动点选**，最可靠）：弹窗后左键依次点 4 个角，
   `u`/退格撤销、回车确认、Esc 取消。可选 `--auto` 改用自动检测（亮度+低饱和分割
   →取含画面中心的连通块→凸包求四边形），或 `--corners` 直接传坐标跳过选点。
2. 透视矫正（四点透视变换拉正，角点向内缩 `margin` 去掉边缘桌面/阴影）
3. 亮度/对比度增强（背景除法去阴影 + CLAHE）
4. 黑白二值化（自适应阈值）
5. 连通域清噪，输出纯黑白扫描件（墙=黑，纸=白）

## 二、路径规划 (maze_planner.py)
1. 从彩色矫正图检测起点(红)/终点(蓝)标记
2. 二值图 → 占用栅格：抹掉标记黑块 → 闭运算补墙缝 → 按机器人半径膨胀墙体 → 降采样
3. A*（8 邻接，禁止对角穿墙缝）搜索
4. 把路径画回彩色图保存

可调参数：`--inflate` 机器人半径(**栅格格数**，与分辨率无关，默认1；调大更安全但易堵)、
`--close` 补墙缝核(默认5)、`--grid-max` 搜索栅格最长边(默认400)。
排查时看 `debug/6_occupancy.png`（白=可走）确认墙体连续、通道没被堵死。

## 三、机械臂仿真绘制 (arm_sim)

把规划好的迷宫路径交给一只 5-DOF 机械臂，在 NVIDIA Isaac Sim 里用笔竖直地描出来，
离屏渲染成 mp4。三个脚本（视频都输出到仓库根目录）：

- `record_video.py`  —— 5 个关节按正弦编排自由运动，纯演示 → `arm_motion.mp4`
- `draw_circle.py`   —— 笔尖在纸面上画一个圆，验证 FK/IK 链路 → `arm_draw_circle.mp4`
- `draw_maze.py`     —— 自动解算 `samples/test_0.jpg` 的迷宫路径，再让笔尖沿路径描画
  （内部调用第二步的规划，固定 `auto=True` 不弹窗）→ `arm_draw_maze.mp4`

运动学在 `arm_kinematics.py`（纯 numpy，不依赖 Isaac）：
- FK 按 URDF 里各关节的变换链式相乘，已和 Isaac 实际笔尖位姿对拍到 0.00mm
- IK 用两段式阻尼最小二乘：先把笔尖收敛到目标位置，再加「笔轴竖直向下」约束
- 全程纯运动学控制（直接设关节角），关掉重力，不依赖连杆质量惯量

### 环境与运行

仿真依赖 NVIDIA Isaac Sim（这里用 conda 环境 `dexbench`，已装 Isaac Sim 5.1 +
`opencv-python-headless` + imageio），和上面 maze 用的 `opencv-python` 是**两套独立环境**
（一个无头渲染、一个带 GUI 点选，互不混用）。

```bash
# 在 Slurm 集群上提交（推荐，自动申请 GPU）
sbatch arm_sim/run_draw_maze.sh
#   日志: arm_sim/logs/draw_maze_<JOBID>.log    视频: arm_draw_maze.mp4

# 或在有 GPU 的机器上直接跑
export OMNI_KIT_ACCEPT_EULA=YES        # 接受 Omniverse EULA（首次必需）
conda activate dexbench
python arm_sim/draw_maze.py            # 或 draw_circle.py / record_video.py

# 只校验 FK/IK + 取景一帧（不出整段视频，快）
MAZE_FRAMETEST=1 python arm_sim/draw_maze.py
```

## 目录结构
> 所有命令都在仓库根目录 `Maze/` 下运行。

```
maze_planner/            迷宫重建 + 路径规划模块
  maze_scanner.py        重建流水线（5 步）
  maze_planner.py        路径规划（占用栅格 + A*）
  make_sample.py         生成合成测试照片（透视倾斜 + 不均匀光照 + 红/蓝标记）
  environment.yml        maze conda 环境定义
  samples/               测试输入照片（maze_photo.jpg, test_0.jpg）
  outputs/               输出（scanned.png, planned.png, debug/）
arm_sim/                 机械臂仿真绘制模块（Isaac Sim）
  arm_kinematics.py      5-DOF 正/逆运动学（纯 numpy）
  record_video.py        关节正弦运动演示
  draw_circle.py         画圆（验证 FK/IK）
  draw_maze.py           沿迷宫解路径描画
  run_draw_maze.sh       Slurm 提交脚本
urdf/                    机械臂模型
  five_dof_arm.urdf      5 个旋转关节，参数与 arm_kinematics 对应
  meshes/                各连杆 STL
arm_motion.mp4           关节运动演示（根目录产物）
arm_draw_circle.mp4      画圆结果
arm_draw_maze.mp4        迷宫描画结果
```
走迷宫任务相关的所有文件都保存在本仓库内。

## 环境
```bash
conda env create -f maze_planner/environment.yml   # 创建 maze 环境
conda activate maze
```
环境装的是带 GUI 窗口的 `opencv-python`（非 headless），手动点选纸角的弹窗依赖它。
如果用的是 `opencv-python-headless`，弹窗会报 `The function is not implemented. Rebuild
the library with ... GTK+ support`——这时要么换成非 headless 版，要么加 `--auto` 跳过点选。

## 用法
```bash
# 处理你自己的照片：默认弹窗手动点选 4 个纸角
python maze_planner/maze_planner.py input.jpg -o planned.png        # 重建 + 规划
python maze_planner/maze_scanner.py input.jpg -o scanned.png        # 只重建

# 不想手动点选时：
python maze_planner/maze_planner.py input.jpg -o planned.png --auto # 自动检测纸角
python maze_planner/maze_planner.py input.jpg -o planned.png \
       --corners "x1,y1 x2,y2 x3,y3 x4,y4"                          # 直接给坐标(原图像素)

# 合成图自测（边缘干净，用 --auto 免去弹窗）
python maze_planner/make_sample.py
python maze_planner/maze_planner.py maze_planner/samples/maze_photo.jpg \
       -o maze_planner/outputs/planned.png --auto --debug maze_planner/outputs/debug
```

说明：手动选点需要图形界面。`--corners` 的坐标是**原图**像素，
顺序任意（程序会自动排成左上/右上/右下/左下）。`--debug DIR` 会保存各步骤中间图。

## 远程使用（SSH + X11 转发）

没有本机显示器、通过 SSH 远程用时，手动点选窗口靠 X11 转发显示到你本地：

1. 用 `ssh -X`（或更宽松的 `-Y`）登录，让服务器的窗口转发到本地。
2. 本地要有 X server：Windows 11 + WSL2 自带 WSLg，开箱即用；Win10 / 无 WSLg 需装
   VcXsrv 或 X410 并启动。
3. 测链路：远程跑 `xeyes`，本地能弹出一双眼睛就说明通了。
4. **如果在 tmux / screen 里**：复用旧会话时 `DISPLAY` 可能还是过期的空值（会话创建时被
   冻结了），导致弹窗失败。从会话缓存刷新到当前 shell：
   ```bash
   export DISPLAY=$(tmux show-environment | sed -n 's/^DISPLAY=//p')   # tmux
   ```
5. 然后照常 `python maze_planner/maze_planner.py input.jpg -o planned.png`。

弹窗时出现 `QFontDatabase: Cannot find font directory` 是无害警告，可忽略。
完全没有图形界面时，用 `--auto` 自动检测纸角，或 `--corners` 直接给坐标。
