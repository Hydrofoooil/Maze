# Maze

把摄像头拍到的「白纸黑笔迷宫」照片重建成扫描件，并做起点→终点路径规划。
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

## 目录结构
> 所有命令都在仓库根目录 `Maze/` 下运行（根目录后续会加入其他模块，如机械臂规划）。

```
maze_planner/            迷宫重建 + 路径规划模块
  maze_scanner.py        重建流水线（5 步）
  maze_planner.py        路径规划（占用栅格 + A*）
  make_sample.py         生成合成测试照片（透视倾斜 + 不均匀光照 + 红/蓝标记）
  environment.yml        conda 环境定义
  samples/               测试输入照片
    maze_photo.jpg
    test_0.jpg
  outputs/               输出结果
    scanned.png          扫描件
    planned.png          带规划路径的结果图
    debug/               各步骤中间图（0_input ~ 6_occupancy）
```
走迷宫任务相关的所有文件都保存在本仓库内。

## 环境
```bash
conda env create -f maze_planner/environment.yml   # 创建 maze 环境
conda activate maze
```

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

说明：手动选点需要图形界面（本机有显示器即可）。`--corners` 的坐标是**原图**像素，
顺序任意（程序会自动排成左上/右上/右下/左下）。`--debug DIR` 会保存各步骤中间图。
