"""
Maze planner: 在重建后的迷宫上做起点->终点路径规划。

输入是一张迷宫照片，内部先调用 maze_scanner 做重建，然后:
  1. 从彩色矫正图里检测起点(绿)/终点(红)标记
  2. 把二值扫描件转成占用栅格 (墙=障碍, 纸=可走)
     - 抹掉标记处的黑块（标记不是墙）
     - 闭运算补上墙体细缝（防止穿墙）
     - 按机器人半径膨胀墙体（防止贴墙/抹角）
     - 降采样到便于搜索的分辨率
  3. A* (8 邻接) 搜索起点->终点
  4. 把路径画回彩色图并保存

用法:
  python maze_planner.py input.jpg -o planned.png
  python maze_planner.py input.jpg -o planned.png --debug debug_dir
"""

import argparse
import heapq
import os
from collections import deque

import cv2
import numpy as np

from maze_scanner import scan, parse_corners


# ----------------------------------------------------------------------------
# 1. 检测起点/终点彩色标记
# ----------------------------------------------------------------------------
def _largest_blob_centroid(mask, min_area=30):
    """返回 mask 中最大连通块的质心 (x, y)，没有则返回 None。"""
    n, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    best, best_area = None, 0
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area >= min_area and area > best_area:
            best_area, best = area, centroids[i]
    if best is None:
        return None
    return (float(best[0]), float(best[1]))


def detect_markers(bgr):
    """
    从彩色图检测起点(红)和终点(蓝)。返回 (start_xy, goal_xy)，找不到为 None。
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    k = np.ones((5, 5), np.uint8)

    # 红: hue 在 0 附近和 180 附近两段
    red = cv2.inRange(hsv, (0, 80, 40), (10, 255, 255)) | \
          cv2.inRange(hsv, (170, 80, 40), (180, 255, 255))
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, k)

    # 蓝: hue ~ 100..130
    blue = cv2.inRange(hsv, (100, 80, 40), (130, 255, 255))
    blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, k)

    return _largest_blob_centroid(red), _largest_blob_centroid(blue)


# ----------------------------------------------------------------------------
# 2. 二值扫描件 -> 占用栅格
# ----------------------------------------------------------------------------
def build_occupancy(binary, markers, close=5, inflate=1, grid_max=400):
    """
    把二值图 (墙=黑) 转成占用栅格。返回:
      occ        : bool 数组, True=障碍 (降采样后)
      scale      : 原图坐标 * scale = 栅格坐标
    markers: 需要从墙里抹掉的点列表 (原图坐标)，避免标记被当成墙。
    inflate: 机器人半径，单位=栅格格数 (与分辨率无关；调大=离墙更远但更易堵)。
    """
    h0, w0 = binary.shape
    wall = (binary < 128).astype(np.uint8)  # 1=墙

    # 抹掉标记处的黑块（半径按图像尺寸取，连同抗锯齿边一起清掉）
    erase_r = max(12, int(0.02 * max(h0, w0)))
    for m in markers:
        if m is not None:
            cv2.circle(wall, (int(round(m[0])), int(round(m[1]))),
                       erase_r, 0, -1)

    # 闭运算: 把手绘墙的细缝补上，避免规划穿墙 (在原分辨率做更准)
    if close > 0:
        wall = cv2.morphologyEx(wall, cv2.MORPH_CLOSE,
                                np.ones((close, close), np.uint8))

    # 降采样到 grid_max（块内有任意墙像素即视为墙，保守，不会把通道误开）
    scale = grid_max / float(max(h0, w0))
    if scale < 1.0:
        gh, gw = max(int(h0 * scale), 1), max(int(w0 * scale), 1)
        wall = (cv2.resize(wall * 255, (gw, gh),
                           interpolation=cv2.INTER_AREA) > 0).astype(np.uint8)
    else:
        scale = 1.0

    # 膨胀墙体 = 给机器人留出半径间隙 (在栅格空间，单位与分辨率无关)
    if inflate > 0:
        wall = cv2.dilate(wall, np.ones((2 * inflate + 1,) * 2, np.uint8))

    return wall.astype(bool), scale


def _nearest_free(occ, cell):
    """若 cell 落在障碍里，BFS 找最近的可走格。"""
    h, w = occ.shape
    r, c = cell
    r = min(max(r, 0), h - 1)
    c = min(max(c, 0), w - 1)
    if not occ[r, c]:
        return (r, c)
    seen = {(r, c)}
    q = deque([(r, c)])
    while q:
        cr, cc = q.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = cr + dr, cc + dc
            if 0 <= nr < h and 0 <= nc < w and (nr, nc) not in seen:
                if not occ[nr, nc]:
                    return (nr, nc)
                seen.add((nr, nc))
                q.append((nr, nc))
    return None


# ----------------------------------------------------------------------------
# 3. A* 搜索 (8 邻接)
# ----------------------------------------------------------------------------
def astar(occ, start, goal):
    """occ: True=障碍。start/goal: (row, col)。返回格点路径列表或 None。"""
    h, w = occ.shape
    nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]

    def heur(a, b):
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    open_heap = [(heur(start, goal), 0.0, start)]
    came = {}
    gscore = {start: 0.0}
    while open_heap:
        _, g, cur = heapq.heappop(open_heap)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        if g > gscore.get(cur, float("inf")):
            continue
        cr, cc = cur
        for dr, dc, cost in nbrs:
            nr, nc = cr + dr, cc + dc
            if not (0 <= nr < h and 0 <= nc < w) or occ[nr, nc]:
                continue
            # 禁止从两墙之间的对角缝穿过
            if dr != 0 and dc != 0 and (occ[cr + dr, cc] and occ[cr, cc + dc]):
                continue
            ng = g + cost
            if ng < gscore.get((nr, nc), float("inf")):
                gscore[(nr, nc)] = ng
                came[(nr, nc)] = cur
                heapq.heappush(open_heap, (ng + heur((nr, nc), goal), ng, (nr, nc)))
    return None


# ----------------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------------
def plan(path, debug_dir=None, inflate=1, close=5, grid_max=400,
         manual=True, auto=False, corners=None):
    binary, warped = scan(path, debug_dir=debug_dir,
                          manual=manual, auto=auto, corners=corners)

    def dbg(name, im):
        if debug_dir:
            os.makedirs(debug_dir, exist_ok=True)
            cv2.imwrite(os.path.join(debug_dir, name), im)

    # 1. 检测标记
    start_xy, goal_xy = detect_markers(warped)
    if start_xy is None or goal_xy is None:
        raise RuntimeError(
            f"未检测到起点/终点标记 (红={start_xy}, 蓝={goal_xy})。"
            "请确认照片里画了红色起点和蓝色终点。")

    # 2. 占用栅格
    occ, scale = build_occupancy(binary, [start_xy, goal_xy],
                                 close=close, inflate=inflate, grid_max=grid_max)
    dbg("6_occupancy.png", (~occ).astype(np.uint8) * 255)  # 白=可走

    # 原图坐标 -> 栅格 (row, col)
    s_cell = (int(round(start_xy[1] * scale)), int(round(start_xy[0] * scale)))
    g_cell = (int(round(goal_xy[1] * scale)), int(round(goal_xy[0] * scale)))
    s_cell = _nearest_free(occ, s_cell)
    g_cell = _nearest_free(occ, g_cell)
    if s_cell is None or g_cell is None:
        raise RuntimeError("起点或终点被墙完全包围，无法规划。")

    # 3. A*
    cells = astar(occ, s_cell, g_cell)
    if cells is None:
        raise RuntimeError("起点到终点之间没有可走通路（可能墙体把通道封死了）。")

    # 4. 画回彩色图
    vis = warped.copy()
    pts = [(int(round(c / scale)), int(round(r / scale))) for r, c in cells]
    for i in range(1, len(pts)):
        cv2.line(vis, pts[i - 1], pts[i], (255, 0, 255), 3)
    cv2.circle(vis, (int(start_xy[0]), int(start_xy[1])), 10, (0, 0, 220), -1)   # 起点红
    cv2.circle(vis, (int(goal_xy[0]), int(goal_xy[1])), 10, (220, 0, 0), -1)     # 终点蓝

    print(f"✓ 路径规划成功: {len(cells)} 个格点 "
          f"(栅格 {occ.shape[1]}x{occ.shape[0]}, scale={scale:.3f})")
    return vis


def main():
    ap = argparse.ArgumentParser(description="迷宫照片 -> 起点到终点路径规划")
    ap.add_argument("input", help="输入照片路径")
    ap.add_argument("-o", "--output", default="planned.png", help="输出路径")
    ap.add_argument("--inflate", type=int, default=1, help="机器人半径(栅格格数)")
    ap.add_argument("--close", type=int, default=5, help="补墙缝的闭运算核大小")
    ap.add_argument("--grid-max", type=int, default=400, help="搜索栅格最长边")
    ap.add_argument("--debug", metavar="DIR", help="保存中间结果")
    ap.add_argument("--auto", action="store_true",
                    help="用自动检测纸张边界 (默认是手动点选4角)")
    ap.add_argument("--corners", metavar="\"x1,y1 x2,y2 x3,y3 x4,y4\"",
                    help="直接给出原图坐标的4个角点，跳过选点")
    args = ap.parse_args()

    vis = plan(args.input, debug_dir=args.debug,
               inflate=args.inflate, close=args.close, grid_max=args.grid_max,
               auto=args.auto, manual=not args.auto,
               corners=parse_corners(args.corners))
    cv2.imwrite(args.output, vis)
    print(f"✓ 已保存: {args.output}")


if __name__ == "__main__":
    main()
