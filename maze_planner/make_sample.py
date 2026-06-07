"""生成一张带透视倾斜 + 不均匀光照的合成迷宫照片，用于测试 maze_scanner。"""
import os
import cv2, numpy as np

OUT = os.path.join(os.path.dirname(__file__), "samples", "maze_photo.jpg")

# 1) 画一个干净的迷宫(纸面)
paper = np.full((800, 600, 3), 255, np.uint8)
rng = np.random.default_rng(0)
# 外框
cv2.rectangle(paper, (60,60),(540,740),(20,20,20),6)
# 一些内墙
for _ in range(40):
    x1,y1 = rng.integers(80,520), rng.integers(80,720)
    if rng.random()<0.5:
        cv2.line(paper,(x1,y1),(x1+rng.integers(40,160),y1),(20,20,20),5)
    else:
        cv2.line(paper,(x1,y1),(x1,y1+rng.integers(40,160)),(20,20,20),5)
# 1b) 画起点(红)/终点(蓝)标记 —— BGR
cv2.circle(paper, (110, 110), 14, (0, 0, 220), -1)   # 起点: 红, 左上
cv2.circle(paper, (490, 690), 14, (220, 0, 0), -1)   # 终点: 蓝, 右下
# 2) 放到更大的"桌面"背景上并加透视
canvas = np.full((1000,1000,3), 200, np.uint8)
canvas[100:900, 200:800] = paper
src = np.float32([[200,100],[800,100],[800,900],[200,900]])
dst = np.float32([[260,140],[770,90],[820,880],[180,840]])  # 倾斜
M = cv2.getPerspectiveTransform(src,dst)
photo = cv2.warpPerspective(canvas, M, (1000,1000), borderValue=(190,190,190))
# 3) 加不均匀光照(渐变阴影)
yy,xx = np.mgrid[0:1000,0:1000]
shade = (0.55 + 0.45*(xx/1000.0)).astype(np.float32)
photo = np.clip(photo.astype(np.float32)*shade[...,None],0,255).astype(np.uint8)
photo = cv2.GaussianBlur(photo,(3,3),0)
os.makedirs(os.path.dirname(OUT), exist_ok=True)
cv2.imwrite(OUT, photo)
print('sample saved ->', OUT)
