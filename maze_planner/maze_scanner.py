"""
Maze scanner: 把摄像头拍到的"白纸黑笔迷宫"照片处理成扫描件风格的纯黑白图。

流程:
  1. 检测纸张边界 (find_paper_contour)
  2. 透视矫正        (four_point_transform)
  3. 亮度/对比度增强 (enhance)
  4. 黑白二值化      (binarize)
  5. 输出扫描件      (clean + 保存)

用法:
  python maze_scanner.py input.jpg -o output.png
  python maze_scanner.py input.jpg -o output.png --debug debug_dir
"""

import argparse
import os

import cv2
import numpy as np


# ----------------------------------------------------------------------------
# 工具
# ----------------------------------------------------------------------------
def _resize_to_max(img, max_side=1500):
    """等比缩放，使最长边不超过 max_side。返回 (缩放后的图, 缩放比例)。"""
    h, w = img.shape[:2]
    scale = max_side / float(max(h, w))
    if scale < 1.0:
        img = cv2.resize(img, (int(w * scale), int(h * scale)),
                         interpolation=cv2.INTER_AREA)
        return img, scale
    return img, 1.0


def parse_corners(s):
    """把 "x1,y1 x2,y2 x3,y3 x4,y4" 解析成 [(x,y)x4] (原图坐标)。"""
    if not s:
        return None
    nums = [float(v) for v in s.replace(",", " ").split()]
    if len(nums) != 8:
        raise ValueError("--corners 需要 8 个数字: x1,y1 x2,y2 x3,y3 x4,y4")
    return [(nums[i], nums[i + 1]) for i in range(0, 8, 2)]


def _order_points(pts):
    """把 4 个点排成 [左上, 右上, 右下, 左下]。"""
    pts = pts.reshape(4, 2).astype("float32")
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]   # 左上: x+y 最小
    rect[2] = pts[np.argmax(s)]   # 右下: x+y 最大
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # 右上: x-y 最小
    rect[3] = pts[np.argmax(diff)]  # 左下: x-y 最大
    return rect


# ----------------------------------------------------------------------------
# 步骤 1: 检测纸张边界
# ----------------------------------------------------------------------------
def _quad_from_contour(c, img_shape):
    """凸包 -> 近似四边形; 退化时用最小外接旋转矩形。返回 4x2 角点。"""
    hull = cv2.convexHull(c)
    peri = cv2.arcLength(hull, True)
    for eps in (0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10):
        ap = cv2.approxPolyDP(hull, eps * peri, True)
        if len(ap) == 4 and cv2.isContourConvex(ap):
            return ap.reshape(4, 2)
    return cv2.boxPoints(cv2.minAreaRect(c))


def _paper_by_brightness(img):
    """
    主方法: 纸张是「亮 + 低饱和」的大块区域。
    取包含图像中心的连通块（纸张通常在画面中央，桌面/笔记本/人在四周），
    再求其四边形角点。对低对比度纸张边缘 (白纸/浅木桌) 比边缘检测稳得多。
    """
    H, W = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    S, V = hsv[:, :, 1], hsv[:, :, 2]
    _, bright = cv2.threshold(V, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    mask = ((S < 60) & (bright > 0)).astype(np.uint8) * 255

    # 核大小按分辨率定 (scan 已把最长边缩到 1500)
    k = max(15, int(max(H, W) * 0.014)) | 1
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((k, k), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            np.ones((k // 2 | 1,) * 2, np.uint8))
    # 腐蚀切断纸张与周围浅色物体 (笔记本/浅色衣物) 的细桥
    er = cv2.erode(mask, np.ones((k, k), np.uint8))

    n, lab, st, cen = cv2.connectedComponentsWithStats(er, 8)
    if n <= 1:
        return None
    cy, cx = H // 2, W // 2
    pick = int(lab[cy, cx])                       # 优先取含图像中心的块
    if pick == 0:
        cand = [i for i in range(1, n)
                if st[i, cv2.CC_STAT_AREA] > 0.1 * H * W]
        if not cand:
            return None
        pick = min(cand, key=lambda i: (cen[i][0] - cx) ** 2 + (cen[i][1] - cy) ** 2)
    if st[pick, cv2.CC_STAT_AREA] < 0.15 * H * W:  # 太小不是纸张
        return None

    comp = (lab == pick).astype(np.uint8) * 255
    comp = cv2.dilate(comp, np.ones((k, k), np.uint8))     # 抵消腐蚀
    comp = cv2.morphologyEx(comp, cv2.MORPH_CLOSE,
                            np.ones((k * 2 | 1,) * 2, np.uint8))
    cnts, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    return _quad_from_contour(max(cnts, key=cv2.contourArea), img.shape)


def _paper_by_edges(img):
    """退化方法: Canny 边缘 + 最大四边形轮廓 (纸张边缘对比强时用)。"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 50, 150)
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=2)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    img_area = img.shape[0] * img.shape[1]
    for c in sorted(contours, key=cv2.contourArea, reverse=True)[:5]:
        if cv2.contourArea(c) < 0.2 * img_area:
            continue
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            return approx.reshape(4, 2)
    return None


def find_paper_contour(img):
    """
    找到纸张的 4 个角点，排成 [左上,右上,右下,左下]。
    先用亮度/饱和度分割 (对低对比度边缘稳健)，失败再退化到边缘检测；
    都找不到返回 None（调用方退化为对整张图处理）。
    """
    quad = _paper_by_brightness(img)
    if quad is None:
        quad = _paper_by_edges(img)
    if quad is None:
        return None
    return _order_points(np.asarray(quad, dtype="float32").reshape(4, 2))


# ----------------------------------------------------------------------------
# 步骤 2: 透视矫正
# ----------------------------------------------------------------------------
def four_point_transform(img, rect):
    """根据 4 个角点把纸张拉正成正视图。"""
    (tl, tr, br, bl) = rect
    width_top = np.linalg.norm(tr - tl)
    width_bottom = np.linalg.norm(br - bl)
    height_left = np.linalg.norm(bl - tl)
    height_right = np.linalg.norm(br - tr)
    max_w = int(round(max(width_top, width_bottom)))
    max_h = int(round(max(height_left, height_right)))
    max_w, max_h = max(max_w, 1), max(max_h, 1)

    dst = np.array([[0, 0],
                    [max_w - 1, 0],
                    [max_w - 1, max_h - 1],
                    [0, max_h - 1]], dtype="float32")
    M = cv2.getPerspectiveTransform(rect, dst)
    return cv2.warpPerspective(img, M, (max_w, max_h))


# ----------------------------------------------------------------------------
# 步骤 3: 亮度/对比度增强（去除不均匀光照/阴影）
# ----------------------------------------------------------------------------
def enhance(img):
    """
    估计背景（大尺度模糊），用原图除以背景，去掉阴影和光照不均，
    让纸面变成均匀的白色。返回单通道灰度图。
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img

    # 背景估计: 核要足够大，盖过笔画宽度
    k = max(31, (max(gray.shape) // 20) | 1)  # 保证奇数
    background = cv2.GaussianBlur(gray, (k, k), 0)
    background = np.where(background == 0, 1, background)  # 防止除零

    norm = gray.astype(np.float32) / background.astype(np.float32)
    norm = np.clip(norm, 0, 1)
    norm = (norm * 255).astype(np.uint8)

    # CLAHE 进一步拉对比度
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    return clahe.apply(norm)


# ----------------------------------------------------------------------------
# 步骤 4: 二值化
# ----------------------------------------------------------------------------
def binarize(gray, block_size=35, C=15):
    """
    自适应阈值二值化。输出: 墙壁(黑笔)=黑(0)，纸面=白(255)。
    block_size 必须为奇数。
    """
    if block_size % 2 == 0:
        block_size += 1
    binary = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        block_size, C)
    return binary


# ----------------------------------------------------------------------------
# 步骤 5: 清理噪点
# ----------------------------------------------------------------------------
def clean(binary, min_blob=20):
    """去掉孤立小黑点，让输出更像干净的扫描件。"""
    # 在墙壁(黑)上操作: 反相 -> 墙=白
    inv = cv2.bitwise_not(binary)
    # 开运算去毛刺
    inv = cv2.morphologyEx(inv, cv2.MORPH_OPEN,
                           np.ones((2, 2), np.uint8))
    # 连通域过滤掉太小的斑点
    n, labels, stats, _ = cv2.connectedComponentsWithStats(inv, connectivity=8)
    out = np.zeros_like(inv)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= min_blob:
            out[labels == i] = 255
    return cv2.bitwise_not(out)  # 还原: 墙=黑, 纸=白


# ----------------------------------------------------------------------------
# 手动选角 (默认方式，比自动检测可靠)
# ----------------------------------------------------------------------------
def pick_corners_interactive(img):
    """
    弹窗让用户左键依次点 4 个角。返回 4x2 角点 (未排序) 或 None(取消)。
      左键 = 选点 (最多4个)   u/退格 = 撤销   回车 = 确认   Esc = 取消
    """
    win = "pick 4 corners: L-click x4, [u]undo [Enter]ok [Esc]cancel"
    pts = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append((x, y))

    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, min(img.shape[1], 1000), min(img.shape[0], 1000))
    cv2.setMouseCallback(win, on_mouse)
    try:
        while True:
            disp = img.copy()
            for i, p in enumerate(pts):
                cv2.circle(disp, p, 7, (0, 0, 255), -1)
                cv2.putText(disp, str(i + 1), (p[0] + 10, p[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            if len(pts) >= 2:
                cv2.polylines(disp, [np.array(pts)], len(pts) == 4,
                              (0, 255, 0), 2)
            cv2.imshow(win, disp)
            key = cv2.waitKey(20) & 0xFF
            if key == 27:                       # Esc: 取消
                pts = []
                break
            if key in (ord('u'), 8) and pts:    # u / Backspace: 撤销
                pts.pop()
            if key in (13, 10) and len(pts) == 4:  # Enter: 确认
                break
    finally:
        cv2.destroyWindow(win)
        cv2.waitKey(1)
    return np.array(pts, dtype="float32") if len(pts) == 4 else None


# ----------------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------------
def scan(path, debug_dir=None, margin=0.015, corners=None,
         manual=True, auto=False):
    """
    corners: 原图坐标系下的 4 个角点 [(x,y)x4]，给了就直接用（最高优先级）。
    manual : True 时弹窗手动点选 4 角（默认）。
    auto   : True 时改用自动检测（manual 会被忽略）。
    选角失败/取消时回退到自动检测；自动也失败则处理整张图。
    """
    img = cv2.imread(path)
    if img is None:
        raise FileNotFoundError(f"无法读取图片: {path}")

    img, scale = _resize_to_max(img, 1500)

    def dbg(name, im):
        if debug_dir:
            os.makedirs(debug_dir, exist_ok=True)
            cv2.imwrite(os.path.join(debug_dir, name), im)

    dbg("0_input.png", img)

    # 1: 确定纸张四角 —— 手动选点优先，自动检测作为 fallback
    rect = None
    if corners is not None:
        rect = _order_points(np.asarray(corners, dtype="float32").reshape(4, 2) * scale)
    elif auto:
        rect = find_paper_contour(img)
    elif manual:
        picked = pick_corners_interactive(img)
        if picked is not None:
            rect = _order_points(picked)
        else:
            print("⚠ 未完成手动选角，回退自动检测。")
            rect = find_paper_contour(img)
    else:
        rect = find_paper_contour(img)

    # 2: 透视矫正
    if rect is not None:
        # 角点常略微落在纸外，向中心内缩一点，去掉边缘的桌面/阴影
        center = rect.mean(axis=0)
        rect_in = (center + (rect - center) * (1.0 - margin)).astype("float32")
        warped = four_point_transform(img, rect_in)
        vis = img.copy()
        cv2.polylines(vis, [rect.astype(int)], True, (0, 255, 0), 3)
        cv2.polylines(vis, [rect_in.astype(int)], True, (0, 180, 255), 2)
        dbg("1_paper_detected.png", vis)
    else:
        print("⚠ 未检测到纸张边界，直接处理整张图。")
        warped = img
    dbg("2_warped.png", warped)

    # 3: 增强
    enhanced = enhance(warped)
    dbg("3_enhanced.png", enhanced)

    # 4: 二值化
    binary = binarize(enhanced)
    dbg("4_binary.png", binary)

    # 5: 清理
    result = clean(binary)
    dbg("5_result.png", result)

    # result: 二值扫描件（墙=黑, 纸=白）; warped: 透视矫正后的彩色图（供检测彩色标记）
    return result, warped


def main():
    ap = argparse.ArgumentParser(description="迷宫照片 -> 扫描件风格纯黑白图")
    ap.add_argument("input", help="输入照片路径")
    ap.add_argument("-o", "--output", default="scanned.png", help="输出路径")
    ap.add_argument("--debug", metavar="DIR",
                    help="保存各步骤中间结果到该目录")
    ap.add_argument("--auto", action="store_true",
                    help="用自动检测纸张边界 (默认是手动点选4角)")
    ap.add_argument("--corners", metavar="\"x1,y1 x2,y2 x3,y3 x4,y4\"",
                    help="直接给出原图坐标的4个角点，跳过选点")
    args = ap.parse_args()

    result, _ = scan(args.input, debug_dir=args.debug, auto=args.auto,
                     manual=not args.auto, corners=parse_corners(args.corners))
    cv2.imwrite(args.output, result)
    print(f"✓ 已保存: {args.output}")


if __name__ == "__main__":
    main()
