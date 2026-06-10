"""
5-DOF 机械臂的正/逆运动学 (纯 numpy, 不依赖 Isaac)。

- FK 依据 urdf/five_dof_arm.urdf 里给出的 origin/axis 链式相乘, 已和 Isaac
  实际 pen_tip 位姿对拍到 0.00mm。
- 笔尖/笔尾取自 link_5.stl 主轴两端点 (PCA 求得)。
- IK: 两段式阻尼最小二乘 (先位置, 再加"笔轴竖直向下"约束)。
"""

import numpy as np


def _T(x, y, z):
    M = np.eye(4); M[:3, 3] = (x, y, z); return M

def _Rx(a):
    c, s = np.cos(a), np.sin(a); M = np.eye(4)
    M[1, 1], M[1, 2], M[2, 1], M[2, 2] = c, -s, s, c; return M

def _Ry(a):
    c, s = np.cos(a), np.sin(a); M = np.eye(4)
    M[0, 0], M[0, 2], M[2, 0], M[2, 2] = c, s, -s, c; return M

def _Rz(a):
    c, s = np.cos(a), np.sin(a); M = np.eye(4)
    M[0, 0], M[0, 1], M[1, 0], M[1, 1] = c, -s, s, c; return M

def _rpy(r, p, y):                          # URDF: R = Rz(y) Ry(p) Rx(r)
    return _Rz(y) @ _Ry(p) @ _Rx(r)

_PI = np.pi
# 每个关节: trans(origin) @ rpy(origin) @ Rz(sign*theta); sign 来自 axis 的 z 符号
_JOINTS = [
    (_T(0, 0, 0.0493),                       np.eye(4),                 -1),  # j1 axis(0,0,-1)
    (_T(0, 0, 0.0335),                       _rpy(_PI/2, 0, 0),         -1),  # j2 axis(0,0,-1)
    (_T(0.03564, 0.23002, 0),                np.eye(4),                 -1),  # j3 axis(0,0,-1)
    (_T(0.24164, 0, 0),                      _rpy(0, -_PI/2, _PI),      +1),  # j4 axis(0,0, 1)
    (_T(0.0121, -0.02478, 0.01835),          _rpy(0, -_PI/2, _PI/2),    +1),  # j5 axis(0,0, 1)
]
_PEN_T = np.array([0.03431, -0.08012, -0.02672])   # URDF pen_tip 偏移 (只用于和 Isaac 对拍)
_NIB = np.array([0.0343, 0.0730, 0.03033])         # +Y 端 = 笔尖(写字端)
_TAIL = np.array([0.0369, -0.0399, 0.03033])       # -Y 端 = 笔尾
_A_DES = np.array([0.0, 0.0, -1.0])


def fk_M5(q):
    """link_5 坐标系到 world 的 4x4 (base 固定在原点)。"""
    M = np.eye(4)
    for (tr, rot, sign), th in zip(_JOINTS, q):
        M = M @ tr @ rot @ _Rz(sign * th)
    return M


def fk_nib_tail(q):
    """返回 (笔尖 nib, 笔尾 tail) 的世界坐标。"""
    M5 = fk_M5(q)
    o, R = M5[:3, 3], M5[:3, :3]
    return o + R @ _NIB, o + R @ _TAIL


def fk_true_pen(q):
    """URDF 原始 pen_tip 世界坐标 (用于和 Isaac 对拍校验 FK)。"""
    M5 = fk_M5(q)
    return M5[:3, 3] + M5[:3, :3] @ _PEN_T


def fk_pos(q):
    return fk_nib_tail(q)[0]


def _out_posdir(q, w_o):
    """输出向量: [笔尖位置(3), w_o*笔轴单位向量(3)]。"""
    nib, tail = fk_nib_tail(q)
    a = nib - tail
    a /= np.linalg.norm(a)
    return np.concatenate([nib, w_o * a])


def _dls(q, out_fn, goal, ndim, lam=0.05, lim=_PI):
    """一步阻尼最小二乘: J=d(out)/dq, dq = J^T (JJ^T+λ²)^-1 (goal-out)。"""
    base = out_fn(q)
    r = goal - base
    J = np.zeros((ndim, 5))
    for i in range(5):
        dq = np.zeros(5); dq[i] = 1e-5
        J[:, i] = (out_fn(q + dq) - base) / 1e-5
    dtheta = J.T @ np.linalg.solve(J @ J.T + (lam**2) * np.eye(ndim), r)
    return np.clip(q + np.clip(dtheta, -0.2, 0.2), -lim, lim)


def ik(target, q0, lim=_PI):
    """两段式: 先纯位置收敛到目标, 再加笔轴竖直向下约束。
    返回 (q, 位置残差, 笔轴偏离竖直角度°)。"""
    q = np.array(q0, float)
    for _ in range(250):                          # 阶段1: 仅位置 (3D)
        if np.linalg.norm(target - fk_pos(q)) < 5e-5:
            break
        q = _dls(q, fk_pos, target, 3, lim=lim)
    w_o = 0.05                                    # 阶段2: 位置 + 笔轴竖直 (6D)
    goal = np.concatenate([target, w_o * _A_DES])
    for _ in range(250):
        out = _out_posdir(q, w_o)
        if np.linalg.norm(out[:3] - target) < 5e-5 and \
           np.linalg.norm(out[3:] / w_o - _A_DES) < 1e-2:
            break
        q = _dls(q, lambda qq: _out_posdir(qq, w_o), goal, 6, lim=lim)
    nib, tail = fk_nib_tail(q)
    a = (nib - tail) / np.linalg.norm(nib - tail)
    tilt = np.degrees(np.arccos(np.clip(-a[2], -1, 1)))
    return q, float(np.linalg.norm(target - nib)), float(tilt)
