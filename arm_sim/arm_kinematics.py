"""
5-DOF 机械臂的正/逆运动学 (不依赖 Isaac)。

- FK 依据 urdf/five_dof_arm.urdf 里给出的 origin/axis 链式相乘, 已和 Isaac
  实际 pen_tip 位姿对拍到 0.00mm。
- 笔尖/笔尾取自 link_5.stl 主轴两端点 (PCA 求得)。
- IK: 带关节限位的 SLSQP 优化 (先位置, 再加"笔轴竖直向下"约束)。
"""

import numpy as np
from scipy.optimize import minimize


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
_NIB = np.array([0.0343, 0.0801, -0.0254])         # +Y 端 = 笔尖(写字端)
_TAIL = np.array([0.0369, -0.0399, -0.0290])       # -Y 端 = 笔尾
_JOINT_LO = np.radians([-180.0, -90.0, -90.0, -90.0, -45.0])
_JOINT_HI = np.radians([ 180.0,  90.0,  90.0,  90.0,  45.0])
_BOUNDS = tuple(zip(_JOINT_LO, _JOINT_HI))
_POS_TOL = 5e-5
_CONTINUITY_W = 1e-4


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


def _pen_axis(q):
    nib, tail = fk_nib_tail(q)
    a = nib - tail
    return a / np.linalg.norm(a)


def ik(target, q0, lim=None):
    """带关节限位的两段式 IK: 先位置最优, 再在位置约束下优化笔轴竖直。
    返回 (q, 位置残差, 笔轴偏离竖直角度°)。"""
    del lim  # 旧接口兼容；实际限位固定为真机可动范围。
    target = np.asarray(target, float)
    q_ref = np.clip(np.asarray(q0, float), _JOINT_LO, _JOINT_HI)

    def pos_cost(q):
        err = fk_pos(q) - target
        return float(err @ err)

    pos_res = minimize(
        pos_cost,
        q_ref,
        method="SLSQP",
        bounds=_BOUNDS,
        options={"maxiter": 250, "ftol": 1e-12, "disp": False},
    )
    q_pos = np.clip(pos_res.x if pos_res.x is not None else q_ref,
                    _JOINT_LO, _JOINT_HI)
    pos_err = float(np.linalg.norm(fk_pos(q_pos) - target))

    if pos_err > _POS_TOL:
        a = _pen_axis(q_pos)
        tilt = np.degrees(np.arccos(np.clip(-a[2], -1, 1)))
        return q_pos, pos_err, float(tilt)

    def orient_cost(q):
        a = _pen_axis(q)
        return float((1.0 + a[2]) + _CONTINUITY_W * np.sum((q - q_ref) ** 2))

    orient_res = minimize(
        orient_cost,
        q_pos,
        method="SLSQP",
        bounds=_BOUNDS,
        constraints=({"type": "eq", "fun": lambda q: fk_pos(q) - target},),
        options={"maxiter": 250, "ftol": 1e-12, "disp": False},
    )
    q = np.clip(orient_res.x if orient_res.x is not None else q_pos,
                _JOINT_LO, _JOINT_HI)
    if np.linalg.norm(fk_pos(q) - target) > max(_POS_TOL, pos_err * 2.0):
        q = q_pos

    a = _pen_axis(q)
    tilt = np.degrees(np.arccos(np.clip(-a[2], -1, 1)))
    return q, float(np.linalg.norm(fk_pos(q) - target)), float(tilt)
