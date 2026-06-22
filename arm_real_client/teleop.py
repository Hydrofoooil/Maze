"""键盘遥操：方向键 + QEAD 控制笔尖在纸面上移动（贴纸面画线）。

设计：脚本在内存里维护「当前笔尖目标 (x,y,z)」。每按一次键，把目标在水平面上挪一个
固定步长，对新目标解一次 IK（保持笔竖直）得到 5 个关节角，通过 robot_server 让机械臂走过去。
z 固定贴纸面 → 移动即画线。底层限幅/串口都交给已运行的 robot_server，本脚本只「读键→IK→发点」。

按键：
  ↑ 前(x+，远离底座)   ↓ 后(x-)   ← 左   → 右            正交四向，每次走一步
  Q 左前   E 右前   A 左后   D 右后                       45° 斜走（对角线位移同样一步）
  + / -  增大/减小步长          Esc 或 Ctrl-C  退出
  （若实测左右/前后整体方向反了，启动时加 --invert-y / --invert-x；到工作空间边界会忽略不发）

坐标：x=底座正前方(前向)，y=横向(左=y-、右=y+)，z=竖直。单位米；步长按毫米显示。

部署（在下位机 Windows 上、与 robot_server 同机运行）：
  1. 把 arm_sim/arm_kinematics.py 拷到本目录（或 arm_sim/ 下），并 pip install numpy。
  2. 确保 robot_server.py 已在运行（监听 9001）。
  3. python teleop.py                 # 默认连 127.0.0.1:9001
  先自检（不连机械臂、不读键盘，验证 IK/坐标/限位是否正常）：
     python teleop.py --selftest
"""

import os
import sys
import argparse

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
for _p in (HERE, os.path.join(REPO, "arm_sim")):     # arm_kinematics 在 arm_sim/，Windows 部署时拷到本目录
    if _p not in sys.path:
        sys.path.insert(0, _p)

from arm_kinematics import ik              # noqa: E402  纯 numpy IK
from robot_client import RobotClient       # noqa: E402
try:
    from robot_config import ROBOT_PORT    # noqa: E402  端口取自唯一真源
except Exception:
    ROBOT_PORT = 9001

# 纸面与笔高（与 plan_maze 一致）：纸面上表面 0.002m，笔尖贴纸面 +1mm
PEN_Z_DEFAULT = 0.003
JOINT_ORDER = ["b", "s", "e", "w", "h"]
# 关节限位（度）。超 b/s/e = 超工作空间 → 拒发；w/h 交给下位机 clamp（w 画竖直笔必然约 -93°）
LIMITS_DEG = {"b": (-180, 180), "s": (-90, 90), "e": (-90, 90),
              "w": (-90, 90), "h": (-180, 180)}
HARD_KEYS = ("b", "s", "e")
Q_SEED0 = np.array([0.0, 1.0, 1.0, 0.0, 0.0])   # 朝前下方折叠的初始猜测
RES_TOL = 0.002                                 # 位置残差 > 2mm 视为解不出/超工作空间

# 每个按键 -> (x方向系数, y方向系数)，单位为 step。
# 正交键单轴 ±1；斜走键双轴 ±1/√2，使「对角线总位移 = step」，和直走一步距离一致。
# x=前向(+前/-后)，y=横向(+右/-左，左右已按实机修正)。
_D = 1.0 / (2 ** 0.5)
KEY_MOVE = {
    "up":    (+1.0,  0.0),   # 前
    "down":  (-1.0,  0.0),   # 后
    "left":  (0.0,  -1.0),   # 左
    "right": (0.0,  +1.0),   # 右
    "Q":     (+_D,  -_D),    # 左前
    "E":     (+_D,  +_D),    # 右前
    "A":     (-_D,  -_D),    # 左后
    "D":     (-_D,  +_D),    # 右后
}


def solve(target, q_seed):
    """对目标点解 IK，返回 (q, deg, res, tilt, ok, bad)。
    ok=可安全下发；bad=超工作空间的关节名列表。"""
    q, res, tilt = ik(np.asarray(target, float), q_seed)
    deg = np.degrees(q)
    bad = [k for i, k in enumerate(JOINT_ORDER)
           if k in HARD_KEYS and not (LIMITS_DEG[k][0] <= deg[i] <= LIMITS_DEG[k][1])]
    ok = (res < RES_TOL) and (not bad)
    return q, deg, res, tilt, ok, bad


def deg_to_joint(deg):
    return {k: float(deg[i]) for i, k in enumerate(JOINT_ORDER)}


def make_key_reader():
    """返回一个 read_key()：单次读一个按键，归一成
    up/down/left/right（正交）、Q/E/A/D（斜走）、step+/step-、quit、None。
    Windows 用 msvcrt（读方向键的双字节序列）；其它平台用 termios 原始模式读 ESC 序列。
    退出只认 Esc / Ctrl-C（q 已让给「左前」）。q/e/a/d 不分大小写。"""
    def _letter(ch_lower):
        return {"q": "Q", "e": "E", "a": "A", "d": "D"}.get(ch_lower)

    try:
        import msvcrt

        def read_key():
            ch = msvcrt.getch()
            if ch in (b"\x00", b"\xe0"):                 # 方向键前缀
                ch2 = msvcrt.getch()
                return {b"H": "up", b"P": "down", b"K": "left", b"M": "right"}.get(ch2)
            if ch == b"\x1b":
                return "quit"
            if ch in (b"+", b"="):
                return "step+"
            if ch in (b"-", b"_"):
                return "step-"
            try:
                return _letter(ch.decode("ascii").lower())
            except Exception:
                return None
        return read_key
    except ImportError:
        import termios
        import tty
        import select

        def read_key():
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                ch = sys.stdin.read(1)
                if ch == "\x1b":                          # ESC：方向键(ESC[A..) 或单独退出
                    if select.select([sys.stdin], [], [], 0.0005)[0]:
                        seq = sys.stdin.read(2)
                        return {"[A": "up", "[B": "down", "[D": "left", "[C": "right"}.get(seq)
                    return "quit"
                if ch in ("+", "="):
                    return "step+"
                if ch in ("-", "_"):
                    return "step-"
                return _letter(ch.lower())
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return read_key


def run_selftest(args):
    """不连机械臂、不读键盘：从起点向 8 个方向各模拟走 5 步，打印 IK/限位结果。"""
    print("[selftest] 仅验证 IK + 坐标 + 限位，不连机械臂、不读键盘\n", flush=True)
    target = [args.paper_cx, args.paper_cy, args.z]
    q, deg, res, tilt, ok, bad = solve(target, Q_SEED0)
    print(f"起点 x={target[0]*1000:.0f} y={target[1]*1000:.0f} z={target[2]*1000:.0f}mm  "
          f"ok={ok} res={res*1000:.2f}mm tilt={tilt:.1f}°  "
          f"deg(b,s,e,w,h)={np.round(deg,1).tolist()}", flush=True)
    sx = -1 if args.invert_x else 1
    sy = -1 if args.invert_y else 1
    demo = [("前 ↑", "up"), ("后 ↓", "down"), ("左 ←", "left"), ("右 →", "right"),
            ("左前 Q", "Q"), ("右前 E", "E"), ("左后 A", "A"), ("右后 D", "D")]
    for name, key in demo:
        fx, fy = KEY_MOVE[key]
        t = list(target)
        q_s = q.copy()
        print(f"\n-- {name} 连走 5 步 (step={args.step*1000:.0f}mm) --", flush=True)
        for i in range(5):
            t[0] += fx * args.step * sx
            t[1] += fy * args.step * sy
            q2, deg2, res2, tilt2, ok2, bad2 = solve(t, q_s)
            tag = "OK" if ok2 else (f"残差{res2*1000:.1f}mm" if res2 >= RES_TOL
                                    else f"{','.join(bad2)}超限")
            print(f"  步{i+1} x={t[0]*1000:6.1f} y={t[1]*1000:6.1f}mm | res={res2*1000:5.2f}mm "
                  f"tilt={tilt2:4.1f}° {tag:8s} | b={deg2[0]:6.1f} s={deg2[1]:6.1f} "
                  f"e={deg2[2]:6.1f} w={deg2[3]:6.1f}", flush=True)
            if ok2:
                q_s = q2
    print("\n[selftest] 完成。若 8 个方向都能走出来、关节角连续，逻辑即正常。", flush=True)


HELP = ("方向键 ↑前 ↓后 ←左 →右 | Q左前 E右前 A左后 D右后(45°斜走) | "
        "+/- 调步长 | Esc/Ctrl-C 退出")


def main():
    ap = argparse.ArgumentParser(description="键盘遥操：方向键+QEAD 控制笔尖在纸面移动")
    ap.add_argument("--host", default="127.0.0.1", help="robot_server 地址（默认本机 127.0.0.1）")
    ap.add_argument("--port", type=int, default=ROBOT_PORT, help="robot_server 端口")
    ap.add_argument("--paper-cx", type=float, default=0.33, help="起点：底座前向距离(m)")
    ap.add_argument("--paper-cy", type=float, default=0.0, help="起点：横向偏移(m)")
    ap.add_argument("--z", type=float, default=PEN_Z_DEFAULT, help="笔尖高度(m)，默认贴纸面 0.003")
    ap.add_argument("--step", type=float, default=0.005, help="每步移动距离(m)，默认 5mm")
    ap.add_argument("--spd", type=int, default=5, help="关节角速度(°/s)，遥操建议小")
    ap.add_argument("--acc", type=int, default=10, help="关节角加速度")
    ap.add_argument("--invert-x", action="store_true", help="前后方向反了时加")
    ap.add_argument("--invert-y", action="store_true", help="左右方向反了时加")
    ap.add_argument("--selftest", action="store_true", help="只验证 IK/坐标/限位，不连机械臂")
    args = ap.parse_args()

    if args.selftest:
        run_selftest(args)
        return

    target = [args.paper_cx, args.paper_cy, args.z]
    q, deg, res, tilt, ok, bad = solve(target, Q_SEED0)
    print(f"[起点] x={target[0]*1000:.0f} y={target[1]*1000:.0f} z={target[2]*1000:.0f}mm  "
          f"res={res*1000:.2f}mm tilt={tilt:.1f}°", flush=True)
    if not ok:
        print(f"[警告] 起点不可达（{','.join(bad) or '残差大'}），换 --paper-cx/--paper-cy 再试。", flush=True)
        return
    print(f"[起点] 关节角 b={deg[0]:.1f} s={deg[1]:.1f} e={deg[2]:.1f} w={deg[3]:.1f} h={deg[4]:.1f}",
          flush=True)
    try:
        input("回车确认：机械臂将移动到该起点（贴纸面，会落笔）。Ctrl-C 取消... ")
    except KeyboardInterrupt:
        print("\n已取消。", flush=True)
        return

    robot = RobotClient(host=args.host, port=args.port)
    print("[conn] ping:", robot.ping(), flush=True)
    robot.joint(spd=args.spd, acc=args.acc, **deg_to_joint(deg))

    read_key = make_key_reader()
    step = args.step
    sx = -1 if args.invert_x else 1
    sy = -1 if args.invert_y else 1
    print("\n" + HELP + "\n", flush=True)
    try:
        while True:
            key = read_key()
            if key == "quit":
                break
            if key == "step+":
                step = min(step + 0.001, 0.02)
                print(f"  步长 = {step*1000:.0f}mm", flush=True)
                continue
            if key == "step-":
                step = max(step - 0.001, 0.001)
                print(f"  步长 = {step*1000:.0f}mm", flush=True)
                continue
            if key not in KEY_MOVE:
                continue
            fx, fy = KEY_MOVE[key]
            nt = list(target)
            nt[0] += fx * step * sx
            nt[1] += fy * step * sy
            q2, deg2, res2, tilt2, ok2, bad2 = solve(nt, q)
            if not ok2:
                reason = f"残差{res2*1000:.1f}mm" if res2 >= RES_TOL else f"{','.join(bad2)}超限"
                print(f"  ✗ 到边界（{reason}），忽略", flush=True)
                continue
            target, q = nt, q2
            robot.joint(spd=args.spd, acc=args.acc, **deg_to_joint(deg2))
            print(f"  → x={target[0]*1000:6.1f} y={target[1]*1000:6.1f}mm | "
                  f"b={deg2[0]:6.1f} s={deg2[1]:6.1f} e={deg2[2]:6.1f} w={deg2[3]:6.1f} "
                  f"h={deg2[4]:6.1f} | res={res2*1000:.2f}mm tilt={tilt2:.1f}°", flush=True)
    except KeyboardInterrupt:
        pass
    print("\n退出遥操。", flush=True)


if __name__ == "__main__":
    main()
