"""探测手腕 w 舵机的物理行程：逐步发 w 目标角，读真机实际角度，判断能否转过 ±90。

用途：确认「让笔竖直需要的 w≈-93°」舵机物理上转不转得到，从而决定是放宽 w 限位还是做几何补偿。

⚠️ 前提（必须先做，否则测不出）：
   下位机 robot_server.py 的 LIMITS["w"] 先临时放宽，例如改成 (-100.0, 100.0) 并重启 robot_server。
   否则 w 指令会在下位机被 clamp 到 ±90，机械臂只到 -90，看不出能不能转过去。

⚠️ 安全：
   - spd 设得很小、全程人盯着、手放在 12V 电源开关上。
   - w 顶到机械极限会堵转（听到异响 / 手腕卡住不动 / 舵机发热）→ 立即 Ctrl-C 并断电。
   - 每个角度会暂停等你回车，先肉眼看手腕动到哪、再继续。

    python arm_real_client/probe_w.py
"""

import os
import sys
import time
import math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_client import RobotClient   # noqa: E402

R2D = 180.0 / math.pi


def read_w(r):
    """从 state 返回里读真机当前 w（度）。读不到返回 None。"""
    st = r.state()
    raw = st.get("raw") or {}
    if not isinstance(raw, dict) or "w" not in raw:
        return None
    return float(raw["w"]) * R2D


if __name__ == "__main__":
    r = RobotClient()
    print("ping:", r.ping(), flush=True)
    print("（其他关节保持 0，只逐步转 w；spd=5 很慢）", flush=True)

    # 从 -85° 起，每次 -2°，一直试到 -99°，看从哪开始转不动
    targets = list(range(-85, -100, -2))   # -85,-87,...,-99
    for w in targets:
        print(f"\n>>> 发 w={w}°  (b/s/e/h=0, spd=5)", flush=True)
        r.joint(b=0, s=0, e=0, w=float(w), h=0, spd=5, acc=5)
        time.sleep(2.5)                     # 等它转到位
        actual = read_w(r)
        if actual is None:
            print("   ⚠ 读不到真机 w（T=105 无返回？检查串口/数据线）", flush=True)
        else:
            stuck = abs(actual - w) > 3.0
            tag = "⚠ 没跟上/可能到极限了" if stuck else "正常跟随"
            print(f"   指令 {w}°  ->  实际 ≈{actual:.1f}°   {tag}", flush=True)
        try:
            input("   肉眼看手腕，回车测下一个角度（异常请 Ctrl-C 并断电）... ")
        except KeyboardInterrupt:
            print("\n已中断。", flush=True)
            break

    print("\n探测结束：从「指令到实际不再跟随」的那个角度，就是 w 的物理极限。", flush=True)
    print("若一路到 -99° 都还跟得上 → w 能转过 ±90，可放宽限位让笔竖直。", flush=True)
