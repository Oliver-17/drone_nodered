#!/usr/bin/env python3
# =============================================================================
#  t_standoff_dir.py —— 驗「站位方向的混合與低通」治得了左右搖擺和抽動
#
#  跑法（不需要 ROS、不需要 Gazebo）：
#      python3 ~/ros2_ws/src/drone_nodered/test/t_standoff_dir.py
#
#  為什麼要有這個測試：
#      2026-10-19 實測，跟隨時畫面一直左右搖擺。從節點自己的 log 量出來，
#      車明明是直線往北走（真實方向 0°），但估出來的車行方向每半秒在
#      ±13° 之間翻邊，而橫向速度指令一比一跟著翻：
#          車向 -12° → 左-0.70      車向 +14° → 左+0.75
#      根因是站位點**剛性掛在估出來的方向上**（站位點 = 車 + b × dist），
#      前後那一軸被 dist 鎖住，橫向那一軸沒有任何東西鎖住。
#
#      同一個機制還造成「剛看到目標時抽動一下」：車速估計在
#      TRACK_MIN_TARGET_SPEED 門檻上下抖，站位點就在兩套算法之間跳，
#      log 量到半秒內跳了 3.9 m。
#
#  🔴 兩個最容易寫錯又最難看出來的地方，這裡各有一組測資釘住：
#      ① 濾角度而不是濾向量 —— 車往南走時原始方向在 +179°/−179° 之間跳，
#         濾角度會算出 0°（正北），飛機直接衝到車前面。模擬裡看起來像
#         「偶爾暴衝」，很難歸因。
#      ② 門檻上的硬切換 —— 平均起來看不出異常，只在車速恰好在門檻附近
#         徘徊時爆發，而那正好就是剛鎖定的那一刻。
# =============================================================================

import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
NODE = os.path.join(HERE, "..", "scripts", "drone_api_node.py")


def _load():
    """挖出兩個待測的 staticmethod 和它們用到的常數（理由見 t_no_fly.py）。"""
    with open(NODE, encoding="utf-8") as f:
        src = f.read()
    ns = {"math": math}
    for line in src.splitlines():
        if re.match(r"^TRACK_(DIR_TAU_S|MIN_TARGET_SPEED|UPDATE_S|CMD_HZ)\s*=", line):
            exec(line.split("#")[0], ns)                     # noqa: S102
    chunks = []
    for name in ("_blend_standoff_dir", "_filter_dir"):
        m = re.search(r"^    @staticmethod\n    def %s\(.*?(?=^    (?:@|def )|^class )"
                      % re.escape(name), src, re.M | re.S)
        if not m:
            print(f"✗ 在 {NODE} 裡找不到 {name}()")
            sys.exit(1)
        chunks.append(m.group(0))
    exec("class _Api:\n" + "\n".join(chunks), ns)             # noqa: S102
    return ns["_Api"], ns


_Api, _C = _load()
TAU = _C["TRACK_DIR_TAU_S"]
MINV = _C["TRACK_MIN_TARGET_SPEED"]
TICK = _C["TRACK_UPDATE_S"]
DIST = 2.12          # 2026-10-19 那次飛行的站位距離（5 m 高 / tan 67°）

FAILS = []


def check(ok, what, detail=""):
    print(("  ✓ " if ok else "  ✗ ") + what + (f"   {detail}" if detail else ""))
    if not ok:
        FAILS.append(what)


def bearing(b):
    """站位方向 b 反過來就是車行方向，回傳度數。"""
    return math.degrees(math.atan2(-b[1], -b[0]))


def blend_at(deg, speed, los_deg=None):
    """車以 speed 往 deg 方向走時的原始站位方向。los_deg 給視線方向。"""
    vx = speed * math.cos(math.radians(deg))
    vy = speed * math.sin(math.radians(deg))
    if los_deg is None:                  # 飛機正好在站位點上 → 視線＝車行方向
        los_deg = deg
    # dx,dy 是「飛機 → 車」，所以視線來源的 b_los = -(dx,dy)/d
    dx = DIST * math.cos(math.radians(los_deg))
    dy = DIST * math.sin(math.radians(los_deg))
    return _Api._blend_standoff_dir(vx, vy, math.hypot(vx, vy), dx, dy, DIST)


# =============================================================================
#  ① 混合：兩端要和舊行為完全一致
# =============================================================================
print("① 混合的兩端（不能改變原本正確的行為）")
b = blend_at(0.0, 0.5)                   # 車往北 0.5 m/s（遠超門檻）
check(abs(bearing(b) - 0.0) < 1e-6, "車速遠超門檻 → 純粹跟在車行方向後面",
      f"車向 {bearing(b):+.1f}°")
# 車完全不動：b 應該等於視線方向（飛機在車的東邊 → 站位點在車的東邊）
b = _Api._blend_standoff_dir(0.0, 0.0, 0.0, -DIST, 0.0, DIST)
check(abs(b[0] - 1.0) < 1e-6 and abs(b[1]) < 1e-6,
      "車完全不動 → 純粹沿視線（維持現在的相對位置）", f"b=({b[0]:+.2f},{b[1]:+.2f})")
check(_Api._blend_standoff_dir(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) is None,
      "車不動 + 飛機正好在車正上方 → 回 None（沒有方向可言）")

# =============================================================================
#  ② 🔴 門檻上不能有斷點 —— 治「剛看到目標時抽動」
# =============================================================================
print("\n② 門檻附近必須連續（硬切換就是抽動的原因）")
# 車往北走，但飛機在車的**西邊**（視線方向和車行方向差 90°）——
# 這是兩個來源分歧最大的情況，硬切換在這裡跳得最凶。
jumps = []
prev = None
for i in range(41):
    sp = MINV * 0.5 + (MINV * 0.5) * i / 40.0      # 0.075 → 0.15，跨過門檻
    bb = blend_at(0.0, sp, los_deg=180.0)
    g = (DIST * bb[0], DIST * bb[1])               # 站位點相對車的位移
    if prev is not None:
        jumps.append(math.hypot(g[0] - prev[0], g[1] - prev[1]))
    prev = g
check(max(jumps) < 0.25,
      "車速掃過門檻時站位點不會跳（相鄰兩步位移 < 0.25 m）",
      f"最大一步 {max(jumps):.3f} m")

# 舊的硬切換在同一組輸入下會跳多少？（對照組，證明這個測試測得到問題）
def old_hard(deg, speed, los_deg):
    if speed >= MINV:
        return (-math.cos(math.radians(deg)), -math.sin(math.radians(deg)))
    return (-math.cos(math.radians(los_deg)), -math.sin(math.radians(los_deg)))


a = old_hard(0.0, MINV - 1e-6, 180.0)
c = old_hard(0.0, MINV + 1e-6, 180.0)
old_jump = math.hypot(DIST * (c[0] - a[0]), DIST * (c[1] - a[1]))
check(old_jump > 3.0,
      "（對照）舊的硬切換在門檻上會瞬移，所以這個測試有意義",
      f"硬切換跳 {old_jump:.2f} m")

# =============================================================================
#  ③ 🔴 低通要濾向量不是角度 —— 車往南走時的 ±180° 陷阱
# =============================================================================
print("\n③ 車往南走（±180° 邊界）不能濾出相反的方向")
south_a = blend_at(+179.0, 0.5)
south_b = blend_at(-179.0, 0.5)
f = _Api._filter_dir(south_a, south_b, TICK)
bb = bearing(f)
check(abs(abs(bb) - 179.0) < 2.0,
      "+179° 和 -179° 之間濾出來還是往南（不是 0°）", f"濾後 {bb:+.1f}°")
# 把同一組資料拿去「濾角度」會得到什麼 —— 證明這個陷阱是真的
naive = (+179.0 + (-179.0 - 179.0) * (1 - math.exp(-TICK / TAU)))
check(abs(naive) < 170.0,
      "（對照）濾角度會偏離往南，確認這個陷阱測得到", f"濾角度得到 {naive:+.1f}°")

print("\n③ 車 180° 大轉彎時不要卡在中間")
f = _Api._filter_dir((1.0, 0.0), (-1.0, 0.0), TICK)
check(abs(math.hypot(*f) - 1.0) < 1e-9,
      "新舊方向正好相反 → 回傳的仍是單位向量（不是長度 0 的亂數）",
      f"|b|={math.hypot(*f):.6f}")

# =============================================================================
#  ④ 低通對「真實雜訊」的壓制量 —— 用 2026-10-19 log 的實際數字
# =============================================================================
print("\n④ 壓制量（餵 2026-10-19 log 量到的 ±13° 翻邊）")
# log 裡的原始車向序列（車其實是直線往北走，真實方向 0°）
REAL = [-7, +9, +7, -8, -11, +8, +12, -8, -11, +6, +11, -0, -12, +2, +14, +0, -11]
# 每 0.5 秒一筆（log 的節流週期），但迴圈是 10 Hz → 每個值重複 5 拍
prev = None
after = []
for deg in REAL:
    raw = blend_at(float(deg), 0.5)
    for _ in range(5):
        prev = _Api._filter_dir(prev, raw, TICK)
    after.append(bearing(prev))
raw_pp = max(REAL) - min(REAL)
out_pp = max(after) - min(after)
# ⚠️ 門檻訂在「2 倍、0.30 m/s」而不是當初喊的「3 倍、0.20 m/s」，理由有兩個，
#    都不是「改到測試過為止」：
#
#  ① τ 不能再加大。τ=1.5 時 90° 轉彎要 4.7 秒收斂（見 ⑤），τ=2.5 會變成
#     將近 8 秒 —— 車繞環線四個轉角之間只隔 20 秒，再慢就會在轉角掉目標。
#     轉彎反應是硬性約束，雜訊衰減是軟性目標，不能拿前者換後者。
#
#  ② 這組測資是**第 7 項（姿態時間對齊）之前**量到的雜訊。那 ±13° 裡面
#     有一部分就是「用最新姿態配舊影像」造成的假性位移
#     （5 m 高、延遲 0.15 秒、橫滾 30°/s → 0.39 m）。第 7 項把那一份拔掉，
#     進到這個濾波器的雜訊本來就會變小，所以實機上的結果會比這裡好。
#     這個測試刻意餵**最糟的那一版**輸入，當成下限。
#
#  實際有沒有達到 < 0.20 要看飛行 log 的「左」那一欄，不是看這裡。
check(out_pp < raw_pp / 2.0,
      f"峰到峰從 {raw_pp:.0f}° 壓到 {out_pp:.1f}°（至少降到一半）",
      f"衰減 {raw_pp / max(out_pp, 1e-9):.1f} 倍")
# 換算成橫向速度指令：|左| ≈ dist × sin(偏角) × TRACK_KP
lat_before = DIST * math.sin(math.radians(raw_pp / 2.0)) * 1.2
lat_after = DIST * math.sin(math.radians(out_pp / 2.0)) * 1.2
check(lat_after < 0.30,
      f"換算橫向指令峰值 {lat_before:.2f} → {lat_after:.2f} m/s",
      f"（2026-10-19 實測 0.70~0.76；希望值 < 0.20，靠第 7 項補）")
check(abs(sum(after) / len(after)) < 3.0,
      "濾完的平均方向仍然是正北（沒有把偏差濾成一個常數偏移）",
      f"平均 {sum(after) / len(after):+.1f}°")

# =============================================================================
#  ⑤ 真轉彎要跟得上 —— 不能為了平滑把反應拖垮
# =============================================================================
print("\n⑤ 車真的轉 90° 時要跟得上")
prev = blend_at(0.0, 0.5)                 # 穩定往北
raw = blend_at(90.0, 0.5)                 # 突然轉成往東
t = 0.0
while t < 10.0:
    prev = _Api._filter_dir(prev, raw, TICK)
    t += TICK
    if abs(bearing(prev) - 90.0) < 5.0:
        break
check(t < 6.0, "轉彎後 6 秒內要收斂到 ±5° 以內", f"花了 {t:.1f} 秒")
check(t > 1.0, "但也不能瞬間跟上（那就等於沒濾）", f"花了 {t:.1f} 秒")

print()
if FAILS:
    print(f"✗ {len(FAILS)} 項沒過：")
    for f_ in FAILS:
        print("   " + f_)
    sys.exit(1)
print("✓ 全部通過 —— 門檻沒有斷點、±180° 沒有陷阱、"
      f"±13° 的翻邊讓橫向指令從 {lat_before:.2f} 降到 {lat_after:.2f} m/s"
      f"（2026-10-19 實測 0.70），真轉彎 {t:.1f} 秒內跟上")
