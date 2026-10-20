#!/usr/bin/env python3
# =============================================================================
#  t_coords.py —— 釘住 map ↔ PX4 本地 NED 的座標轉換
#
#  跑法（不需要 ROS、不需要 Gazebo）：
#      python3 ~/ros2_ws/src/drone_nodered/test/t_coords.py
#
#  為什麼要有這個測試：
#      座標轉換是這個專案踩過最多次的雷，而它壞掉的方式特別惡劣 ——
#      **不會報錯**。搜尋航點整組平移、禁飛方框對不上貨架、儀表板把無人機
#      畫在牆裡，全都照常執行、照常回報成功。
#
#      2026-10-20 新增 _ned_to_map（儀表板要畫視覺目標的位置）時，
#      local_origin_map 的解析從兩份變成三份 —— 那是「有一天只改到兩份」
#      的標準起點。所以把解析提取成 _origin_map()，並用這支測試把
#      「三個轉換永遠互相一致」釘住。
#
#  🔴 最該被釘住的一條：_ned_to_map 和 _local_to_map 對**同一個點**必須
#     給出一樣的答案。前者是通用版（目標、航點），後者是飛機自己的版本，
#     兩個各寫一次加法就是最容易出現反號的地方。
# =============================================================================

import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
NODE = os.path.join(HERE, "..", "scripts", "drone_api_node.py")


def _load():
    """挖出四個座標轉換的 method，組成一個不需要 ROS 的假節點類別。

    用正規表示式挖原始碼而不是 import：節點 import 了 rclpy 和 px4_msgs，
    在沒有 source 過工作區的環境（例如 CI、或剛開的終端機）會直接失敗。
    這和 t_no_fly.py、t_standoff_dir.py 是同一套手法。
    """
    with open(NODE, encoding="utf-8") as f:
        src = f.read()
    chunks = []
    for name in ("_origin_map", "_ned_to_map", "_map_to_local", "_local_to_map"):
        m = re.search(r"^    def %s\(self.*?(?=^    (?:@|def )|^class )"
                      % re.escape(name), src, re.M | re.S)
        if not m:
            print(f"✗ 在 {NODE} 裡找不到 {name}()")
            sys.exit(1)
        chunks.append(m.group(0))
    ns = {"math": math}
    exec("class _Api:\n    pass\n", ns)                      # noqa: S102
    exec("class _Api:\n" + "\n".join(chunks), ns)            # noqa: S102
    return ns["_Api"]


_Api = _load()
FAILS = []


def check(ok, what, detail=""):
    print(("  ✓ " if ok else "  ✗ ") + what + (f"   {detail}" if detail else ""))
    if not ok:
        FAILS.append(what)


class _Lpos:
    """只有座標轉換會讀到的那幾個欄位。"""

    def __init__(self, x, y, heading=0.0, xy_valid=True):
        self.x, self.y, self.heading, self.xy_valid = x, y, heading, xy_valid


def api(origin="-14.0,-9.0", lpos=None):
    a = _Api()
    a.local_origin_map = origin
    a.lpos = lpos
    return a


# 倉庫場景的實際值：生成點在 map (-14, -9)
SPAWN = (-14.0, -9.0)
ORIGIN = f"{SPAWN[0]},{SPAWN[1]}"

# =============================================================================
#  ① 已知的對照值（2026-10-08 用倉庫的搜尋環線手算對照過的那組）
# =============================================================================
print("① 已知對照值：倉庫的搜尋環線")
a = api()
for mx, my, nx, ny in [(5.6, -9.0, 0.0, 19.6),
                       (5.6, 9.0, 18.0, 19.6),
                       (0.0, 9.0, 18.0, 14.0),
                       (0.0, -9.0, 0.0, 14.0),
                       (-14.0, -9.0, 0.0, 0.0)]:      # 生成點 → 本地原點
    got = a._map_to_local(mx, my)
    check(got is not None and abs(got[0] - nx) < 1e-9 and abs(got[1] - ny) < 1e-9,
          f"map({mx:+.1f},{my:+.1f}) → 北{nx:+.1f} 東{ny:+.1f}",
          f"得到 北{got[0]:+.1f} 東{got[1]:+.1f}" if got else "得到 None")

# =============================================================================
#  ② 雙向一致：map → NED → map 必須回到原處
# =============================================================================
print("② 雙向一致（_map_to_local 和 _ned_to_map 互為反函數）")
for mx, my in [(0.0, 0.0), (5.6, -9.0), (-17.9, 11.9), (8.4, 7.0), (-2.8, -0.5)]:
    ned = a._map_to_local(mx, my)
    back = a._ned_to_map(ned[0], ned[1])
    check(abs(back[0] - mx) < 1e-9 and abs(back[1] - my) < 1e-9,
          f"map({mx:+.1f},{my:+.1f}) 轉過去再轉回來",
          f"回到 ({back[0]:+.2f},{back[1]:+.2f})")

# =============================================================================
#  ③ 🔴 _ned_to_map 和 _local_to_map 對同一個點要給一樣的答案
#
#  這兩個是不同的函式（一個通用、一個算飛機自己還要附機頭角度），
#  各自寫了一次加法 —— 反號寫錯一邊，症狀是「無人機畫對了但車畫錯了」，
#  而那看起來像追蹤誤差，不像座標 bug。
# =============================================================================
print("③ 通用版和飛機版必須一致")
for nx, ny in [(0.0, 0.0), (18.0, 19.6), (-3.2, 7.1), (25.0, -4.0)]:
    g = api(ORIGIN, _Lpos(nx, ny))
    gen = g._ned_to_map(nx, ny)
    own = g._local_to_map()
    check(abs(gen[0] - own[0]) < 1e-9 and abs(gen[1] - own[1]) < 1e-9,
          f"北{nx:+.1f} 東{ny:+.1f} → 兩邊都是 map({gen[0]:+.2f},{gen[1]:+.2f})",
          f"飛機版給 ({own[0]:+.2f},{own[1]:+.2f})")

# =============================================================================
#  ④ 機頭角度：PX4 的航向（北起算、順時針）→ map 的角度（東起算、逆時針）
#
#  前端直接把這個角度丟進 SVG 的 rotate(-h)，錯一個反號就是「三角形指向
#  反方向」—— 看起來像飛機在倒退飛。
# =============================================================================
print("④ 機頭角度換算")
# ⚠️ 正規化的區間是 **[-180, 180)**，不是 (-180, 180]：
#        (deg + 180) % 360 - 180
#    朝西（PX4 航向 -90°）算出 map 角度 180 → (180+180)%360-180 = **-180**。
#    -180 和 +180 是同一個方向，SVG 的 rotate() 兩個都畫得一樣，所以不特別
#    處理。這裡把區間明寫出來，是為了下次有人看到 -180 時不會以為是 bug。
for deg_ned, want in [(0.0, 90.0),        # 朝北
                      (90.0, 0.0),        # 朝東
                      (180.0, -90.0),     # 朝南
                      (-90.0, -180.0),    # 朝西 → -180（等價於 +180，見上面）
                      (45.0, 45.0)]:      # 東北
    g = api(ORIGIN, _Lpos(0.0, 0.0, math.radians(deg_ned)))
    h = g._local_to_map()[2]
    check(abs(h - want) < 1e-6, f"PX4 航向 {deg_ned:+.0f}° → map {want:+.0f}°",
          f"得到 {h:+.1f}°")
    check(-180.0 <= h < 180.0, f"  （{deg_ned:+.0f}° 的結果落在 [-180,180)）")

# 繞一整圈都要落在區間內，而且方向對得上（每 15° 掃一遍）
for d in range(-180, 181, 15):
    g = api(ORIGIN, _Lpos(0.0, 0.0, math.radians(d)))
    h = g._local_to_map()[2]
    want = ((90.0 - d) + 180.0) % 360.0 - 180.0
    if abs(h - want) >= 1e-6 or not (-180.0 <= h < 180.0):
        check(False, f"繞圈掃描 PX4 航向 {d:+d}°", f"得到 {h:+.1f}°，期望 {want:+.1f}°")
        break
else:
    check(True, "繞一整圈（每 15°）都落在 [-180,180) 且方向正確")

# =============================================================================
#  ⑤ 算不出來的時候要回 None，不可以回 (0,0)
#
#  回 (0,0) 的話儀表板會把無人機畫在倉庫正中央 —— 一個看起來完全正常、
#  但完全錯誤的位置。回 None 前端才顯示「位置未知」。
# =============================================================================
print("⑤ 原點沒設定／位置無效 → None")
for bad in ("", "  ", "abc", "5.6", "x,y", ","):
    b = api(bad, _Lpos(1.0, 2.0))
    check(b._origin_map() is None, f"local_origin_map={bad!r} → _origin_map 回 None")
    check(b._ned_to_map(1.0, 2.0) is None, f"local_origin_map={bad!r} → _ned_to_map 回 None")
    check(b._map_to_local(1.0, 2.0) is None,
          f"local_origin_map={bad!r} → _map_to_local 回 None")
    check(b._local_to_map() is None, f"local_origin_map={bad!r} → _local_to_map 回 None")

check(api(ORIGIN, None)._local_to_map() is None, "還沒收到位置 → _local_to_map 回 None")
check(api(ORIGIN, _Lpos(1.0, 2.0, xy_valid=False))._local_to_map() is None,
      "位置估計失效（xy_valid=False）→ _local_to_map 回 None")
# ⚠️ _ned_to_map 刻意**不**看 lpos：它換算的是別人的座標（視覺目標、航點），
#    和飛機自己的位置有效性無關。
check(api(ORIGIN, None)._ned_to_map(1.0, 2.0) is not None,
      "_ned_to_map 不依賴飛機位置（換算的是別人的座標）")

# 空白容錯：參數是從 launch 或 systemd 的字串傳進來的
check(api(" -14.0 , -9.0 ")._origin_map() == SPAWN, "原點字串前後有空白也要解析得出來")

print()
if FAILS:
    print(f"✗ {len(FAILS)} 項失敗：")
    for f in FAILS:
        print(f"    - {f}")
    sys.exit(1)
print("✓ 全部通過")
