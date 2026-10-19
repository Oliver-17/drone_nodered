#!/usr/bin/env python3
# =============================================================================
#  t_no_fly.py —— 驗「跟隨段的禁飛方框真的攔得住 2026-10-19 那次摔機」
#
#  跑法（不需要 ROS、不需要 Gazebo）：
#      python3 ~/ros2_ws/src/drone_nodered/test/t_no_fly.py
#
#  為什麼要有這個測試：
#      2026-10-19 摔機一次。api/track 的**跟隨段**是 cmd_vel 直通 offboard、
#      不經過 Nav2，所以那一段沒有 costmap、沒有膨脹層、沒有任何避障；
#      整個跟隨迴圈當時唯一的位置保險是 PATROL_MAX_RANGE_M（50 m），
#      在 36×24 m 的倉庫裡等於沒有。
#
#      站位點算錯方向之後，飛機以滿速 1.40 m/s 往西開了 1.9 秒，
#      從 map x=+5.32 推到 +3.78，撞上第 2 排貨架的東面（x=+3.6）然後翻覆。
#
#      下面的測資就是從 PX4 的飛行記錄 02_46_33.ulg 抄出來的真實數字。
#      這種保險平常不會被觸發，所以**只能靠測試釘住** —— 等到下次摔機
#      才發現它壞了就太晚了。
#
#  ⚠️ 這裡故意不 import rclpy。要測的三個函式都不碰 ROS：
#     _parse_no_fly / _box_entry_axis / _clamp_no_fly。
#     用一個假的 self 餵進去（見 _Fake），這樣測試在任何機器上都跑得起來。
# =============================================================================

import math
import os
import re
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
NODE = os.path.join(HERE, "..", "scripts", "drone_api_node.py")
GEOM = os.path.join(HERE, "..", "config", "warehouse_geometry.yaml")

# 2026-10-19 那次飛行的生成點（＝ PX4 本地原點在 map 的位置）
ORIGIN = "-14.0,-9.0"


# -----------------------------------------------------------------------------
#  把要測的函式從節點裡挖出來，不執行 import（那會拉進 rclpy / px4_msgs）
# -----------------------------------------------------------------------------
def _load_methods():
    """從 drone_api_node.py 的原始碼取出三個待測函式。

    為什麼不 import：那支檔案 import rclpy 和 px4_msgs，在沒有 source 過
    工作區的終端機裡會直接失敗 —— 而這個測試的重點是「不需要環境也能跑」。
    用正規表示式切出 class 裡的那幾個 def，再 exec 進一個假的類別。
    """
    with open(NODE, encoding="utf-8") as f:
        src = f.read()
    want = ("_parse_no_fly", "_box_entry_axis", "_clamp_no_fly", "_map_to_local",
            "_target_velocity")
    chunks = []
    for name in want:
        # 從 "    def <name>(" 一路抓到下一個同縮排的 def／class 之前
        m = re.search(r"^    (?:@staticmethod\n    )?def %s\(.*?(?=^    (?:@|def )|^class )"
                      % re.escape(name), src, re.M | re.S)
        if not m:
            print(f"✗ 在 {NODE} 裡找不到 {name}() —— 函式被改名或刪掉了？")
            sys.exit(1)
        chunks.append(m.group(0))
    ns = {"math": math}
    # _target_velocity 會用到幾個模組層級的常數，一起挖出來
    for line in src.splitlines():
        if re.match(r"^TRACK_(VEL|MIN_TARGET)\w*\s*=", line):
            exec(line.split("#")[0], ns)                 # noqa: S102
    exec("class _Api:\n" + "\n".join(chunks), ns)        # noqa: S102
    return ns["_Api"], ns


_Api, _CONST = _load_methods()


class _Fake(_Api):
    """假的節點：只有待測函式需要的那幾個屬性。"""

    def __init__(self, boxes_text, margin=0.8, lookahead=1.0, origin=ORIGIN):
        self.local_origin_map = origin
        self.no_fly_margin = margin
        self.no_fly_lookahead_s = lookahead
        self.lpos = None
        self.logs = []
        self._no_fly = self._parse_no_fly(boxes_text)

    # 待測函式會呼叫 self.get_logger().error(...) / .warn(...)
    def get_logger(self):
        rec = self.logs
        return types.SimpleNamespace(
            error=rec.append, warn=rec.append, info=rec.append)

    def at(self, north, east):
        self.lpos = types.SimpleNamespace(x=north, y=east)
        return self


def _boxes_from_yaml():
    """從產生出來的 warehouse_geometry.yaml 讀 no_fly_boxes（和 launch 同一招）。"""
    with open(GEOM, encoding="utf-8") as f:
        m = re.search(r"^no_fly_boxes:\s*\[([^\]]*)\]", f.read(), re.M)
    if not m:
        print(f"✗ {GEOM} 裡沒有 no_fly_boxes —— 請重跑 "
              "python3 scripts/gen_warehouse.py --target hatchback")
        sys.exit(1)
    return m.group(1).replace(" ", "")


def _map_to_ned(mx, my):
    """map(ENU) → PX4 本地 NED，和 _map_to_local 同一條式子（測試自己算一份對帳）。"""
    ox, oy = (float(v) for v in ORIGIN.split(","))
    return (my - oy, mx - ox)


FAILS = []


def check(ok, what, detail=""):
    print(("  ✓ " if ok else "  ✗ ") + what + (f"   {detail}" if detail else ""))
    if not ok:
        FAILS.append(what)


# =============================================================================
#  ① 剖析：方框數量、座標換算、兩個軸有沒有互換
# =============================================================================
print("① 剖析 no_fly_boxes")
api = _Fake(_boxes_from_yaml())
check(len(api._no_fly) == 8, "讀到 8 個方框（4 排貨架 + 4 面外牆）",
      f"實際 {len(api._no_fly)}")

# 第 2 排貨架（map x 2.0~3.6、y -7~+7）是撞到的那一個。
# 膨脹 0.8 之後 map 的東面應該在 x=4.4 → NED east = 4.4 - (-14) = 18.4
shelf2 = [b for b in api._no_fly if "x[+2.0,+3.6]" in b[4]]
check(len(shelf2) == 1, "找得到第 2 排貨架的方框")
if shelf2:
    n0, e0, n1, e1, _ = shelf2[0]
    exp_n0, exp_e0 = _map_to_ned(2.0, -7.0)
    exp_n1, exp_e1 = _map_to_ned(3.6, 7.0)
    check(abs(n0 - (exp_n0 - 0.8)) < 1e-9 and abs(n1 - (exp_n1 + 0.8)) < 1e-9,
          "north 區間來自 map 的 y（兩個軸沒有搞反）",
          f"north {n0:+.1f}~{n1:+.1f}")
    check(abs(e0 - (exp_e0 - 0.8)) < 1e-9 and abs(e1 - (exp_e1 + 0.8)) < 1e-9,
          "east 區間來自 map 的 x、膨脹 0.8",
          f"east {e0:+.1f}~{e1:+.1f}（map x 1.2~4.4）")

print("\n① 壞輸入要安全地退成「沒有方框」")
check(_Fake("")._no_fly == [], "空字串 → 空清單")
check(_Fake("1,2,3")._no_fly == [], "數字不是 4 的倍數 → 整組不用")
check(_Fake("1,2,x,4")._no_fly == [], "有非數字 → 整組不用")
check(_Fake("1,2,3,4", origin="")._no_fly == [],
      "沒有 local_origin_map → 整組不用（不能拿 map 座標當 NED 用）")

# =============================================================================
#  ② 重演 2026-10-19 的摔機：同樣的位置、同樣的速度指令
# =============================================================================
print("\n② 重演摔機（02_46_33.ulg 的真實數字）")
# 67.6 秒：飛機在 map (5.32, -1.34)，指令是「滿速 1.40 m/s 往西」。
# 往西 = map 的 -x = NED 的 -east。
crash_n, crash_e = _map_to_ned(5.32, -1.34)
vn, ve, blocked = api.at(crash_n, crash_e)._clamp_no_fly(0.0, -1.40)
check(blocked is not None, "當時的那一道指令會被方框攔下來", f"{blocked}")
check(abs(ve) < 1e-9, "往西（朝貨架）的分量被砍成 0", f"east {ve:+.2f}")

# 68.5 秒：已經推到 map x=3.78，再 0.1 秒就撞上了
late_n, late_e = _map_to_ned(3.78, -0.39)
vn2, ve2, blocked2 = api.at(late_n, late_e)._clamp_no_fly(0.0, -1.17)
check(blocked2 is not None, "就算已經飛到 x=3.78（貨架前 0.18 m）也還是攔得住")
check(ve2 >= -1e-9, "不會再往貨架靠", f"east {ve2:+.2f}")

# =============================================================================
#  ③ 平行貨架面的分量必須留著 —— 不然車沿走道走，飛機一靠近就停住跟丟
# =============================================================================
print("\n③ 只砍法線方向，平行的要留著")
# 貨架東面外 0.3 m（map x=4.7，在膨脹框 4.4 之外），指令「往西 + 往北」
side_n, side_e = _map_to_ned(4.7, 0.0)
vn3, ve3, blocked3 = api.at(side_n, side_e)._clamp_no_fly(1.00, -1.00)
check(blocked3 is not None, "朝貨架的指令被攔住")
check(abs(ve3) < 1e-9, "東西向（垂直貨架面）砍成 0", f"east {ve3:+.2f}")
check(abs(vn3 - 1.00) < 1e-9, "南北向（平行貨架面）完整保留 → 還能沿走道跟車",
      f"north {vn3:+.2f}")

print("\n③ 走道中央正常跟隨時不應該被干擾")
mid_n, mid_e = _map_to_ned(5.6, 0.0)       # 走道中心，離膨脹框 1.2 m
vn4, ve4, blocked4 = api.at(mid_n, mid_e)._clamp_no_fly(1.00, 0.0)
check(blocked4 is None and abs(vn4 - 1.00) < 1e-9,
      "沿走道往北 1.0 m/s 完全不被夾", f"north {vn4:+.2f}")
# 往西 0.5 m/s：1 秒後到 map x=5.1，還在膨脹框（4.4）外 → 不該夾
vn5, ve5, blocked5 = api.at(mid_n, mid_e)._clamp_no_fly(0.0, -0.5)
check(blocked5 is None, "往貨架靠但 1 秒內還到不了 → 不夾（不會過度保守）",
      f"east {ve5:+.2f}")
# 往西 1.4 m/s：1 秒後到 map x=4.2，進到膨脹框 → 要夾
vn6, ve6, blocked6 = api.at(mid_n, mid_e)._clamp_no_fly(0.0, -1.4)
check(blocked6 is not None and abs(ve6) < 1e-9,
      "同一個位置但速度夠快到得了 → 要夾（預判有在作用）")

# =============================================================================
#  ④ 外牆：環線的 y=±9 橫段離牆只有 3 m
# =============================================================================
print("\n④ 外牆（北牆 y=+12，環線橫段在 y=+9）")
wall_n, wall_e = _map_to_ned(5.6, 9.0)
vn7, ve7, blocked7 = api.at(wall_n, wall_e)._clamp_no_fly(1.40, 0.0)
check(blocked7 is None, "離北牆 3 m、1 秒只飛 1.4 m → 還不用夾")
near_n, near_e = _map_to_ned(5.6, 10.5)
vn8, ve8, blocked8 = api.at(near_n, near_e)._clamp_no_fly(1.40, 0.0)
check(blocked8 is not None and abs(vn8) < 1e-9,
      "離北牆 1.5 m 還往北衝 → 砍掉", f"{blocked8}")

# =============================================================================
#  ⑤ 已經在膨脹框裡：要能飛出來，不能被鎖死
# =============================================================================
print("\n⑤ 已經在膨脹框內（餘裕被吃掉）時只准往外")
in_n, in_e = _map_to_ned(4.0, 0.0)          # map x=4.0，在貨架面 3.6 和膨脹邊 4.4 之間
vn9, ve9, blocked9 = api.at(in_n, in_e)._clamp_no_fly(0.0, +1.00)
check(abs(ve9 - 1.00) < 1e-9, "往外（往東離開貨架）要放行", f"east {ve9:+.2f}")
vn10, ve10, blocked10 = api.at(in_n, in_e)._clamp_no_fly(0.0, -1.00)
check(abs(ve10) < 1e-9 and blocked10 is not None,
      "往裡（繼續朝貨架）要砍掉", f"east {ve10:+.2f}")

# =============================================================================
#  ⑥ 沒有方框時必須原封不動放行（空白世界、實機都走這條）
# =============================================================================
print("\n⑥ 沒有設定方框時不改動任何指令")
bare = _Fake("").at(0.0, 0.0)
check(bare._clamp_no_fly(1.23, -0.45) == (1.23, -0.45, None),
      "原值原封不動回傳")

# =============================================================================
#  ⑦ 車速估計的可信度門檻 —— 摔機的另一半原因
# =============================================================================
print("\n⑦ 車速估計：資料不夠就要回 (0,0)，不要給一個亂的方向")
WIN = _CONST["TRACK_VEL_WINDOW_S"]
NEED_T = _CONST["TRACK_VEL_MIN_WINDOW_S"]
NEED_N = _CONST["TRACK_VEL_MIN_SAMPLES"]
check(NEED_T >= 0.6 and NEED_N >= 4,
      f"門檻有收緊（窗 >= {NEED_T} 秒、>= {NEED_N} 筆）")


def vel(hist):
    """hist 是 [(時間, 北, 東), ...]"""
    o = _Fake("")
    o._target_hist = list(hist)
    return o._target_velocity()


# 摔機當下的樣子：剛鎖定，只有 3 筆、跨 0.25 秒。
# 車其實往北走 0.5 m/s，但姿態劇變讓算出來的位置亂跳 ±0.3 m ——
# 差分之後方向完全是雜訊（這裡故意讓它看起來像「往東 2.4 m/s」）。
check(vel([(10.00, 0.0, 0.0), (10.12, 0.1, 0.3), (10.25, 0.1, 0.6)]) == (0.0, 0.0),
      "3 筆 / 0.25 秒 → 回 (0,0)（舊門檻 0.2 秒會放過這組，就是摔機那次）")
check(vel([(10.0, 0.0, 0.0), (10.5, 0.0, 0.25)]) == (0.0, 0.0),
      "2 筆 / 0.5 秒 → 筆數不足，回 (0,0)")
check(vel([(10.0, 0.0, 0.0), (10.2, 0.1, 0.0),
           (10.4, 0.2, 0.0), (10.5, 0.25, 0.0)]) == (0.0, 0.0),
      "4 筆但只跨 0.5 秒 → 時間窗不足，回 (0,0)")

# 資料夠了就要算得出來：車往北 0.5 m/s，10 筆、跨 0.9 秒
good = [(10.0 + i * 0.1, i * 0.05, 0.0) for i in range(10)]
vx, vy = vel(good)
check(abs(vx - 0.5) < 1e-6 and abs(vy) < 1e-9,
      "10 筆 / 0.9 秒、車往北 0.5 m/s → 估得出 0.50", f"北{vx:+.2f} 東{vy:+.2f}")

# deque 裡留著上一次看到目標的舊資料時，不能把舊資料算進可信度
stale = [(1.0, 0.0, 0.0), (1.1, 0.0, 0.0), (1.2, 0.0, 0.0), (1.3, 0.0, 0.0),
         (50.0, 9.0, 9.0), (50.2, 9.1, 9.0)]
check(vel(stale) == (0.0, 0.0),
      "舊資料 + 2 筆新資料 → 回 (0,0)（筆數不能用整個 deque 的長度算）")

print()
if FAILS:
    print(f"✗ {len(FAILS)} 項沒過：")
    for f in FAILS:
        print("   " + f)
    sys.exit(1)
print("✓ 全部通過 —— 禁飛方框攔得住 2026-10-19 那次摔機，"
      "而且不會把正常的跟隨夾死")
