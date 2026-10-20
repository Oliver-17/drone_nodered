#!/usr/bin/env python3
# =============================================================================
#  t_search_waypoint.py —— 驗「搜尋航點真的生效了」這道保險
#
#  跑法（不需要 ROS、不需要 Gazebo）：
#      python3 ~/ros2_ws/src/drone_nodered/test/t_search_waypoint.py
#
#  為什麼要有這個測試：
#      2026-10-20 Oliver 回報「它沒有去巡航找車」。從飛行記錄
#      （instance_0/log/2026-10-20/10_49_04.ulg）還原出來的 30 毫秒競態：
#          58.15 s  我們送 DO_REPOSITION（第一個搜尋航點）→ ACK **ACCEPTED**
#          58.17 s  PX4 → AUTO_LOITER，navigator 接手
#          58.18 s  cmd_vel_to_px4_node 送 DO_SET_MODE 搶回 OFFBOARD
#          58.20 s  PX4 → OFFBOARD。搜尋航點作廢，飛機定在原地
#          68.26 s  bridge 終於放手 → AUTO_LOITER。**白站 10.1 秒**
#      那一趟「搜尋 25 秒」裡有 10 秒站著不動，是車自己開進視野才追到的。
#
#  🔴 這個病最惡劣的地方：**ACK 是 ACCEPTED**。
#     Commander 收到 DO_REPOSITION 只做一件事 —— 切 AUTO_LOITER，切成功就
#     回 ACCEPTED（Commander.cpp:756-778）。「切完之後有沒有人把模式搶走」
#     不在它的 ACK 範圍裡。所以光看 ACK 永遠查不出來。
#
#     治法是回頭確認 nav_state 真的離開 OFFBOARD。這支測試就是釘住那件事：
#     ACCEPTED 但模式沒變，必須被當成失敗處理，不能默默往下走。
# =============================================================================

import math
import os
import re
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
NODE = os.path.join(HERE, "..", "scripts", "drone_api_node.py")


def _load():
    """挖出 _send_search_waypoint 和 _left_offboard，以及它們用到的常數。

    不 import 節點：那支檔案 import rclpy 和 px4_msgs（手法同 t_no_fly.py）。
    """
    with open(NODE, encoding="utf-8") as f:
        src = f.read()
    ns = {"math": math, "time": time}
    for line in src.splitlines():
        if re.match(r"^TRACK_SEARCH_(CONFIRM_S|TRIES)\s*=", line):
            exec(line.split("#")[0], ns)                     # noqa: S102
    chunks = []
    for name in ("_left_offboard", "_send_search_waypoint"):
        m = re.search(r"^    def %s\(self.*?(?=^    (?:@|def )|^class )"
                      % re.escape(name), src, re.M | re.S)
        if not m:
            print(f"✗ 在 {NODE} 裡找不到 {name}()")
            sys.exit(1)
        chunks.append(m.group(0))
    exec("class _Api:\n" + "\n".join(chunks), ns)             # noqa: S102
    return ns["_Api"], ns


_Api, _C = _load()
CONFIRM_S = _C["TRACK_SEARCH_CONFIRM_S"]
TRIES = _C["TRACK_SEARCH_TRIES"]

ACCEPTED = 0
TEMPORARILY_REJECTED = 1

FAILS = []


def check(ok, what, detail=""):
    print(("  ✓ " if ok else "  ✗ ") + what + (f"   {detail}" if detail else ""))
    if not ok:
        FAILS.append(what)


class _Logger:
    def __init__(self):
        self.warns = []
        self.infos = []

    def warn(self, m):
        self.warns.append(m)

    def info(self, m):
        self.infos.append(m)


class _Status:
    """只有 _left_offboard 會讀到的那一個欄位。"""

    def __init__(self, nav_state):
        self.nav_state = nav_state


class _Lpos:
    def __init__(self, x=0.0, y=0.0):
        self.x, self.y = x, y


# PX4 的 nav_state 常數（vehicle_status.msg）
OFFBOARD = 14
AUTO_LOITER = 4


class Fake(_Api):
    """假節點：把 _send、模式、release_hold 全部換成可控的東西。

    mode_after 是一串「每次 _send 之後 PX4 的模式」—— 用它重演
    「ACCEPTED 但模式被搶回去」的情境。
    """

    def __init__(self, acks, modes, released_becomes=None):
        self.acks = list(acks)              # 每次 _send 回什麼 ack
        self.modes = list(modes)            # 每一次 _send 之後的模式
        self.released_becomes = released_becomes   # release_hold 之後模式變成什麼
        self.sent = []                      # 送出去的 (lat, lon, yaw)
        self.releases = 0
        self.status = _Status(OFFBOARD)
        self.lpos = _Lpos()
        self.cancel_settle_s = 0.01         # 測試不要真的睡 0.8 秒
        self._track_stop = threading.Event()
        self._log = _Logger()

    # --- 被測函式用到的東西 ---------------------------------------------------
    def get_logger(self):
        return self._log

    def _local_to_global(self, px, py):
        return (47.0 + px * 1e-5, 8.0 + py * 1e-5)

    def _send(self, command, **kw):
        self.sent.append((kw.get("p5"), kw.get("p6"), kw.get("p4")))
        if self.modes:
            self.status.nav_state = self.modes.pop(0)
        return self.acks.pop(0) if self.acks else ACCEPTED

    def _ack_text(self, r):
        return {None: "PX4 沒有回應（逾時）",
                TEMPORARILY_REJECTED: "TEMPORARILY_REJECTED"}.get(r, str(r))

    def _release_bridge_hold(self):
        self.releases += 1
        if self.released_becomes is not None:
            self.status.nav_state = self.released_becomes
        return True


# _left_offboard 用 VehicleStatus.NAVIGATION_STATE_OFFBOARD，補上那個常數
class _VS:
    NAVIGATION_STATE_OFFBOARD = OFFBOARD


# 被 exec 出來的 function 沒有自己的模組命名空間，px4_msgs 的常數要塞進
# 它的 __globals__ —— 不然被測函式一跑到 VehicleStatus 就 NameError。
_Api._left_offboard.__globals__["VehicleStatus"] = _VS
_Api._send_search_waypoint.__globals__.update({
    "VehicleStatus": _VS,
    "VehicleCommand": type("VC", (), {"VEHICLE_CMD_DO_REPOSITION": 192}),
    "VehicleCommandAck": type("VCA", (), {"VEHICLE_CMD_RESULT_ACCEPTED": ACCEPTED}),
    "REPOSITION_CHANGE_MODE": 1.0,
})

# =============================================================================
#  ① 一切正常：送一次就切好了
# =============================================================================
print("① 正常情況")
f = Fake(acks=[ACCEPTED], modes=[AUTO_LOITER])
t0 = time.monotonic()
r = f._send_search_waypoint(18.0, 19.6, 492.0)
dt = time.monotonic() - t0
check(r is True, "回 True（PX4 真的接手了）", f"得到 {r!r}")
check(len(f.sent) == 1, "只送一次", f"送了 {len(f.sent)} 次")
check(f.releases == 0, "沒有必要請 bridge 放手")
# 🔴 正常情況不可以每個航點都多花 CONFIRM_S 秒 —— 搜尋環線 4 個點，
#    每點多等 1 秒就是整趟多 4 秒，而且是白等的
check(dt < CONFIRM_S * 0.5, f"確認用輪詢，正常情況遠少於 {CONFIRM_S:.0f} 秒",
      f"花了 {dt * 1000:.0f} ms")
# 機頭朝行進方向（飛機在原點，航點在北 18 東 19.6 → atan2(19.6, 18)）
check(abs(f.sent[0][2] - math.atan2(19.6, 18.0)) < 1e-9,
      "機頭朝行進方向（相機只看得到前方）")

# =============================================================================
#  ② 🔴 重演那次 bug：ACCEPTED 但模式被搶回 OFFBOARD
# =============================================================================
print("② ACCEPTED 但模式被搶回 OFFBOARD（2026-10-20 的真實情境）")
# 第一次送：ACCEPTED，但模式還是 OFFBOARD（bridge 搶走了）
# release_hold 之後 bridge 放手 → 第二次送就成功
f = Fake(acks=[ACCEPTED, ACCEPTED], modes=[OFFBOARD, AUTO_LOITER],
         released_becomes=OFFBOARD)
r = f._send_search_waypoint(18.0, 19.6, 492.0)
check(r is True, "請 bridge 放手再重送 → 成功", f"得到 {r!r}")
check(len(f.sent) == 2, "總共送了 2 次", f"送了 {len(f.sent)} 次")
check(f.releases == 1, "請 bridge 放手 1 次", f"{f.releases} 次")
check(any("搶控制權" in w for w in f._log.warns),
      "有印出「有人在搶控制權」的警告",
      f._log.warns[0][:60] if f._log.warns else "一則都沒有")

# =============================================================================
#  ③ 一直被搶 → 必須當成失敗，不可以默默往下走
#
#  這是整支測試最重要的一條。舊程式在這個情境下回報「成功」，
#  飛機站在原地 10 秒，而 log 上一片祥和。
# =============================================================================
print("③ 一直被搶 → 回報失敗")
f = Fake(acks=[ACCEPTED] * 10, modes=[OFFBOARD] * 10, released_becomes=OFFBOARD)
r = f._send_search_waypoint(18.0, 19.6, 492.0)
check(isinstance(r, str), "回失敗訊息（不是 True）", f"得到 {r!r}")
check("OFFBOARD" in r and "cmd_vel" in r,
      "訊息講得出根因（有人在送 cmd_vel 搶控制權）", r)
check(len(f.sent) == TRIES, f"送滿 {TRIES} 次才放棄", f"送了 {len(f.sent)} 次")
# 最後一次失敗後不該再多叫一次 release（那是白做的）
check(f.releases == TRIES - 1, f"release_hold 叫 {TRIES - 1} 次（最後一次不用）",
      f"{f.releases} 次")

# =============================================================================
#  ④ ACK 本身被拒 → 立刻失敗，不要重試
#
#  TEMPORARILY_REJECTED 是 PX4 說「我現在不能切模式」，重送 3 次也是一樣的
#  答案，而且會多花 3 秒。這和「被搶控制權」是兩件不同的事。
# =============================================================================
print("④ ACK 被拒 → 立刻失敗")
f = Fake(acks=[TEMPORARILY_REJECTED], modes=[OFFBOARD])
r = f._send_search_waypoint(18.0, 19.6, 492.0)
check(isinstance(r, str) and "被拒絕" in r, "回「被拒絕」", r)
check(len(f.sent) == 1, "只送一次，不重試", f"送了 {len(f.sent)} 次")

print("   （ACK 逾時也一樣）")
f = Fake(acks=[None], modes=[OFFBOARD])
r = f._send_search_waypoint(18.0, 19.6, 492.0)
check(isinstance(r, str) and "沒有回應" in r, "逾時 → 回「沒有回應」", r)
check(len(f.sent) == 1, "只送一次", f"送了 {len(f.sent)} 次")

# =============================================================================
#  ⑤ 追蹤被取消 → 回 None，呼叫端要直接 return
#
#  不能回 False/字串 —— 那會讓呼叫端把「使用者按了降落」報成任務失敗。
# =============================================================================
print("⑤ 等待期間被取消")
f = Fake(acks=[ACCEPTED] * 5, modes=[OFFBOARD] * 5, released_becomes=OFFBOARD)
threading.Timer(0.05, f._track_stop.set).start()
t0 = time.monotonic()
r = f._send_search_waypoint(18.0, 19.6, 492.0)
check(r is None, "回 None（既不是成功也不是失敗）", f"得到 {r!r}")
check(time.monotonic() - t0 < CONFIRM_S,
      "取消之後立刻就回來，不會等滿確認時間",
      f"花了 {(time.monotonic() - t0) * 1000:.0f} ms")

# =============================================================================
#  ⑥ _left_offboard 的邊界：狀態還沒收到時不能當成「已經離開」
# =============================================================================
print("⑥ _left_offboard 的邊界")
f = Fake(acks=[], modes=[])
f.status = None
check(f._left_offboard() is False,
      "還沒收到 vehicle_status → False（不知道就不要假設已經切好了）")
f.status = _Status(OFFBOARD)
check(f._left_offboard() is False, "OFFBOARD → False")
for ns, name in [(AUTO_LOITER, "AUTO_LOITER"), (3, "AUTO_MISSION"), (18, "AUTO_LAND")]:
    f.status = _Status(ns)
    check(f._left_offboard() is True, f"{name} → True")

print()
if FAILS:
    print(f"✗ {len(FAILS)} 項失敗：")
    for x in FAILS:
        print(f"    - {x}")
    sys.exit(1)
print("✓ 全部通過 —— 「ACCEPTED 但被搶走」會被抓到並重送，重送無效時回報失敗")
