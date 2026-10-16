#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""量 api/goto 實際飛多快（起飛 → 導航到一個點 → 統計速度）。

⚠️ 為什麼不量牆上的時鐘：模擬的 RTF（實時倍率）會變 —— 開不開 GUI、
   CPU 忙不忙都會影響。同一趟飛行在不同 RTF 下牆鐘秒數不一樣，
   拿來比較參數會得到假的結論（這個坑在 api/track 的搜尋上限也踩過，
   所以那邊的上限用「圈數」不是秒數）。
   PX4 的 timestamp 和 vehicle_local_position 的速度都是**模擬時間**，
   所以全部用那個算，RTF 只當附註印出來。

用法：
    python3 measure_goto.py --goal 5.6 7.0 --altitude 5.0 --label "max_vel 0.8"
"""
import argparse
import json
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition
from drone_nodered_interfaces.srv import Takeoff, GoTo


class Bench(Node):
    def __init__(self, ns):
        super().__init__("measure_goto")
        # PX4 用 BEST_EFFORT 發布，拿預設的 RELIABLE 訂會完全收不到而且不報錯
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.samples = []          # (sim_t 秒, x, y, vx, vy)
        self.nav = {"state": "idle"}
        self.connected = False
        self.alt = None
        self.create_subscription(
            VehicleLocalPosition, f"/{ns}/fmu/out/vehicle_local_position_v1",
            self._on_pos, qos)
        self.create_subscription(String, f"/{ns}/api/status", self._on_status, 10)
        self.takeoff = self.create_client(Takeoff, f"/{ns}/api/takeoff")
        self.goto = self.create_client(GoTo, f"/{ns}/api/goto")

    def _on_pos(self, m):
        if m.xy_valid:
            self.samples.append((m.timestamp * 1e-6, m.x, m.y, m.vx, m.vy))
        if m.z_valid:
            self.alt = -m.z

    def _on_status(self, m):
        try:
            s = json.loads(m.data)
        except ValueError:
            return
        self.nav = s.get("nav", {})
        self.connected = bool(s.get("connected"))

    def call(self, client, req, what):
        if not client.wait_for_service(timeout_sec=20.0):
            sys.exit(f"✗ 服務 {what} 不在")
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        r = fut.result()
        if r is None or not r.success:
            sys.exit(f"✗ {what} 失敗：{'沒有回應' if r is None else r.message}")
        print(f"  {what}：{r.message}")

    def wait(self, cond, timeout_s, what):
        """等條件成立。timeout 用牆鐘 —— 這裡只是保險，不是測量值。"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
            if cond():
                return True
        sys.exit(f"✗ 等「{what}」超過 {timeout_s:.0f} 秒")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="MAV1")
    # ⚠️ 用「x,y」一個字串而不是 nargs=2：argparse 會把「-6.0」當成選項名，
    #    而走道裡的目標點常常是負的（--goal 0.0 -6.0 會直接報錯）。
    ap.add_argument("--goal", default="5.6,7.0",
                    help="map 座標「x,y」（ENU：x 東、y 北）")
    ap.add_argument("--altitude", type=float, default=5.0)
    ap.add_argument("--label", default="")
    ap.add_argument("--origin", nargs=2, type=float, default=(-14.0, -9.0),
                    help="PX4 本地原點在 map 的位置（= 生成點，"
                         "config/warehouse_geometry.yaml 的 spawn）")
    ap.add_argument("--timeout", type=float, default=240.0, help="牆鐘保險（秒）")
    a = ap.parse_args()

    rclpy.init()
    n = Bench(a.ns)
    print(f"=== 量 api/goto：{a.label or '(沒給標籤)'} ===")
    n.wait(lambda: n.alt is not None, 30.0, "收到 PX4 的位置")
    # ⚠️ 「Nav2 的 Managed nodes are active」不代表 drone_api_node 準備好了 ——
    #    它是堆疊裡最後才起來的（API_DELAY 45 秒），剛起來那幾秒還沒收到
    #    PX4 的狀態，這時候送起飛會被拒絕（實測踩過）。等它自己說連上了。
    n.wait(lambda: n.connected, 60.0, "drone_api_node 收到 PX4 的狀態")

    print(f"① 起飛到 {a.altitude:.1f} m")
    req = Takeoff.Request(); req.altitude = a.altitude
    n.call(n.takeoff, req, "takeoff")
    n.wait(lambda: n.alt is not None and abs(n.alt - a.altitude) < 0.3, 60.0, "爬到高度")
    print(f"  到了（{n.alt:.2f} m），穩定 3 秒再出發")
    t = time.monotonic()
    while time.monotonic() - t < 3.0:
        rclpy.spin_once(n, timeout_sec=0.1)

    gx, gy = (float(v) for v in a.goal.split(","))
    print(f"② 導航到 map ({gx:+.1f}, {gy:+.1f})")
    start_i = len(n.samples)
    wall0 = time.monotonic()
    req = GoTo.Request()
    req.map_x, req.map_y, req.yaw, req.timeout = gx, gy, 999.0, 0.0
    n.call(n.goto, req, "goto")
    n.wait(lambda: n.nav.get("state") == "running", 15.0, "Nav2 開始跑")
    n.wait(lambda: n.nav.get("state") != "running", a.timeout, "導航結束")
    wall = time.monotonic() - wall0
    state = n.nav.get("state")
    print(f"  結束：{state}（{n.nav.get('message', '')}）")

    seg = n.samples[start_i:]
    if len(seg) < 10:
        sys.exit("✗ 樣本太少，量不出東西")
    sim_t = seg[-1][0] - seg[0][0]
    path = sum(math.hypot(seg[i][1] - seg[i - 1][1], seg[i][2] - seg[i - 1][2])
               for i in range(1, len(seg)))
    sp = sorted(math.hypot(s[3], s[4]) for s in seg)
    # 終點誤差：樣本是 PX4 本地 NED，目標是 map ENU，要換算才能比。
    #     ned_x(北) = map_y - origin_y     ned_y(東) = map_x - origin_x
    ox, oy = a.origin
    err = math.hypot(seg[-1][1] - (gy - oy), seg[-1][2] - (gx - ox))
    # 只看「真的在走」的那些取樣（>0.1 m/s）—— 起步和煞停的零速度會把
    # 中位數整個拉低，那不是我們要比較的東西。
    moving = [v for v in sp if v > 0.1]
    pct = lambda p: sp[min(len(sp) - 1, int(len(sp) * p))]          # noqa: E731
    print(f"""
  結果（{a.label}）
    走過的路徑長     {path:6.2f} m
    模擬時間         {sim_t:6.2f} s      ← 比較用的就是這個
    牆鐘時間         {wall:6.2f} s      （RTF 約 {sim_t / wall:.2f}）
    平均速度         {path / sim_t:6.2f} m/s
    速度中位數       {statistics.median(moving) if moving else 0:6.2f} m/s（只算 > 0.1 的）
    第 90 百分位     {pct(0.90):6.2f} m/s
    最高             {sp[-1]:6.2f} m/s
    終點誤差         {err:6.2f} m      （xy_goal_tolerance 0.4）
    取樣數           {len(seg)}
""")
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if state == "done" else 2)


if __name__ == "__main__":
    main()
