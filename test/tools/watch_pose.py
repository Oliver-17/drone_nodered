#!/usr/bin/env python3
"""訂 Gazebo 的姿態廣播，記錄某個模型的 x / y / yaw。

為什麼不用 `gz model -m X -p`：它的姿態輸出是**兩行**（XYZ 一行、RPY 下一行），
用 grep -A1 只會抓到 XYZ，yaw 永遠是空的（2026-10-08 踩過）。
而且一次 spawn 一個行程，取樣頻率拉不高。
"""
import math
import sys
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node


def main():
    world, model, secs, period = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
    node = Node()
    latest = {}

    def on(msg):
        for p in msg.pose:
            if p.name == model:
                q = p.orientation
                # 四元數 → yaw（繞 z）。Gazebo 的姿態是 ENU，所以這個 yaw
                # 就是「機頭和東方的夾角」，和 atan2(北分量, 東分量) 同一個基準。
                latest["v"] = (p.position.x, p.position.y, p.position.z,
                               math.atan2(2 * (q.w * q.z + q.x * q.y),
                                          1 - 2 * (q.y * q.y + q.z * q.z)))
                return

    topic = f"/world/{world}/dynamic_pose/info"
    if not node.subscribe(Pose_V, topic, on):
        print(f"  x 訂不到 {topic}")
        return 1
    end = time.monotonic() + secs
    rows = []
    while time.monotonic() < end:
        if "v" in latest:
            rows.append((time.monotonic(), *latest["v"]))
        time.sleep(period)
    if not rows:
        print(f"  x {secs:.0f} 秒內沒收到 {model} 的姿態")
        return 1
    t0 = rows[0][0]
    # 欄位：時間 x y z yaw。z 是必要的 ——
    # 2026-10-08 少了它，差點答不出「飛機有沒有從貨架上面飛過去」。
    for t, x, y, z, yaw in rows:
        print(f"{t - t0:.2f} {x:.4f} {y:.4f} {z:.4f} {yaw:.5f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
