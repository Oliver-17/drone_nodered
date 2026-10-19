#!/usr/bin/env python3
# =============================================================================
#  measure_target_error.py —— 量「視覺算出來的目標位置」準不準
#
#  用法（模擬跑起來、飛機已經看得到車之後）：
#      python3 measure_target_error.py [秒數]
#
#  做法：訂 <ns>/api/target（target_world_node 用射線和地面交會算出來的位置），
#       和車在世界裡的**真實座標**比對。車靜止，所以真值是常數，不用再讀 Gazebo
#       （少一個 gz 訂閱，也就少一個干擾來源）。
#
#  ⚠️ 這支量的是「整條視覺鏈的端到端精度」：
#       偵測框的中心 → 相機內參反投影 → 相機安裝位姿 → 機體姿態 → 和地面交會
#     任何一環的幾何參數錯了都會在這裡現形，而那些錯誤**都不會報錯**，
#     只會讓算出來的位置固定偏一個方向 —— 所以要量，不能只看「有沒有偵測到」。
# =============================================================================
import math
import sys

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

# 車的真實位置（世界 ENU）。gz/worlds/warehouse.sdf 的 include pose，
# 車靜止（target:=false）所以是常數。
TARGET_ENU = (0.0, -6.0)
# PX4 本地 NED 的原點 = 飛機生成點，config/warehouse_geometry.yaml 的 spawn
SPAWN_ENU = (-14.0, -9.0)


def enu_to_ned(ex, ey):
    """ENU(x 東, y 北) → PX4 本地 NED(x 北, y 東)，原點在生成點。"""
    return (ey - SPAWN_ENU[1], ex - SPAWN_ENU[0])


class Measure(Node):
    def __init__(self):
        super().__init__("measure_target_error")
        self.samples = []
        self.create_subscription(PointStamped, "/MAV1/api/target", self._on, 10)

    def _on(self, msg):
        self.samples.append((msg.point.x, msg.point.y))


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    truth = enu_to_ned(*TARGET_ENU)
    print(f"車的真實位置：ENU {TARGET_ENU} → NED (北 {truth[0]:.3f}, 東 {truth[1]:.3f})")

    rclpy.init()
    n = Measure()
    t0 = n.get_clock().now()
    print(f"收 {secs:.0f} 秒…")
    while (n.get_clock().now() - t0).nanoseconds / 1e9 < secs:
        rclpy.spin_once(n, timeout_sec=0.2)

    if not n.samples:
        print("✗ 完全沒收到 api/target —— 飛機看得到車嗎？視覺鏈有起來嗎？")
        rclpy.try_shutdown()
        return 1

    xs = [s[0] for s in n.samples]
    ys = [s[1] for s in n.samples]
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    errs = [math.hypot(x - truth[0], y - truth[1]) for x, y in n.samples]
    errs_s = sorted(errs)
    print(f"\n收到 {len(n.samples)} 筆")
    print(f"  算出來的位置：平均 (北 {mx:.3f}, 東 {my:.3f})")
    print(f"  真值        ：     (北 {truth[0]:.3f}, 東 {truth[1]:.3f})")
    print(f"  平均偏移    ：北 {mx - truth[0]:+.3f}  東 {my - truth[1]:+.3f}")
    print(f"  距離誤差    ：中位 {errs_s[len(errs_s) // 2]:.3f} m  "
          f"最小 {errs_s[0]:.3f}  最大 {errs_s[-1]:.3f}")
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
