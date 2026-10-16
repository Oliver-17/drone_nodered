#!/usr/bin/env python3
# =============================================================================
#  target_world_node.py —— 把「相機看到的方向」換成「目標在世界的哪裡」
#
#  用法：
#      ros2 run drone_nodered target_world_node.py --ros-args -p px4_namespace:=MAV1
#
#  輸入：/target_ray（drone_vision 發的單位方向，相機光學系）
#        PX4 的位置和姿態
#  輸出：<ns>/api/target —— PointStamped，PX4 的本地 NED 座標（x 北、y 東、z 下）
#
#  ---------------------------------------------------------------------------
#  為什麼用「射線和地面交會」而不是深度
#  ---------------------------------------------------------------------------
#  2026-09-28 實測：Gazebo 這顆模擬深度相機只有上約 1/3 的影像有值，目標所在的
#  中央和下方全是 -inf。與其跟渲染問題纏鬥，不如用幾何：
#      我們知道飛機多高、知道相機朝哪個方向，而目標在地上 —— 三個條件就夠了。
#      射線從相機射出去，和「地面平面」交會的那一點就是目標。
#
#  這個方法對地面目標又穩又準（模擬的地面是完全平的），而且完全不依賴深度。
#  實機接上 D435i 之後，深度可以拿來當第二個來源互相驗證。
#
#  ---------------------------------------------------------------------------
#  三次座標轉換（每一次都容易錯，所以分開寫清楚）
#  ---------------------------------------------------------------------------
#    1. 相機光學系（x 右、y 下、z 前）→ 相機機體系（x 前、y 左、z 上）
#    2. 相機機體系 → 無人機機體 FRD（x 前、y 右、z 下），要加上相機的安裝俯角
#    3. 機體 FRD → 世界 NED，用 PX4 的**完整姿態**（四元數）旋轉
#
#  ⚠️ 只用機頭角、假設飛機水平是不夠的：追蹤時飛機為了加速會前傾好幾度，
#     每傾 1 度，3 公尺高算出來的地面點就差約 5 公分。
#     2026-09-28 第一版就是這樣，位置系統性偏近 0.6 m。
#  ⚠️ 相機不在機體原點上（往前 0.12 m、往上 0.242 m），射線的起點要跟著移，
#     否則高度少算 0.24 m，距離會短 8%。
# =============================================================================

import math

import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)


class TargetWorld(Node):
    def __init__(self):
        super().__init__("target_world_node")

        def p(name, default):
            return self.declare_parameter(name, default).value

        ns = p("px4_namespace", "MAV1")
        # 相機的安裝俯角（度，往下為正）。和 gz/models/x500_depth_tilt 的 0.785 rad 一致。
        # 改機體模型的角度時，這裡要一起改。
        self.pitch = math.radians(p("camera_pitch_deg", 45.0))
        # 目標中心離地多高（洋紅方塊中心約在 0.2 m）。射線要和「這個高度的平面」交會，
        # 不是和地面 z=0 交會 —— 差這一點會讓算出來的位置往遠處偏。
        # 洋紅方塊的中心離地高度：車體中心 0.075 + 方塊相對車體 0.2 = 0.275
        self.target_height = p("target_height_m", 0.275)
        # 相機在機體座標（FRD：x 前、y 右、z 下）的安裝位置
        self.mount = [p("camera_x_m", 0.12), p("camera_y_m", 0.0), p("camera_z_m", -0.242)]
        # 太接近水平的射線會和地面交在很遠的地方，誤差被放大到沒有意義
        self.min_down = p("min_down_component", 0.15)

        self.lpos = None
        self.att = None
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(VehicleLocalPosition,
                                 f"/{ns}/fmu/out/vehicle_local_position_v1",
                                 lambda m: setattr(self, "lpos", m), qos)
        self.create_subscription(VehicleAttitude, f"/{ns}/fmu/out/vehicle_attitude",
                                 lambda m: setattr(self, "att", m), qos)
        self.create_subscription(Vector3Stamped, "/target_ray", self._on_ray,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(PointStamped, f"/{ns}/api/target", 10)

        self._ok = 0
        self._bad = {}
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"目標世界座標啟動：相機俯角 {math.degrees(self.pitch):.0f}°、"
            f"目標高度 {self.target_height} m → 發布 /{ns}/api/target")

    @staticmethod
    def _rotate(q, v):
        """用四元數 q（w, x, y, z）把向量 v 從機體轉到 NED。

        寫開來而不用 tf2：這裡只需要一次旋轉，拉進 tf2 會多一整套依賴，
        而且 tf2 用的是 ROS 的 (x,y,z,w) 順序，混用兩種順序很容易出錯。
        """
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
        # v' = v + 2w(q_vec × v) + 2 q_vec × (q_vec × v)
        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)
        return (vx + w * tx + (y * tz - z * ty),
                vy + w * ty + (z * tx - x * tz),
                vz + w * tz + (x * ty - y * tx))

    def _skip(self, why):
        self._bad[why] = self._bad.get(why, 0) + 1

    def _on_ray(self, msg):
        lp, att = self.lpos, self.att
        if lp is None or not lp.xy_valid or not lp.z_valid:
            self._skip("位置估計無效")
            return
        if att is None:
            self._skip("還沒收到姿態")
            return

        # --- 1) 光學系 → 相機機體系 ---------------------------------------------
        # 光學：x 右、y 下、z 前  ／  機體：x 前、y 左、z 上
        r = msg.vector
        cx, cy, cz = r.z, -r.x, -r.y

        # --- 2) 相機機體系 → 無人機 FRD ------------------------------------------
        # 先繞 y 軸轉「俯角」把相機的下傾轉回機身（往下看＝繞 y 正轉），
        # 再把 FLU 換成 FRD（y、z 反向）
        c, s = math.cos(self.pitch), math.sin(self.pitch)
        fx = c * cx + s * cz          # 前
        fy = -cy                      # 右（FLU 的左 → FRD 的右）
        fz = -(-s * cx + c * cz)      # 下

        # --- 3) 機體 FRD → 世界 NED（用完整姿態，不是只有機頭角）--------------------
        # PX4 的 q 是 (w, x, y, z)，代表「機體 FRD → NED」的旋轉
        nx, ny, nz = self._rotate(att.q, (fx, fy, fz))

        # 射線的起點是「相機」不是「機體原點」，安裝位移也要轉到 NED
        mx, my, mz = self._rotate(att.q, self.mount)
        ox, oy, oz = lp.x + mx, lp.y + my, lp.z + mz

        # --- 4) 和「目標高度的水平面」交會 ------------------------------------------
        # NED 的 z 往下為正。飛機在 lp.z（負值代表在原點上方），
        # 目標平面在 -target_height 的高度。
        if nz < self.min_down:
            self._skip("射線太接近水平")
            return
        plane_z = -self.target_height
        t = (plane_z - oz) / nz       # 沿射線要走多遠才碰到那個平面
        if t <= 0:
            self._skip("射線沒有指向地面")
            return

        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "ned"
        out.point.x = float(ox + nx * t)
        out.point.y = float(oy + ny * t)
        out.point.z = float(plane_z)
        self.pub.publish(out)
        self._ok += 1

    def _report(self):
        if self._ok or self._bad:
            bad = "、".join(f"{k}×{v}" for k, v in self._bad.items()) or "無"
            self.get_logger().info(f"最近 5 秒：算出目標位置 {self._ok} 次，略過：{bad}")
        self._ok = 0
        self._bad = {}


def main():
    rclpy.init()
    node = TargetWorld()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
