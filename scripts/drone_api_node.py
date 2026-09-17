#!/usr/bin/env python3
# =============================================================================
#  drone_api_node.py — 把「起飛／盤旋／降落」開放成三個獨立的 ROS 2 服務
#
#  用法：
#      ros2 launch drone_nodered drone_api.launch.py
#      ros2 service call /MAV1/api/takeoff drone_nodered_interfaces/srv/Takeoff "{altitude: 1.5}"
#      ros2 service call /MAV1/api/hover   std_srvs/srv/Trigger
#      ros2 service call /MAV1/api/land    std_srvs/srv/Trigger
#      ros2 topic echo /MAV1/api/status
#
#  為什麼用 PX4 內建模式（AUTO_TAKEOFF / AUTO_LOITER / AUTO_LAND），不用 offboard：
#      送一次指令，PX4 自己把整件事做完。offboard 要持續每秒送 20 次 setpoint，
#      上游（Node-RED、rosbridge、網路）只要卡一下超過 COM_OF_LOSS_T，
#      就會觸發失效保護。這三個動作不需要冒這個風險。
#
#  為什麼三個服務彼此獨立、不互相呼叫：
#      之後要在 Node-RED 自由組合（起飛 → 等 → 盤旋 → 降落）。
#      服務裡如果偷偷串了下一步，外面就組不出別的順序。
#
#  介面約定（nodered_ros2 的積木只認這個，不認得無人機）：
#      指令  <namespace>/api/<動作>   不需要參數的用 std_srvs/srv/Trigger；
#                                     需要參數的用 drone_nodered_interfaces 裡的格式
#      狀態  <namespace>/api/status   std_msgs/msg/String（內容是 JSON）
# =============================================================================

import json
import math
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (VehicleCommand, VehicleCommandAck, VehicleGlobalPosition,
                          VehicleLandDetected, VehicleLocalPosition, VehicleStatus)
from std_msgs.msg import String
from std_srvs.srv import Trigger

from drone_nodered_interfaces.srv import Takeoff

NAN = float("nan")

# DO_SET_MODE 的參數（Commander.cpp:800 與 px4_custom_mode.h 查證）
MODE_FLAG_CUSTOM = 1.0      # VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED
MAIN_MODE_AUTO = 4.0        # PX4_CUSTOM_MAIN_MODE_AUTO
SUB_MODE_AUTO_LOITER = 3.0  # PX4_CUSTOM_SUB_MODE_AUTO_LOITER


def _names(msg_cls, prefix):
    """從訊息類別的常數自動建出「數字 → 名稱」對照表。

    不手寫對照表：PX4 升級時編號可能變，手寫的表會悄悄過期，
    直接從 px4_msgs 讀就永遠跟實際編譯的版本一致。
    """
    return {getattr(msg_cls, k): k[len(prefix):]
            for k in dir(msg_cls) if k.startswith(prefix)}


NAV_STATE_NAME = _names(VehicleStatus, "NAVIGATION_STATE_")
ACK_RESULT_NAME = _names(VehicleCommandAck, "VEHICLE_CMD_RESULT_")


class DroneApi(Node):
    def __init__(self):
        super().__init__("drone_api_node")

        def p(name, default):
            return self.declare_parameter(name, default).value

        self.px4_ns = p("px4_namespace", "MAV1")
        self.target_system = p("target_system", 1)
        # 起飛高度由每次呼叫帶進來（Takeoff.srv），這裡只管上下限。
        # 下限 0.5 m：太低時 PX4 的起飛判定和地面效應都不穩定。
        # 上限 5 m：模擬用。室內實機要改成 1.5 m 左右（天花板、動捕範圍）。
        self.min_takeoff_altitude = p("min_takeoff_altitude", 0.5)
        self.max_takeoff_altitude = p("max_takeoff_altitude", 5.0)
        self.ack_timeout = p("ack_timeout_s", 3.0)
        # 超過這麼久沒收到 PX4 的狀態，就當作沒連上，拒絕所有指令。
        self.stale_after = p("stale_after_s", 2.0)
        status_rate = p("status_rate_hz", 2.0)

        # ⚠️ 執行緒模型：服務要「等 PX4 的 ack」才回覆。
        #    單執行緒的話，服務回呼在等待時，收 ack 的訂閱回呼永遠排不到 —— 死鎖。
        #    所以：訂閱放可重入群組（隨時可以跑），
        #          服務放互斥群組（一次只處理一個指令，順便避免兩個指令交錯）。
        self._sub_group = ReentrantCallbackGroup()
        self._srv_group = MutuallyExclusiveCallbackGroup()

        # PX4 的 uXRCE-DDS 用 BEST_EFFORT 發布；用預設的 RELIABLE 訂閱會完全收不到，
        # 而且不會報錯。
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        out = f"/{self.px4_ns}/fmu/out"
        # 版本後綴由 .msg 的 MESSAGE_VERSION 決定（非 0 才有 _vN）：
        #   VehicleStatus=1、VehicleLocalPosition=1，其餘為 0
        self.status = self.land = self.lpos = self.gpos = None
        self._t_status = 0.0
        self.create_subscription(VehicleStatus, f"{out}/vehicle_status_v1",
                                 self._on_status, qos, callback_group=self._sub_group)
        self.create_subscription(VehicleLandDetected, f"{out}/vehicle_land_detected",
                                 lambda m: setattr(self, "land", m), qos,
                                 callback_group=self._sub_group)
        self.create_subscription(VehicleLocalPosition, f"{out}/vehicle_local_position_v1",
                                 lambda m: setattr(self, "lpos", m), qos,
                                 callback_group=self._sub_group)
        self.create_subscription(VehicleGlobalPosition, f"{out}/vehicle_global_position",
                                 lambda m: setattr(self, "gpos", m), qos,
                                 callback_group=self._sub_group)
        self.create_subscription(VehicleCommandAck, f"{out}/vehicle_command_ack",
                                 self._on_ack, qos, callback_group=self._sub_group)

        self.cmd_pub = self.create_publisher(
            VehicleCommand, f"/{self.px4_ns}/fmu/in/vehicle_command", 10)
        # 相對名稱：launch 把節點放進 namespace，就會變成 /MAV1/api/status
        self.status_pub = self.create_publisher(String, "api/status", 10)

        self.create_service(Takeoff, "api/takeoff", self._srv_takeoff,
                            callback_group=self._srv_group)
        self.create_service(Trigger, "api/hover", self._srv_hover,
                            callback_group=self._srv_group)
        self.create_service(Trigger, "api/land", self._srv_land,
                            callback_group=self._srv_group)

        self.create_timer(1.0 / status_rate, self._publish_status,
                          callback_group=self._sub_group)

        self._ack_lock = threading.Lock()
        self._ack_event = threading.Event()
        self._ack_waiting_for = None
        self._ack_result = None

        self._last_cmd = ""
        self._last_ok = None
        self._last_msg = ""
        self._prev = {}

        self.get_logger().info(
            f"drone_api_node 啟動：PX4 namespace={self.px4_ns} "
            f"target_system={self.target_system} "
            f"起飛高度限制={self.min_takeoff_altitude}~{self.max_takeoff_altitude} m")
        self.get_logger().info(
            f"服務：{self.get_namespace().rstrip('/')}/api/{{takeoff,hover,land}}")

    # -------------------------------------------------------------------------
    #  狀態
    # -------------------------------------------------------------------------

    def _on_status(self, m):
        self.status = m
        self._t_status = time.monotonic()

    def _connected(self):
        return (self.status is not None and self.land is not None
                and time.monotonic() - self._t_status < self.stale_after)

    def _armed(self):
        return self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def _nav_name(self):
        return NAV_STATE_NAME.get(self.status.nav_state, str(self.status.nav_state))

    def _altitude(self):
        if self.lpos is None or not self.lpos.z_valid:
            return None
        return -self.lpos.z

    def _publish_status(self):
        connected = self._connected()
        d = {"connected": connected}
        if connected:
            d.update({
                "armed": self._armed(),
                "landed": bool(self.land.landed),
                "nav_state": int(self.status.nav_state),
                "mode": self._nav_name(),
                "altitude_m": None if self._altitude() is None else round(self._altitude(), 2),
                "xy_valid": None if self.lpos is None else bool(self.lpos.xy_valid),
            })
            # 狀態改變時印 log —— 事後看 log 才知道「按下按鈕之後飛機經歷了什麼」
            for key in ("armed", "landed", "mode"):
                if self._prev.get(key) != d[key]:
                    if key in self._prev:
                        self.get_logger().info(f"[狀態] {key}: {self._prev[key]} -> {d[key]}")
                    self._prev[key] = d[key]
        d.update({"last_command": self._last_cmd, "last_ok": self._last_ok,
                  "last_message": self._last_msg})
        self.status_pub.publish(String(data=json.dumps(d, ensure_ascii=False)))

    # -------------------------------------------------------------------------
    #  送指令並等 ack
    # -------------------------------------------------------------------------

    def _on_ack(self, m):
        with self._ack_lock:
            if self._ack_waiting_for is not None and m.command == self._ack_waiting_for:
                self._ack_result = m.result
                self._ack_event.set()

    def _send(self, command, p1=NAN, p2=NAN, p3=NAN, p4=NAN, p5=NAN, p6=NAN, p7=NAN):
        """送一個 VehicleCommand，等 PX4 回 ack。回傳 result 碼，逾時回傳 None。

        ⚠️ 沒用到的參數一律填 NaN，不能留預設的 0.0。
           navigator 處理 NAV_TAKEOFF 時（navigator_main.cpp:598 起）：
               if (PX4_ISFINITE(cmd.param5) && PX4_ISFINITE(cmd.param6))
                   rep->current.lat = cmd.param5; ...
           0.0 是有限值，會被當成「起飛到緯度 0、經度 0」。
        """
        m = VehicleCommand()
        m.command = command
        (m.param1, m.param2, m.param3, m.param4,
         m.param5, m.param6, m.param7) = p1, p2, p3, p4, p5, p6, p7
        m.target_system = self.target_system
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        # 少了這個 PX4 會當成內部指令處理（和 arm_and_takeoff.py 同一個坑）
        m.from_external = True
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        with self._ack_lock:
            self._ack_waiting_for = command
            self._ack_result = None
            self._ack_event.clear()
        self.cmd_pub.publish(m)
        got = self._ack_event.wait(self.ack_timeout)
        with self._ack_lock:
            self._ack_waiting_for = None
            return self._ack_result if got else None

    @staticmethod
    def _ack_text(result):
        if result is None:
            return "PX4 沒有回應（逾時）"
        return ACK_RESULT_NAME.get(result, str(result))

    def _finish(self, response, name, ok, message):
        self._last_cmd, self._last_ok, self._last_msg = name, ok, message
        # ⚠️ info 和 warn 一定要寫在「不同行」，不能寫成
        #        (logger.info if ok else logger.warn)(...)
        #    rclpy 的 logger 用「呼叫的位置」記住那一行第一次用的等級，
        #    同一行之後換等級會直接丟 ValueError（rcutils_logger.py:286-299）。
        #    2026-09-17 實測：先被拒絕（warn）再成功（info），節點當場死掉，
        #    而飛機已經起飛了。
        if ok:
            self.get_logger().info(f"[{name}] 成功：{message}")
        else:
            self.get_logger().warn(f"[{name}] 拒絕：{message}")
        response.success = ok
        response.message = message
        return response

    # -------------------------------------------------------------------------
    #  三個服務
    # -------------------------------------------------------------------------

    def _srv_takeoff(self, request, response):
        name = "takeoff"
        h = float(request.altitude)
        self.get_logger().info(f"[{name}] 收到指令（高度 {h:.2f} m）")
        # 先檢查高度：math.isfinite 擋掉 NaN／無限大 ——
        # NaN 跟任何數字比較都是 False，只用範圍比較會讓它直接通過。
        if not math.isfinite(h) or not (self.min_takeoff_altitude <= h <= self.max_takeoff_altitude):
            return self._finish(response, name, False,
                                f"高度 {h} m 超出範圍 {self.min_takeoff_altitude}~"
                                f"{self.max_takeoff_altitude} m")
        if not self._connected():
            return self._finish(response, name, False, "沒有收到 PX4 的狀態，確認 agent 有連上")
        if self._armed() and not self.land.landed:
            return self._finish(response, name, False, "已經在空中")

        # 起飛高度要給「海拔」（takeoff.cpp:184）。有有效的海拔就換算成離地高度；
        # 沒有的話填 NaN，PX4 會改用 MIS_TAKEOFF_ALT（takeoff.cpp:188）。
        # ⚠️ 室內用動捕時通常沒有有效海拔，屆時高度會是 MIS_TAKEOFF_ALT 而不是這個參數。
        if self.gpos is not None and self.gpos.alt_valid:
            alt_amsl = float(self.gpos.alt) + h
            alt_note = f"離地 {h:.2f} m"
        else:
            alt_amsl = NAN
            alt_note = "沒有有效海拔，改用 PX4 參數 MIS_TAKEOFF_ALT"

        # NAV_TAKEOFF 只會切換模式，不會解鎖（Commander.cpp:1064），所以先解鎖。
        if not self._armed():
            r = self._send(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)
            if r != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                return self._finish(response, name, False, f"解鎖失敗：{self._ack_text(r)}")

        r = self._send(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, p7=alt_amsl)
        if r != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            return self._finish(response, name, False, f"起飛指令被拒絕：{self._ack_text(r)}")
        return self._finish(response, name, True, f"已開始起飛（{alt_note}）")

    def _srv_hover(self, request, response):
        name = "hover"
        self.get_logger().info(f"[{name}] 收到指令")
        if not self._connected():
            return self._finish(response, name, False, "沒有收到 PX4 的狀態，確認 agent 有連上")
        if not self._armed() or self.land.landed:
            return self._finish(response, name, False, "飛機在地上，不能盤旋")
        r = self._send(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                       p1=MODE_FLAG_CUSTOM, p2=MAIN_MODE_AUTO, p3=SUB_MODE_AUTO_LOITER)
        if r != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            return self._finish(response, name, False, f"切換盤旋被拒絕：{self._ack_text(r)}")
        return self._finish(response, name, True, "已切換為盤旋（定點定高）")

    def _srv_land(self, request, response):
        name = "land"
        self.get_logger().info(f"[{name}] 收到指令")
        if not self._connected():
            return self._finish(response, name, False, "沒有收到 PX4 的狀態，確認 agent 有連上")
        if not self._armed() or self.land.landed:
            return self._finish(response, name, False, "已經在地上")
        # PX4 對 NAV_LAND 一律接受（Commander.cpp:1093 force=true），降在目前位置。
        r = self._send(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        if r != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            return self._finish(response, name, False, f"降落指令被拒絕：{self._ack_text(r)}")
        return self._finish(response, name, True, "已開始降落")


def main():
    rclpy.init()
    node = DroneApi()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
