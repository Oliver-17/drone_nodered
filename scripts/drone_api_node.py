#!/usr/bin/env python3
# =============================================================================
#  drone_api_node.py — 把「起飛／盤旋／降落」開放成三個獨立的 ROS 2 服務
#
#  用法：
#      ros2 launch drone_nodered drone_api.launch.py
#      ros2 service call /MAV1/api/takeoff drone_nodered_interfaces/srv/Takeoff "{altitude: 1.5}"
#      ros2 service call /MAV1/api/hover   std_srvs/srv/Trigger
#      ros2 service call /MAV1/api/land    std_srvs/srv/Trigger
#      ros2 service call /MAV1/api/set_altitude drone_nodered_interfaces/srv/SetAltitude "{altitude: 3.0}"
#      ros2 service call /MAV1/api/patrol drone_nodered_interfaces/srv/Patrol \
#          "{x: [10,10,0,0], y: [0,10,10,0], altitude: 3.0, speed: 2.0, loops: 1}"
#      ros2 service call /MAV1/api/state   std_srvs/srv/Trigger    # 狀態查詢
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

from drone_nodered_interfaces.srv import Patrol, SetAltitude, Takeoff

NAN = float("nan")

# DO_SET_MODE 的參數（Commander.cpp:800 與 px4_custom_mode.h 查證）
MODE_FLAG_CUSTOM = 1.0      # VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED
MAIN_MODE_AUTO = 4.0        # PX4_CUSTOM_MAIN_MODE_AUTO
SUB_MODE_AUTO_LOITER = 3.0  # PX4_CUSTOM_SUB_MODE_AUTO_LOITER

# DO_REPOSITION 的 param2 是 bitmask。Commander.cpp:765-769：
#     第 0 位元沒設 → 直接回 UNSUPPORTED，飛機完全不動。
REPOSITION_CHANGE_MODE = 1.0

# 地球半徑，和 PX4 用同一個值（geo.h:55 CONSTANTS_RADIUS_OF_EARTH）
EARTH_RADIUS_M = 6371000.0

# 巡航的安全上限。打錯字（10 打成 100）在戶外會直接飛出視線。
PATROL_MAX_POINTS = 20
PATROL_MAX_RANGE_M = 50.0
PATROL_MIN_SPEED = 0.5
PATROL_MAX_SPEED = 5.0
# 水平距離小於這個就算到點。PX4 自己的 NAV_ACC_RAD 預設更大，
# 但我們是自己判斷「可以送下一個點了」，抓緊一點路線比較方正。
PATROL_ARRIVE_M = 0.5


def _names(msg_cls, prefix):
    """從訊息類別的常數自動建出「數字 → 名稱」對照表。

    不手寫對照表：PX4 升級時編號可能變，手寫的表會悄悄過期，
    直接從 px4_msgs 讀就永遠跟實際編譯的版本一致。
    """
    return {getattr(msg_cls, k): k[len(prefix):]
            for k in dir(msg_cls) if k.startswith(prefix)}


def _compass(deg):
    """把角度換成方位字。0 度是北、順時針（PX4 的 heading 就是這個定義）。

    +22.5 再除 45：讓「337.5~22.5 度」都落在北，而不是只有剛好 0 度才算北。
    """
    names = ("北", "東北", "東", "東南", "南", "西南", "西", "西北")
    return names[int((deg % 360.0 + 22.5) // 45) % 8]


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
        self.create_service(SetAltitude, "api/set_altitude", self._srv_set_altitude,
                            callback_group=self._srv_group)
        self.create_service(Patrol, "api/patrol", self._srv_patrol,
                            callback_group=self._srv_group)
        self.create_service(Trigger, "api/state", self._srv_state,
                            callback_group=self._srv_group)

        self.create_timer(1.0 / status_rate, self._publish_status,
                          callback_group=self._sub_group)

        # 一次只能有一個「送指令並等 ack」在進行：等 ack 的狀態是共用的，
        # 巡航執行緒和服務回呼同時送的話，後者會把前者等的指令蓋掉。
        self._cmd_lock = threading.Lock()
        self._ack_lock = threading.Lock()
        self._ack_event = threading.Event()
        self._ack_waiting_for = None
        self._ack_result = None

        # 巡航的狀態。_patrol_stop 是「請停下來」的旗標，背景執行緒每一輪都會看它。
        # 用 Event 而不是 bool：等待到點時可以用 wait(timeout)，要取消時立刻醒來。
        self._patrol_thread = None
        self._patrol_stop = threading.Event()
        self._patrol_info = {"active": False, "index": 0, "total": 0}

        self._last_cmd = ""
        self._last_ok = None
        self._last_msg = ""
        self._prev = {}

        self.get_logger().info(
            f"drone_api_node 啟動：PX4 namespace={self.px4_ns} "
            f"target_system={self.target_system} "
            f"起飛高度限制={self.min_takeoff_altitude}~{self.max_takeoff_altitude} m")
        self.get_logger().info(
            f"服務：{self.get_namespace().rstrip('/')}/api/"
            "{takeoff,hover,land,set_altitude,patrol,state}")

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
        # 巡航進度：Node-RED 才看得到「現在飛到第幾個點」
        d["patrol"] = dict(self._patrol_info)
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
        with self._cmd_lock:
            return self._send_locked(command, p1, p2, p3, p4, p5, p6, p7)

    def _send_locked(self, command, p1, p2, p3, p4, p5, p6, p7):
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
                                # :.2f —— float32 存不下剛好 0.2，直接印會變成 0.20000000298023224
                                f"高度 {h:.2f} m 超出範圍 {self.min_takeoff_altitude}~"
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
        # 巡航是背景在送目標點的，不先停掉的話它會繼續把飛機拉去下一個點
        was_patrolling = self._cancel_patrol("使用者按了盤旋")
        if was_patrolling:
            # 巡航中：要清掉 PX4 手上那個還沒飛完的目標點，否則飛機會先飛完再停
            r = self._pause_here()
            if r != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                return self._finish(response, name, False, f"取消巡航被拒絕：{self._ack_text(r)}")
            return self._finish(response, name, True, "已取消巡航，就地盤旋")
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
        if self._cancel_patrol("使用者按了降落"):
            # 先煞停再降落，否則 PX4 會先飛完當下那一段才開始降
            self._pause_here()
        # PX4 對 NAV_LAND 一律接受（Commander.cpp:1093 force=true），降在目前位置。
        r = self._send(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        if r != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            return self._finish(response, name, False, f"降落指令被拒絕：{self._ack_text(r)}")
        return self._finish(response, name, True, "已開始降落")

    # -------------------------------------------------------------------------
    #  巡航
    # -------------------------------------------------------------------------

    def _local_to_global(self, x, y):
        """把本地座標（公尺，北東）換成經緯度。

        照抄 PX4 自己的投影（geo.cpp:90 MapProjection::reproject，方位等距投影），
        不用「一度幾公尺」的近似式 —— 這樣算出來的點和 PX4 內部認定的位置一致，
        誤差不會隨距離累積。參考點是本地原點 ref_lat/ref_lon（＝起飛點）。
        """
        ref_lat = math.radians(float(self.lpos.ref_lat))
        ref_lon = math.radians(float(self.lpos.ref_lon))
        x_rad = float(x) / EARTH_RADIUS_M
        y_rad = float(y) / EARTH_RADIUS_M
        c = math.sqrt(x_rad * x_rad + y_rad * y_rad)
        if c == 0.0:                       # 就是原點，直接回參考點
            return math.degrees(ref_lat), math.degrees(ref_lon)
        sin_c, cos_c = math.sin(c), math.cos(c)
        sin_ref, cos_ref = math.sin(ref_lat), math.cos(ref_lat)
        lat = math.asin(cos_c * sin_ref + (x_rad * sin_c * cos_ref) / c)
        lon = ref_lon + math.atan2(y_rad * sin_c,
                                   c * cos_ref * cos_c - x_rad * sin_ref * sin_c)
        return math.degrees(lat), math.degrees(lon)

    def _cancel_patrol(self, why):
        """請背景的巡航停下來，並等它真的結束。回傳有沒有真的取消到東西。

        一定要等（join）—— 不等的話，舊的巡航可能在新指令送出之後才送出下一個點，
        飛機會突然折返。這是最容易出事的競態。
        """
        t = self._patrol_thread
        if t is None or not t.is_alive():
            return False
        self.get_logger().info(f"[patrol] 取消：{why}")
        self._patrol_stop.set()
        t.join(timeout=5.0)
        return True

    def _pause_here(self):
        """就地煞停。

        ⚠️ 只停掉我們的執行緒是不夠的：最後送出去的那個目標點還留在 PX4 裡，
        它會繼續飛完那一段。2026-09-18 實測：按了盤旋之後飛機仍一路飛到 (10, 10)。
        navigator_main.cpp 的註解寫明「All three set to NaN - pause vehicle」——
        經緯度和高度都填 NaN 的 DO_REPOSITION 就是「取消目標、原地煞停」。
        """
        return self._send(VehicleCommand.VEHICLE_CMD_DO_REPOSITION,
                          p1=-1.0, p2=REPOSITION_CHANGE_MODE)

    def _srv_patrol(self, request, response):
        name = "patrol"
        xs = [float(v) for v in request.x]
        ys = [float(v) for v in request.y]
        alt = float(request.altitude)
        speed = float(request.speed)
        loops = max(1, int(request.loops))
        self.get_logger().info(f"[{name}] 收到指令（{len(xs)} 個點，繞 {loops} 圈）")

        if len(xs) != len(ys) or not xs:
            return self._finish(response, name, False,
                                f"x 和 y 數量要一樣且不能是空的（現在 {len(xs)} / {len(ys)}）")
        if len(xs) > PATROL_MAX_POINTS:
            return self._finish(response, name, False,
                                f"最多 {PATROL_MAX_POINTS} 個點，現在有 {len(xs)}")
        for i, (px, py) in enumerate(zip(xs, ys)):
            if not (math.isfinite(px) and math.isfinite(py)):
                return self._finish(response, name, False, f"第 {i + 1} 個點不是有效數字")
            if abs(px) > PATROL_MAX_RANGE_M or abs(py) > PATROL_MAX_RANGE_M:
                return self._finish(response, name, False,
                                    f"第 {i + 1} 個點 ({px:.1f}, {py:.1f}) 超出 "
                                    f"±{PATROL_MAX_RANGE_M:.0f} m 範圍")
        if speed != 0.0 and not (PATROL_MIN_SPEED <= speed <= PATROL_MAX_SPEED):
            return self._finish(response, name, False,
                                f"速度 {speed:.2f} m/s 超出範圍 "
                                f"{PATROL_MIN_SPEED}~{PATROL_MAX_SPEED} m/s")
        if alt != 0.0 and not (self.min_takeoff_altitude <= alt <= self.max_takeoff_altitude):
            return self._finish(response, name, False,
                                f"高度 {alt:.2f} m 超出範圍 {self.min_takeoff_altitude}~"
                                f"{self.max_takeoff_altitude} m")
        if not self._connected():
            return self._finish(response, name, False, "沒有收到 PX4 的狀態，確認 agent 有連上")
        # navigator_main.cpp:263 要求已解鎖，否則 DO_REPOSITION 不會被執行
        if not self._armed() or self.land.landed:
            return self._finish(response, name, False, "飛機在地上，請先起飛")
        if self.lpos is None or not self.lpos.xy_global or not self.lpos.z_global:
            return self._finish(response, name, False,
                                "沒有有效的經緯度參考（xy_global／z_global 為 false），"
                                "無法把 x/y 換算成 PX4 要的座標")
        if not self.lpos.xy_valid:
            return self._finish(response, name, False, "水平位置估測無效（xy_valid 為 false）")

        # 新的巡航要蓋掉舊的：先確實停掉，不然兩條執行緒會輪流送不同的點
        self._cancel_patrol("收到新的巡航指令")

        # 高度：0 代表維持現在的高度
        height = alt if alt != 0.0 else (self._altitude() or 0.0)
        alt_amsl = float(self.lpos.ref_alt) + height

        self._patrol_stop.clear()
        self._patrol_info = {"active": True, "index": 0, "total": len(xs) * loops}
        self._patrol_thread = threading.Thread(
            target=self._patrol_worker, args=(xs, ys, alt_amsl, height, speed, loops),
            daemon=True)
        self._patrol_thread.start()
        return self._finish(response, name, True,
                            f"已開始巡航：{len(xs)} 個點 × {loops} 圈，高度 {height:.2f} m")

    def _patrol_worker(self, xs, ys, alt_amsl, height, speed, loops):
        """背景依序送點。

        為什麼不在服務裡直接飛完再回覆：飛 10 公尺就要十幾秒，
        rosbridge 的服務逾時是 15 秒（rosbridge.launch.py），等飛完一定逾時。
        所以服務只負責「開始」，進度放在 api/status 裡讓外面看。
        """
        total = len(xs) * loops
        done = 0
        try:
            for lap in range(loops):
                for px, py in zip(xs, ys):
                    if self._patrol_stop.is_set():
                        return
                    lat, lon = self._local_to_global(px, py)
                    r = self._send(VehicleCommand.VEHICLE_CMD_DO_REPOSITION,
                                   # param1 是水平速度，-1 代表用 PX4 預設
                                   p1=(speed if speed != 0.0 else -1.0),
                                   p2=REPOSITION_CHANGE_MODE,
                                   p5=lat, p6=lon, p7=alt_amsl)
                    if r != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                        self._patrol_note(False, f"第 {done + 1} 個點被拒絕：{self._ack_text(r)}")
                        return
                    self.get_logger().info(
                        f"[patrol] 第 {done + 1}/{total} 點：x={px:.1f} y={py:.1f}")

                    # 逾時保護：用距離和速度估一個上限，再加 15 秒起步／煞車的時間。
                    # 沒有這個的話，飛機卡住時巡航會永遠等下去。
                    dist = self._distance_to(px, py)
                    limit = (dist / max(speed, 1.0)) * 3.0 + 15.0
                    if not self._wait_arrival(px, py, limit):
                        if self._patrol_stop.is_set():
                            return
                        self._patrol_note(False, f"飛到第 {done + 1} 個點逾時（等了 {limit:.0f} 秒）")
                        return
                    done += 1
                    self._patrol_info = {"active": True, "index": done, "total": total}
            self._patrol_note(True, f"巡航完成（{total} 個點），停在最後一點盤旋")
        finally:
            self._patrol_info = {"active": False,
                                 "index": done,
                                 "total": total}

    def _distance_to(self, px, py):
        """目前位置到目標點的水平距離。位置無效時回傳 0，讓呼叫者自己決定。"""
        if self.lpos is None or not self.lpos.xy_valid:
            return 0.0
        return math.hypot(px - float(self.lpos.x), py - float(self.lpos.y))

    def _wait_arrival(self, px, py, limit_s):
        """等到接近目標點。回傳 True＝到了，False＝逾時或被取消。"""
        deadline = time.monotonic() + limit_s
        while time.monotonic() < deadline:
            # wait 而不是 sleep：按「盤旋」取消時會立刻醒來，不用等這 0.2 秒過完
            if self._patrol_stop.wait(0.2):
                return False
            if self.lpos is not None and self.lpos.xy_valid \
                    and self._distance_to(px, py) < PATROL_ARRIVE_M:
                return True
        return False

    def _patrol_note(self, ok, message):
        """巡航是背景跑的，沒有服務回應可以用，所以結果寫進狀態和 log。"""
        self._last_cmd, self._last_ok, self._last_msg = "patrol", ok, message
        if ok:
            self.get_logger().info(f"[patrol] {message}")
        else:
            self.get_logger().warn(f"[patrol] {message}")

    def _srv_state(self, request, response):
        """狀態查詢：回報「按下去那一刻」的位置、高度、速度、朝向。

        不送任何指令給 PX4，純粹讀最近收到的訊息，所以任何時候按都安全。

        ⚠️ 每一項都先看有效旗標（xy_valid / v_xy_valid / z_valid）。
           無效卻照印的話，會看到一個很正常的數字而以為飛機知道自己在哪 ——
           2026-09-16 在飛場就是這樣被誤導過（eph 看起來正常，其實根本沒在融合）。
        """
        name = "state"
        if not self._connected():
            return self._finish(response, name, False, "沒有收到 PX4 的狀態，確認 agent 有連上")

        lp = self.lpos
        parts = []
        if lp is not None and lp.xy_valid:
            parts.append(f"位置 x={lp.x:+.2f} y={lp.y:+.2f} m")
        else:
            parts.append("位置 無效")

        h = self._altitude()
        parts.append("高度 無效" if h is None else f"高度 {h:.2f} m")
        # dist_bottom 要有測距儀才有；模擬的 x500 沒有，所以有才顯示
        if lp is not None and lp.dist_bottom_valid:
            parts.append(f"離地 {lp.dist_bottom:.2f} m")

        if lp is not None and lp.v_xy_valid:
            parts.append(f"水平速度 {math.hypot(lp.vx, lp.vy):.2f} m/s")
        else:
            parts.append("水平速度 無效")
        if lp is not None and lp.v_z_valid:
            # vz 是「往下為正」（NED），翻成正的代表上升比較直覺
            parts.append(f"垂直速度 {-lp.vz:+.2f} m/s")

        if lp is not None:
            deg = math.degrees(lp.heading) % 360.0
            parts.append(f"朝向 {deg:.1f}°（{_compass(deg)}）")

        parts.append(f"模式 {self._nav_name()}")
        parts.append("已解鎖" if self._armed() else "未解鎖")
        parts.append("在地面" if self.land.landed else "在空中")

        gp = self.gpos
        if gp is not None and gp.lat_lon_valid:
            parts.append(f"經緯度 {gp.lat:.7f}, {gp.lon:.7f}")

        pt = self._patrol_info
        if pt.get("active"):
            parts.append(f"巡航中 {pt['index']}/{pt['total']} 點")

        return self._finish(response, name, True, " ｜ ".join(parts))

    def _srv_set_altitude(self, request, response):
        """在空中改高度，水平位置不動。

        用 DO_CHANGE_ALTITUDE（186）而不是重下一次起飛指令：
          - navigator_main.cpp:420 的註解寫明它「等同只填高度的 DO_REPOSITION」，
            lat/lon 沿用現有設定點（:458），所以飛機不會水平跑掉。
          - Commander.cpp:782 收到後會切成 AUTO_LOITER，改完就在新高度盤旋，
            跟這裡的「盤旋」行為一致。
        """
        name = "set_altitude"
        h = float(request.altitude)
        self.get_logger().info(f"[{name}] 收到指令（高度 {h:.2f} m）")
        # 檢查順序刻意和 takeoff 一致：先擋不合理的數字，再看飛機狀態。
        # math.isfinite 要先做 —— NaN 和任何數字比較都是 False，只用範圍比較會放它過去。
        if not math.isfinite(h) or not (self.min_takeoff_altitude <= h <= self.max_takeoff_altitude):
            return self._finish(response, name, False,
                                f"高度 {h:.2f} m 超出範圍 {self.min_takeoff_altitude}~"
                                f"{self.max_takeoff_altitude} m")
        if not self._connected():
            return self._finish(response, name, False, "沒有收到 PX4 的狀態，確認 agent 有連上")
        # navigator_main.cpp:416 要求 ARMING_STATE_ARMED，沒解鎖時指令會被吃掉不做事。
        # 與其讓它默默沒反應，不如在這裡就講清楚。
        if not self._armed() or self.land.landed:
            return self._finish(response, name, False, "飛機在地上，請先起飛")

        # param1 是海拔（navigator_main.cpp:424「only supports … absolute altitude amsl」）。
        # ref_alt 是本地原點（起飛點）的海拔，z_global 為 true 才代表它有效。
        # 沒有有效參考時直接拒絕，不用目前高度去猜 —— 猜錯會讓飛機衝到錯的高度。
        if self.lpos is None or not self.lpos.z_global:
            return self._finish(response, name, False,
                                "沒有有效的高度參考（z_global 為 false），無法換算成海拔")
        alt_amsl = float(self.lpos.ref_alt) + h
        now = self._altitude()
        now_note = "目前高度未知" if now is None else f"目前 {now:.2f} m"

        r = self._send(VehicleCommand.VEHICLE_CMD_DO_CHANGE_ALTITUDE, p1=alt_amsl)
        if r != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            return self._finish(response, name, False, f"調整高度被拒絕：{self._ack_text(r)}")
        return self._finish(response, name, True, f"已送出：{now_note} → {h:.2f} m")


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
