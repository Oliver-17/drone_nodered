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
#      ros2 service call /MAV1/api/goto drone_nodered_interfaces/srv/GoTo \
#          "{map_x: 0.0, map_y: 0.0, yaw: .nan, timeout: 0.0}"
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
#  ⚠️ api/goto 是這裡唯一「不走 PX4 內建模式」的動作（2026-10-07 L2 新增）。
#      它把目標轉給 Nav2，Nav2 發 cmd_vel，cmd_vel_to_px4_node 翻成 offboard
#      setpoint。也就是說整個節點裡有兩套互斥的控制路徑：
#
#          takeoff / hover / land / patrol / track  →  PX4 內建模式
#          goto                                     →  Nav2 → offboard
#
#      所以 hover 和 land 一定要先取消進行中的 Nav2 目標，否則 Nav2 會繼續灌
#      setpoint 把飛機拉走（見 _cancel_nav）。
#
#  ⚠️ goto 結束後的控制權交還，模擬和實機是**相反**的，這點驗過：
#      模擬（drone_control/config/px4_bridge.yaml，hold_on_timeout: false）
#          cmd_vel 停 → bridge 銷毀 publisher → PX4 自己掉回 AUTO_LOITER。
#          L1 實測五個目標五次都這樣，交接乾淨，之後 hover/land 直接可用。
#      實機（drone_bringup/config/real/px4_bridge.yaml，hold_on_timeout: true）
#          bridge **不會**銷毀 publisher，它持續送位置 setpoint 把飛機釘在原地，
#          PX4 一直留在 OFFBOARD。那是室內沒有 failsafe 出路時的刻意設計。
#      L2 只保證模擬那一半。實機要加「goto 結束後呼叫 bridge 的 ~/release_hold」，
#      但那件事沒辦法在開發機上驗證，所以留到實際飛場時再做（Oliver 的決定）。
#
#  介面約定（nodered_ros2 的積木只認這個，不認得無人機）：
#      指令  <namespace>/api/<動作>   不需要參數的用 std_srvs/srv/Trigger；
#                                     需要參數的用 drone_nodered_interfaces 裡的格式
#      狀態  <namespace>/api/status   std_msgs/msg/String（內容是 JSON）
# =============================================================================

import collections
import json
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (BatteryStatus, FailsafeFlags, VehicleCommand, VehicleCommandAck,
                          VehicleGlobalPosition, VehicleLandDetected, VehicleLocalPosition,
                          VehicleStatus)
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Trigger

from drone_nodered_interfaces.srv import GoTo, Patrol, SetAltitude, Takeoff, Track

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

# 追蹤：停在目標多遠、多久沒看到就放棄
# 預設跟隨距離（公尺，站在目標後方多遠）。
#
# ⚠️ 2026-10-08：從 2.0 改成 1.3。2.0 是 45° 相機時代的值 ——
#    那時正下方是盲區（45° 俯角在 3 m 高只看得到前方 0.96~9.37 m），
#    所以非得保持距離。換成 67° 之後正下方看得到（-0.22~3.60 m），
#    這個理由消失了。
#
# 為什麼不是 0（正上方）：正上方只有 4.3° 的「機頭上仰」餘裕，而「以 1 m/s²
#    減速」就要傾 5.8° —— 飛機每次煞車都會把目標頂出畫面上緣。
#
# 1.3 m 的來源是**光軸打到地面的位置**：
#        最佳站位 = 高度 / tan(俯角) = 3.0 / tan(67°) = 1.27 m
#    這個距離上目標落在畫面正中央，上仰和下俯各有 27° 餘裕（對稱），
#    側向 ±2.24 m、方位 ±60°。對照表（俯角 67°、高度 3 m）：
#        站位    側向容許  畫面垂直  方位容許  上仰餘裕  下俯餘裕
#        0.00 m  ±1.90 m   +0.82     ±90°      4.3°     50.3°
#        1.27 m  ±2.24 m   +0.00     ±60°     27.3°     27.4°   <- 最佳
#        2.00 m  ±2.43 m   -0.37     ±51°     38.0°     16.6°
#        3.00 m  ±2.70 m   -0.78     ±42°     49.3°      5.3°
#
# 這個常數只是「算不出來時」的退路 —— 正常情況下 distance=0 會由
# camera_pitch_deg 和目前高度自動算（見 _ideal_standoff）。
TRACK_DISTANCE_M = 1.3
TRACK_DURATION_S = 30.0
TRACK_LOST_S = 3.0
# 看不到目標時最多找多久。要夠掃完整整兩圈：2026-09-28 實測掃一圈約 18 秒，
# 只給一圈的時間的話，目標剛好在死角就白白放棄。
TRACK_ACQUIRE_S = 45.0
# 找不到目標時每秒把機頭轉多少度。相機是鎖死的，水平視野只有 ±34°，不轉頭就只能
# 守株待兔（2026-09-28 實測：目標繞一圈 39 秒，等滿 30 秒還差 2 秒才進視野）。
# ⚠️ 實際轉速會打折：每秒都以「當下的機頭」為基準加這個角度，飛機轉不到位時
#    下一次的基準就落後一點。實測命令 20°/秒 只轉出 13°/秒（掃一圈要 28 秒）。
#    所以這裡給 30°，實際約 20°/秒、掃一圈約 18 秒。
#    不能再大：PX4 的 MPC_YAWRAUTO_MAX 預設 45°/s，命令超過飛機轉得動的速度沒有意義。
TRACK_SCAN_STEP_DEG = 30.0
TRACK_YAW_TOL_RAD = math.radians(8.0)   # 機頭差這麼多以內就不用再轉，免得一直微調抖動
# 搜尋時最多繞幾圈還沒看到就放棄。
# ⚠️ 用「圈數」而不是「秒數」：環線 47 m、飛機 2 m/s 理論上 24 秒一圈，
#    但實測 RTF 約 0.57，牆鐘要 42 秒 —— 用秒數當上限會變成「隨負載而異」，
#    同一個設定在不同機器上行為不同。圈數是和 RTF 無關的量。
TRACK_SEARCH_LAPS = 2
# 跟隨時給 DO_REPOSITION 的水平速度（公尺／秒）。
# ⚠️ 不要用 -1.0（PX4 預設）。2026-10-08 實測：車 0.5 m/s 直線走，
#    飛機只跟到 0.12 m/s，一路落後直到超出相機的 3.6 m 取得距離而追丟。
#    要追上並維持，飛機必須明顯比目標快。
TRACK_SPEED_MPS = 2.0
# 目標速度低於這個值就當它沒在動（站位點退回「沿飛機→目標連線」的算法）。
TRACK_MIN_TARGET_SPEED = 0.15

# --- 跟隨階段的速度控制（2026-10-16 從 DO_REPOSITION 改過來）-------------------
#
# ⚠️ 為什麼非改不可：DO_REPOSITION 是「飛到那個點然後停」，目標在動就必定
#    反覆衝刺。2026-10-15 量化過：飛機 100% 的時間落在車正後方、側向只偏 0.27 m
#    （跟得很準），但距離比設定值多 1.25 m —— 除以車速 0.5 m/s 正好是
#    **2.5 秒的反應延遲**。那是架構造成的，不是參數：2026-10-08 試過把更新率
#    拉到 5 Hz、縮小容差、調 PX4 的加速度和加加速度限制，Oliver 實測
#    「暴衝急剎」完全沒有改善，全部還原。
#    速度控制沒有「終點」這個概念，所以不會有停下來再起步的週期。
#
# 跟隨改走 <ns>/cmd_vel → cmd_vel_to_px4_node → offboard，和 Nav2 同一條路。
# ⚠️ 搜尋階段仍然用 DO_REPOSITION（AUTO 模式）—— 那一段是「飛到航點」，
#    本來就該用位置控制，而且已經驗過。兩種模式的切換見 _track_worker。
TRACK_CMD_HZ = 10.0          # cmd_vel 的發送頻率。PX4 要求 offboard setpoint > 2 Hz，
                             # bridge 以 20 Hz 轉發，這裡 10 Hz 有五倍餘裕。
TRACK_KP = 1.2               # 位置誤差（m）→ 速度（m/s）的比例增益。
                             # 誤差 1 m 時給 1.2 m/s，由 TRACK_FOLLOW_SPEED 封頂。
# 跟隨階段的速度上限。**不能用 TRACK_SPEED_MPS（2.0）** ——
# 那是給搜尋段的 DO_REPOSITION 用的，那條路不經過 bridge。
# 跟隨走 cmd_vel，而 cmd_vel_to_px4_node 的 max_speed_xy 會再夾一次：
#       模擬 config/px4_bridge.yaml      1.5
#       實機 config/real/px4_bridge.yaml 0.5（室內刻意壓低）
# 設超過那個值不會比較快，只會讓「設定值」和「實際飛的速度」對不起來。
# 1.4 留一點餘裕，不要剛好壓在上限上。
TRACK_FOLLOW_SPEED = 1.4
TRACK_YAW_KP = 1.5           # 機頭誤差（rad）→ 角速度（rad/s）
TRACK_MAX_YAWRATE = 0.7      # rad/s ≈ 40°/s。PX4 的 MPC_YAWRAUTO_MAX 預設 45°/s，
                             # 命令超過飛機轉得動的速度沒有意義。
# 從「跟隨」切回「搜尋」時要等多久才發 DO_REPOSITION。
# ⚠️ cmd_vel_to_px4_node 的 ensureOffboard() 只要 cmd_timeout_s（預設 0.5 秒）
#    內收到過 cmd_vel，就會主動把 PX4 拉回 OFFBOARD。沒等就送 DO_REPOSITION
#    的話，AUTO 和 OFFBOARD 會互搶 —— 而且不會有任何錯誤訊息。
#    這和 _settle_after_cancel 處理 nav 的理由完全一樣。
TRACK_MODE_SWITCH_S = 0.8
# 站位點離飛機超過這麼遠才重送指令。太小會每秒都在送、讓 navigator 抖動。
# ⚠️ 2026-10-08 試過 0.1 想讓指令連續一點，實測對晃動沒有改善（見
#    TRACK_UPDATE_S 的說明），已還原。
# ⚠️ 2026-10-16 起**沒有人在用**。跟隨改成速度控制之後不再需要「到位就不送」
#    這個閘門（每一拍都送 cmd_vel，誤差小時比例控制自然給小速度）。
#    留著是為了讓下面那段歷史註解讀得懂，要調跟隨行為請看 TRACK_KP。
TRACK_GOAL_TOL_M = 0.3
# 估目標速度用多長的時間窗。太短會被偵測雜訊放大，太長會跟不上轉向。
TRACK_VEL_WINDOW_S = 1.0
# 多久更新一次目標點。
#
# ⚠️ 2026-10-08 試過 0.2（5 Hz）＋ TRACK_GOAL_TOL_M 0.1 ＋ PX4 的
#    MPC_ACC_HOR 2.0 / MPC_JERK_AUTO 2.0，Oliver 實測「暴衝急剎」**沒有改善**，
#    全部還原。結論：問題不在調參，而在「用位置設定點追移動目標」這個架構 ——
#    每一道 DO_REPOSITION 都是「衝到那個點然後停」，目標在動就必定反覆衝刺。
#    要治本得改成速度控制（見檔頭的「追蹤架構」說明）。
# ⚠️ 2026-10-16 拆成兩個節拍。跟隨改成速度控制之後，兩段的需求完全相反：
#      跟隨  要高頻 —— cmd_vel 斷流超過 bridge 的 cmd_timeout_s（0.5 秒）
#                      就會被收回控制權
#      搜尋  要低頻 —— 每一拍是一道完整的 DO_REPOSITION，而且原地掃描是
#                      「每拍把機頭轉 TRACK_SCAN_STEP_DEG 度」設計的；
#                      照 10 Hz 跑會變成每秒轉 300°
TRACK_UPDATE_S = 1.0 / TRACK_CMD_HZ   # 迴圈節拍（0.1 秒）
TRACK_SEARCH_TICK_S = 1.0             # 搜尋段多久才真正動一次

# --- 鎖定交接的保險（2026-10-19 加，起因是撞貨架摔機）------------------------
#
# 🔴 那次摔機的完整過程（從 PX4 的飛行記錄 02_46_33.ulg 還原）：
#      66.6 s  鎖定 → OFFBOARD 接手。那一刻飛機正以 **3.2 m/s 往北**飛
#              （搜尋段的 DO_REPOSITION，PX4 自己算到 4.40 m/s），
#              第一則 cmd_vel 是 1.40 → PX4 抬頭 **22.7°** 硬煞。
#      67.6 s  站位點算在飛機西邊 → 指令變成**滿速 1.40 m/s 往西**，
#              連續 1.9 秒，從 map x=+5.32 一路推到 +3.78。
#      68.6 s  撞上第 2 排貨架的東面（x=+3.6），翻覆，從 5 m 高墜地。
#
#    為什麼站位點會跑到西邊：站位點 = 車的位置 − v̂方向 × dist。
#    鎖定那一刻 _target_hist 只有兩三筆、間隔 0.2~0.3 秒，而且正好落在那個
#    22.7° 的姿態劇變裡 —— v̂ 的方向是純雜訊。它指東，站位點就落在車西邊
#    dist 公尺（5 m 高時 dist = 5/tan67° = 2.12 m），剛好是貨架。
#
# 下面幾個常數各擋一段。真正防摔的是禁飛方框（見 _clamp_no_fly），
# 這幾個是讓「算錯」不要那麼容易發生。
TRACK_VEL_MIN_WINDOW_S = 0.6   # 估車速的時間窗至少要這麼長才採用。
                               # 原本的門檻是 0.2 秒 —— 太鬆，正好放過上面那次。
TRACK_VEL_MIN_SAMPLES = 4      # 而且窗內至少要這麼多筆。偵測約 15 Hz，
                               # 0.6 秒會有 9 筆左右，4 筆是很寬鬆的下限。
                               # 不足就回 (0,0) → 站位點退回「沿視線前後」，
                               # 那個算法只會朝車或離開車，絕對不會橫移進貨架。
TRACK_HANDOFF_RAMP_S = 1.0     # 接手後速度上限從 0 線性拉到 TRACK_FOLLOW_SPEED
                               # 要花多久。治的是上面那個「抬頭 22.7° 硬煞」。
                               # ⚠️ 這一項同時也在幫 TRACK_VEL_MIN_WINDOW_S：
                               #    硬煞造成的姿態劇變本身就是 v̂ 變成雜訊的原因
                               #    （target_world_node 用最新姿態配舊影像），
                               #    不硬煞，前 0.6 秒的偵測資料才乾淨。
# --- 站位方向的平滑（2026-10-19 加，治「左右搖擺」和「鎖定時抽動」）----------
#
# 🔴 站位點 = 車的位置 − v̂方向 × dist，所以它**剛性地掛在估出來的車行方向上**。
#    前後那一軸被 dist 鎖住（所以前進指令很穩），橫向那一軸沒有任何東西鎖住
#    —— 方向一變，站位點就繞著車瞬移，最多 2×dist。
#
#    2026-10-19 從節點自己的 log 量到的（車是**直線**往北走，真實方向 0°）：
#        車向 -7  → 左-0.55        車向 +9  → 左+0.35
#        車向 -11 → 左-0.59        車向 +12 → 左+0.76
#        車向 -12 → 左-0.70        車向 +14 → 左+0.75
#    方向每半秒在 ±13° 之間翻邊，橫向速度指令一比一跟著翻。
#    幅度對帳：2.12 × sin(13°) × 1.2 = 0.57 m/s，量到的峰值 0.70~0.76 —— 吻合。
#    而機頭差一直在 ±5° 內、轉速大多是 0 → **機頭完全沒在抖**，不是 yaw 的問題。
#
#    同一個機制還造成兩件事：
#      鎖定時抽動  車速在 TRACK_MIN_TARGET_SPEED 門檻上下抖，站位點就在
#                  「沿車向」和「沿視線」兩套算法之間跳。log 量到半秒內
#                  站位點從 北+0.4 跳到 北+4.3（3.9 m）再跳回來。
#      轉角抽動    車真的轉 83°，站位點跟著瞬移，差距從 0.42 m 跳到 2.01 m，
#                  指令直接飽和。
TRACK_DIR_TAU_S = 1.5          # 站位方向的低通時間常數（秒）。
                               # 1.5 秒的依據：要壓掉 1 秒週期的翻邊（量到的
                               # 就是每半秒換邊），又不能慢到跟不上車真正的轉彎
                               # （環線 47 m、車 0.5 m/s，一圈 94 秒、四個轉角，
                               # 每個轉角之間有 20 秒以上，1.5 秒遠遠跟得上）。
TRACK_LOG_S = 0.5              # 跟隨時多久印一行診斷。
                               # ⚠️ 2026-10-19 查這兩個問題要去解析 PX4 的
                               #    二進位飛行記錄（.ulg）才看得出來，很花時間。
                               #    該印出來的數字就印出來。


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
        # 看門狗：狀況不對就自動降落。預設開 —— 誤觸發最多是提早降落，漏掉才危險。
        self.watchdog_enabled = p("watchdog_enabled", True)
        self.battery_min = p("battery_min", 0.30)          # 剩餘電量低於這個比例就降落
        self.watchdog_debounce = p("watchdog_debounce_s", 1.0)  # 連續成立這麼久才動作
        self.altitude_margin = p("altitude_margin_m", 1.0)      # 超過上限這麼多就算失控爬升
        # 超過這麼久沒收到 PX4 的狀態，就當作沒連上，拒絕所有指令。
        self.stale_after = p("stale_after_s", 2.0)
        # ⚠️ 2026-10-20：2.0 → 5.0。儀表板加了即時地圖之後 2 Hz 太慢 ——
        #    飛機 1.2 m/s 時每次更新位置跳 0.6 m，看起來一格一格的。
        #    這包 JSON 很小、走本機 websocket，5 Hz 的成本可以忽略。
        status_rate = p("status_rate_hz", 5.0)
        # goto 的預設逾時。這是節點自己的保險，不是 Nav2 的 ——
        # Nav2 的行為樹卡住時可能很久都不回 result（復原行為會一直重試）。
        # 120 秒的依據：倉庫對角約 40 m，max_vel 0.8 m/s，繞貨架算 1.6 倍 → 約 80 秒。
        self.goto_timeout = p("goto_timeout_s", 120.0)
        # Nav2 的 action 名稱。它**不帶 namespace** —— nav2.launch.py 是用
        # 全域名稱開那些伺服器的（參數檔裡的 topic 才帶 MAV1）。
        self.nav_action = p("nav_action", "/navigate_to_pose")
        # PX4 本地原點（＝起飛點）在 map 座標系的哪裡，格式 "x,y"。
        # 給 api/track 的 search_x/search_y 做 map → NED 換算用。
        #
        # ⚠️ 這個值必須和 px4_bridge 的 odom_origin **完全相同** ——
        #    兩者描述的是同一件事（PX4 的原點是「生成／起飛位置」而不是地圖原點）。
        #    不一致的話搜尋航點會整組平移，而且不會報錯。
        #    warehouse_all.launch.py 把同一個字串餵給兩邊，所以不會錯；
        #    手動啟動時要自己對齊。
        self.local_origin_map = p("local_origin_map", "")
        # 相機的安裝俯角（度，0 = 水平、90 = 正下方）。只用來算「最佳跟隨距離」：
        #     最佳站位 = 高度 / tan(俯角)   ← 光軸打到地面的位置，目標落在畫面正中
        # 俯角 90（正下方）→ 0 公尺（正上方），和直覺一致。
        # ⚠️ 要和 vision.launch.py 的 camera_pitch 一致。那邊是依機體自動選的
        #    （x500_cam_down → 67、x500_depth_tilt → 45），所以換機體時這裡也要換。
        self.camera_pitch_deg = p("camera_pitch_deg", 67.0)
        # 取消 Nav2 之後要等多久才去切 PX4 模式。必須大於 bridge 的
        # cmd_timeout_s（預設 0.5 秒），否則 ensureOffboard() 會把模式搶回去
        # —— 理由和實測證據見 _settle_after_cancel。
        self.cancel_settle_s = p("cancel_settle_s", 0.8)
        # --- 跟隨段的禁飛方框 ----------------------------------------------
        # ⚠️ api/track 的**跟隨段**是 cmd_vel 直通 offboard → 不經過 Nav2，
        #    所以 costmap、膨脹層、避障在那一段完全不存在。
        #    整個跟隨迴圈原本唯一的位置保險是 PATROL_MAX_RANGE_M（50 m），
        #    在 36×24 m 的倉庫裡等於沒有 —— 2026-10-19 因此撞上貨架摔機。
        #
        # 方框是 map 座標攤平的一串，每 4 個一組 (x_min, y_min, x_max, y_max)，
        # 由 warehouse_all.launch.py 從 config/warehouse_geometry.yaml 帶入。
        # 空的 = 沒有保險（啟動時會明確印出來）。
        self.no_fly_margin = p("no_fly_margin_m", 0.8)
        self.no_fly_lookahead_s = p("no_fly_lookahead_s", 1.0)
        # 直接在這裡就換算成 PX4 本地 NED 並把餘裕膨脹進去 ——
        # 跟隨迴圈是 10 Hz，不要每一拍重算一次不會變的東西。
        # ⚠️ _parse_no_fly 會順便填 self._scene_boxes（map 座標的原始方框，
        #    給儀表板的地圖畫）。先給一個空的，剖析失敗時才不會是未定義。
        self._scene_boxes = []
        self._no_fly = self._parse_no_fly(p("no_fly_boxes", ""))

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
        # 看門狗要看的兩個來源
        self.battery = self.failsafe = None
        self.create_subscription(BatteryStatus, f"{out}/battery_status_v1",
                                 lambda m: setattr(self, "battery", m), qos,
                                 callback_group=self._sub_group)
        self.create_subscription(FailsafeFlags, f"{out}/failsafe_flags",
                                 lambda m: setattr(self, "failsafe", m), qos,
                                 callback_group=self._sub_group)

        self.cmd_pub = self.create_publisher(
            VehicleCommand, f"/{self.px4_ns}/fmu/in/vehicle_command", 10)
        # 相對名稱：launch 把節點放進 namespace，就會變成 /MAV1/api/status
        self.status_pub = self.create_publisher(String, "api/status", 10)

        # 視覺目標的位置（target_world_node 發的，PX4 本地 NED）。
        # 相對名稱 → 節點在 namespace 裡就會變成 /MAV1/api/target
        self.target = None
        self._t_target = 0.0
        self.create_subscription(PointStamped, "api/target", self._on_target, 10)

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
        self.create_service(GoTo, "api/goto", self._srv_goto,
                            callback_group=self._srv_group)
        self.create_service(Track, "api/track", self._srv_track,
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
        self._track_thread = None
        self._track_stop = threading.Event()
        # 目標位置的歷史，用來估它的速度。maxlen 20 在偵測約 10~30 Hz 下
        # 涵蓋 0.7~2 秒，夠蓋住 TRACK_VEL_WINDOW_S。
        self._target_hist = collections.deque(maxlen=20)
        # want/locked_s/reacquire 是 2026-10-12 加的，給畫面分辨「在找」和
        # 「追到了而且穩住了」：
        #   want      要保持的距離（distance=0 時是自動算出來的，看不到它
        #             就沒辦法判斷實際的 distance 算不算準）
        #   locked_s  鎖定後過了多久（＝「保持 N 秒」）
        #   reacquire 這一趟曾經鎖定過 → 現在在找就是「跟丟回頭找」而不是冷啟動
        self._track_info = {"active": False, "distance": None, "lost_s": None,
                            "searching": False, "search_s": None,
                            "phase": "idle", "search_index": None,
                            "search_laps": None,
                            "want": None, "locked_s": None, "reacquire": False}

        # --- Nav2 導航（goto）---------------------------------------------
        # ActionClient 放可重入群組：send_goal_async 的回呼要能在服務回呼
        # 還沒返回時就被執行緒池排到，否則就是死鎖（和上面訂閱同一個理由）。
        self._nav_client = ActionClient(self, NavigateToPose, self.nav_action,
                                        callback_group=self._sub_group)
        # goal handle 要留著才能取消。用鎖保護：服務回呼（送出）和
        # action 回呼（收結果）在不同執行緒。
        self._nav_lock = threading.Lock()
        self._nav_handle = None
        self._nav_seq = 0
        self._nav_info = {"state": "idle", "goal": None, "seq": 0,
                          "distance_remaining": None, "message": ""}
        # cmd_vel 的完整話題名（就緒燈要數它有沒有訂閱者＝橋在不在）。
        # 在這裡解析一次而不是每次發狀態都解析：namespace 在節點建好之後
        # 就不會變了，而 resolve_topic_name 每次都要走一遍 rcl。
        self._cmd_vel_topic = self.resolve_topic_name("cmd_vel")
        # 跟隨階段用它驅動飛機（見 TRACK_CMD_HZ 那段）。
        # ⚠️ 和 Nav2 共用同一個 topic 和同一個下游節點 —— 兩者絕不能同時在發，
        #    互斥靠 _cancel_all_motion 這個唯一的節流點。
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # 請 bridge 放手用的。跟隨（cmd_vel/offboard）切回搜尋（DO_REPOSITION/AUTO）
        # 時要先叫它交還控制權 —— 見 _release_bridge_hold。
        # 放在可重入群組：追蹤執行緒呼叫它時，服務回呼不能和互斥群組互鎖。
        self._release_cli = self.create_client(
            Trigger, "cmd_vel_to_px4_node/release_hold",
            callback_group=self._sub_group)

        # 任務狀態：Node-RED 的「等待完成」積木就是看這個。
        # seq 每接受一個新指令就 +1 —— 積木靠它分辨「這次」和「上一次」的完成。
        self._task = {"seq": 0, "name": "", "state": "idle", "message": ""}
        self._task_deadline = 0.0
        self._task_target = None        # 起飛／調整高度的目標高度
        self._wd = {"tripped": False, "reason": ""}
        self._wd_since = {}             # 每個異常條件「第一次成立」的時間

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
            "{takeoff,hover,land,set_altitude,patrol,state,track,goto}")
        self.get_logger().info(
            f"api/track：相機俯角 {self.camera_pitch_deg:.0f}°，"
            f"distance=0 會自動算成「高度/tan(俯角)」"
            f"（3 m 高 → {self._ideal_standoff(3.0) or float('nan'):.2f} m）")
        self.get_logger().info(
            "api/track 的搜尋環線："
            + (f"map 座標，本地原點在 map({self.local_origin_map})"
               if self.local_origin_map
               else "local_origin_map 未設定 → 只能用原地轉頭掃描"))
        # ⚠️ 這一行一定要印。跟隨段不經過 Nav2，沒有方框就是「完全沒有避障」，
        #    而那件事從外面看不出來 —— 飛機照樣起飛、照樣追、照樣回報成功，
        #    直到某一次站位點算錯就直接撞上去（2026-10-19 摔過一次）。
        if self._no_fly:
            self.get_logger().info(
                f"api/track 跟隨段的禁飛方框：{len(self._no_fly)} 個"
                f"（餘裕 {self.no_fly_margin:.2f} m、預判 "
                f"{self.no_fly_lookahead_s:.1f} 秒）。"
                "只擋靜態障礙物，不是避障系統")
        else:
            self.get_logger().warn(
                "api/track 跟隨段**沒有禁飛方框** —— 那一段是 cmd_vel 直通 "
                "offboard、不經過 Nav2，所以沒有任何避障。"
                "倉庫場景請用 warehouse_all.launch.py 啟動（它會自動帶入）")
        self.get_logger().info(
            f"goto 用的 Nav2 action：{self.nav_action}"
            f"（逾時 {self.goto_timeout:.0f} 秒）。"
            "座標是 map 座標系，和 api/patrol 的「離起飛點 NED」不同")

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

    def _ready(self):
        """「現在可以用了嗎」—— 給 Node-RED 的就緒燈用（api/status 的 ready 欄位）。

        為什麼在節點裡算而不是讓 Node-RED 自己判斷：Nav2 的 action server 和
        cmd_vel 的橋**只有這個節點看得到**（Node-RED 只收得到 api/status 這串
        JSON）。四項都是「不就緒的話指令會被拒、或是白按」的必要條件。

        ⚠️ 這裡**不檢查視覺偵測**（Oliver 2026-10-12 決定）：偵測節點在不在
        和「現在看得到車」是兩回事，放進就緒燈會讓人以為燈亮了就一定追得到。
        追蹤看不看得到目標，看的是 track 那一組欄位。
        """
        connected = self._connected()
        # xy_valid：連盤旋都需要水平位置估計，所以它和「連上 PX4」一樣是底線。
        pos = connected and self.lpos is not None and bool(self.lpos.xy_valid)
        # server_is_ready() 不阻塞（會阻塞的是 wait_for_server），所以放在
        # 2 Hz 的狀態迴圈裡是安全的。
        nav2 = self._nav_client.server_is_ready()
        # cmd_vel 的訂閱者就是 cmd_vel_to_px4_node。少了它最陰險：Nav2 規劃
        # 成功、狀態顯示「導航中」，但沒有人把 cmd_vel 翻成 offboard setpoint，
        # 飛機從頭到尾不動，而且**不會有任何錯誤訊息**。
        bridge = self.count_subscribers(self._cmd_vel_topic) > 0
        r = {"px4": connected, "position": pos, "nav2": nav2, "bridge": bridge}
        r["all"] = all(r.values())
        return r

    def _publish_status(self):
        self._check_task()
        self._watchdog()
        # 降落完成後解除看門狗，這樣下一趟飛行才會重新監看
        if self._wd["tripped"] and self.land is not None and self.land.landed and not self._armed():
            self._wd = {"tripped": False, "reason": ""}
            self._wd_since.clear()
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
            # 儀表板的地圖和速度表（2026-10-20）。
            # 位置給 **map 座標**而不是 PX4 的本地 NED —— 前端就不用知道
            # 「原點在哪、哪個軸是北」，而那正是這個專案最常踩的雷。
            sp = self._ground_speed()
            d["speed_mps"] = None if sp is None else round(sp, 2)
            pose = self._local_to_map()
            if pose is not None:
                d["map_x"], d["map_y"] = round(pose[0], 2), round(pose[1], 2)
                d["map_heading_deg"] = round(pose[2], 1)
            # 狀態改變時印 log —— 事後看 log 才知道「按下按鈕之後飛機經歷了什麼」
            for key in ("armed", "landed", "mode"):
                if self._prev.get(key) != d[key]:
                    if key in self._prev:
                        self.get_logger().info(f"[狀態] {key}: {self._prev[key]} -> {d[key]}")
                    self._prev[key] = d[key]
        # 巡航進度：Node-RED 才看得到「現在飛到第幾個點」
        d["patrol"] = dict(self._patrol_info)
        d["track"] = dict(self._track_info)
        # 導航進度：Node-RED 的進度條看 distance_remaining（Nav2 的 feedback）
        d["nav"] = dict(self._nav_info)
        # 就緒燈：儀表板靠這個顯示「可以用了嗎」（見 _ready 的說明）
        d["ready"] = self._ready()
        d["task"] = dict(self._task)
        d["watchdog"] = dict(self._wd)
        # 場地幾何（map 座標的方框，含貨架和外牆）。前端用它畫底圖、
        # 並從所有方框的聯集算出畫布範圍。
        # ⚠️ 每一拍都送（約 300 bytes）而不是只送一次：rosbridge 的訂閱者
        #    隨時可能重連（重新整理瀏覽器就會），只送一次的話重連後地圖是空的，
        #    而且不會有任何錯誤訊息。走本機 websocket，這個量可以忽略。
        d["scene"] = {"boxes": self._scene_boxes}
        d.update({"last_command": self._last_cmd, "last_ok": self._last_ok,
                  "last_message": self._last_msg})
        self.status_pub.publish(String(data=json.dumps(d, ensure_ascii=False)))

    # -------------------------------------------------------------------------
    #  任務狀態（Node-RED 的「等待完成」積木就是看這個）
    # -------------------------------------------------------------------------

    def _begin_task(self, name, timeout_s, target=None, done_now=False):
        """開始一個新任務。seq +1 是關鍵 —— 外面靠它分辨這次和上一次。"""
        self._task = {"seq": self._task["seq"] + 1, "name": name,
                      "state": "done" if done_now else "running", "message": ""}
        self._task_target = target
        self._task_deadline = time.monotonic() + timeout_s

    def _end_task(self, ok, message=""):
        if self._task["state"] == "running":
            self._task = dict(self._task, state="done" if ok else "failed", message=message)
            self.get_logger().info(f"[任務] {self._task['name']} → {self._task['state']}"
                                   f"{'：' + message if message else ''}")

    def _check_task(self):
        """每次發狀態時檢查目前任務做完了沒。"""
        t = self._task
        if t["state"] != "running":
            return
        name, alt = t["name"], self._altitude()

        if name in ("takeoff", "set_altitude"):
            # ⚠️ 不能用「模式變成 LOITER」當完成 —— PX4 的 NAV_MC_ALT_RAD 預設 0.8 m，
            #    離目標還有 0.8 m 就會切 LOITER，之後才慢慢爬完。只能看高度。
            if alt is not None and self._task_target is not None \
                    and abs(alt - self._task_target) < 0.2:
                self._end_task(True, f"已到 {alt:.2f} m")
        elif name == "land":
            if self.land is not None and self.land.landed and not self._armed():
                self._end_task(True, "已落地上鎖")
        # patrol 由背景執行緒自己收尾（_patrol_note），這裡只管逾時
        # goto 由 Nav2 的 action result 收尾（_nav_result），這裡也只管逾時

        if self._task["state"] == "running" and time.monotonic() > self._task_deadline:
            # ⚠️ goto 逾時一定要連 Nav2 的 goal 一起取消。只把任務標成失敗的話
            #    Nav2 會繼續飛到底，飛機在動而 Node-RED 顯示「失敗」—— 最糟的組合。
            if name == "goto":
                self._cancel_nav("節點端逾時")
            self._end_task(False, "逾時")

    # -------------------------------------------------------------------------
    #  看門狗：狀況不對就自動降落
    # -------------------------------------------------------------------------

    def _watchdog(self):
        """每次發狀態時檢查一輪。只在「空中且已解鎖」時作用 —— 地上沒什麼好降的。"""
        if not self.watchdog_enabled or self._wd["tripped"]:
            return
        if not self._connected() or not self._armed() or self.land.landed:
            self._wd_since.clear()
            return

        bad = {}
        # 位置估計失效：連盤旋都需要位置，所以只能降落
        if self.failsafe is not None and self.failsafe.local_position_invalid:
            bad["位置估計失效"] = "PX4 回報 local_position_invalid"
        if self.lpos is not None and not self.lpos.xy_valid:
            bad["水平位置無效"] = "xy_valid = false"
        # remaining 是 0~1，-1 代表沒有有效值（BatteryStatus.msg:17）
        if self.battery is not None and 0.0 <= self.battery.remaining < self.battery_min:
            bad["電量過低"] = f"剩餘 {self.battery.remaining * 100:.0f}%"
        alt = self._altitude()
        limit = self.max_takeoff_altitude + self.altitude_margin
        if alt is not None and alt > limit:
            bad["高度超過上限"] = f"{alt:.2f} m > {limit:.2f} m"

        now = time.monotonic()
        for k in list(self._wd_since):
            if k not in bad:
                del self._wd_since[k]      # 條件不再成立就重新計時，避免偶發雜訊累積
        for k, why in bad.items():
            self._wd_since.setdefault(k, now)
            # 防誤觸發：要連續成立一段時間。起飛瞬間 xy_valid 可能閃一下。
            if now - self._wd_since[k] >= self.watchdog_debounce:
                self._trip(f"{k}（{why}）")
                return

    def _trip(self, reason):
        self._wd = {"tripped": True, "reason": reason}
        self.get_logger().warn(f"[看門狗] {reason} → 自動降落")
        # 開執行緒去送指令：送指令要等 ack（最多 3 秒），不能卡住發狀態的計時器
        threading.Thread(target=self._emergency_land, args=(reason,), daemon=True).start()

    def _emergency_land(self, reason):
        # ⚠️ 2026-10-07：原本漏了 _cancel_nav。看門狗在導航中觸發時，Nav2 會
        #    繼續發 cmd_vel、bridge 繼續把 PX4 要回 OFFBOARD，和 AUTO_LAND 互搶，
        #    而 nav.state 會一直停在 running —— Node-RED 顯示「導航中」而飛機在降落。
        #    這是安全路徑，不能有這種洞。
        self._settle_after_cancel(self._cancel_all_motion(reason))
        r = self._send(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        ok = r == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        self._end_task(False, f"看門狗：{reason}")
        self._last_cmd, self._last_ok = "watchdog_land", ok
        self._last_msg = f"{reason} → 降落{'已送出' if ok else '失敗：' + self._ack_text(r)}"
        if not ok:
            self.get_logger().error(f"[看門狗] 降落指令被拒絕：{self._ack_text(r)}")

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

        # 起飛只在落地時才允許（上面已擋），而 patrol/track/nav 都要求飛在空中，
        # 所以理論上這裡沒有東西要取消。接上節流點是防禦性的 ——
        # 「看門狗降落了但背景執行緒還活著」這類狀況出現過，而節流點的意義
        # 就是不用再逐一推理哪個 handler 需要它。
        self._settle_after_cancel(self._cancel_all_motion(f"改用 {name}"))

        # 🔴 在地上還停在 OFFBOARD 的話，先切回 AUTO_LOITER 再解鎖。
        #
        # 症狀：追蹤（或任何走 cmd_vel 的動作）→ 降落 → 想再飛一次，
        #       api/takeoff 回「解鎖失敗：TEMPORARILY_REJECTED」，
        #       而且 goto/track 跟著全部拒絕（它們都要求「已經在空中」）。
        #       儀表板上看起來就是「整組指令都失效了」。
        #
        # 根因：PX4 在 AUTO_LAND 落地上鎖的那一刻，會把 nav_state 退回
        #       「上一個被明確指定的模式」—— 而追蹤期間 bridge 的
        #       ensureOffboard() 送過 DO_SET_MODE OFFBOARD，所以退回 OFFBOARD。
        #       2026-10-19 的 log（12:50:21.661 同一毫秒）：
        #           [狀態] armed: True -> False
        #           [狀態] mode: AUTO_LAND -> OFFBOARD
        #       而「在 OFFBOARD 解鎖」要求有人正在串流 offboard setpoint，
        #       那時候 bridge 早就銷毀 publisher 交還控制權了（12:50:10.161），
        #       所以 PX4 拒絕解鎖 —— 而且回的是 TEMPORARILY_REJECTED，
        #       看起來像「等一下再試就好」，其實等多久都一樣。
        #
        # ⚠️ 這和 bridge 沒放手無關，bridge 有正常放手。是 PX4 自己的模式殘留。
        # ⚠️ 失敗不當致命錯誤：切不過去的話下面的解鎖本來就會失敗並報出原因，
        #    在這裡提早 return 只會把真正的錯誤訊息藏起來。
        if not self._armed() and self.status.nav_state == \
                VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info(
                "[takeoff] 飛機在地上但模式還停在 OFFBOARD（降落上鎖時 PX4 退回的）"
                " → 先切 AUTO_LOITER，否則解鎖會被拒絕")
            r = self._send(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                           p1=MODE_FLAG_CUSTOM, p2=MAIN_MODE_AUTO,
                           p3=SUB_MODE_AUTO_LOITER)
            if r != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                self.get_logger().warn(
                    f"[takeoff] 切 AUTO_LOITER 沒被接受（{self._ack_text(r)}），"
                    "還是繼續試解鎖")

        # NAV_TAKEOFF 只會切換模式，不會解鎖（Commander.cpp:1064），所以先解鎖。
        if not self._armed():
            r = self._send(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)
            if r != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                return self._finish(
                    response, name, False,
                    f"解鎖失敗：{self._ack_text(r)}（目前模式 {self._nav_name()}）")

        r = self._send(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, p7=alt_amsl)
        if r != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            return self._finish(response, name, False, f"起飛指令被拒絕：{self._ack_text(r)}")
        # 60 秒：爬 5 m（上限）就算慢也夠了；逾時會讓「等待完成」走失敗出口
        self._begin_task(name, 60.0, target=h)
        return self._finish(response, name, True, f"已開始起飛（{alt_note}）")

    def _srv_hover(self, request, response):
        name = "hover"
        self.get_logger().info(f"[{name}] 收到指令")
        if not self._connected():
            return self._finish(response, name, False, "沒有收到 PX4 的狀態，確認 agent 有連上")
        if not self._armed() or self.land.landed:
            return self._finish(response, name, False, "飛機在地上，不能盤旋")
        # 任何正在驅動飛機的動作都要先停掉，否則它會繼續把飛機拉走
        canceled = self._cancel_all_motion("使用者按了盤旋")
        if canceled:
            self._end_task(False, "被盤旋指令取消")
        # patrol/track 要清掉 PX4 手上的目標點；nav 要等過 bridge 的逾時，
        # 否則 ensureOffboard() 會把下面要設的 AUTO_LOITER 推翻（見 _settle_after_cancel）
        self._settle_after_cancel(canceled)
        # ⚠️ 不管有沒有取消到東西，都要明確送一次 AUTO_LOITER。
        #    取消 Nav2 之後 PX4 會不會自己掉回 LOITER，模擬和實機是相反的
        #    （模擬的 bridge 銷毀 publisher 觸發 failsafe；實機的 bridge 原地保持
        #    讓 PX4 留在 OFFBOARD）。不要依賴那個差異。
        r = self._send(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                       p1=MODE_FLAG_CUSTOM, p2=MAIN_MODE_AUTO, p3=SUB_MODE_AUTO_LOITER)
        if r != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            return self._finish(response, name, False, f"切換盤旋被拒絕：{self._ack_text(r)}")
        self._begin_task(name, 5.0, done_now=True)     # 切模式沒有「過程」，送出就算完成
        if canceled:
            return self._finish(response, name, True,
                                f"已取消 {'、'.join(sorted(canceled))}，就地盤旋")
        return self._finish(response, name, True, "已切換為盤旋（定點定高）")

    def _srv_land(self, request, response):
        name = "land"
        self.get_logger().info(f"[{name}] 收到指令")
        if not self._connected():
            return self._finish(response, name, False, "沒有收到 PX4 的狀態，確認 agent 有連上")
        if not self._armed() or self.land.landed:
            return self._finish(response, name, False, "已經在地上")
        # 進行中的任務（例如巡航）被降落打斷，要標成失敗，
        # 否則 Node-RED 的「等待完成」會一直等一個永遠不會完成的任務
        self._end_task(False, "被降落指令取消")
        # 先停掉所有在驅動飛機的動作，再煞停，否則 PX4 會先飛完當下那一段才開始降。
        # ⚠️ 導航中降落也必須走 _settle_after_cancel —— NAV_LAND 雖然一律被接受
        #    （force=true），但 bridge 的 ensureOffboard() 會在 cmd_timeout_s 內
        #    持續把 PX4 要回 OFFBOARD，和 AUTO_LAND 互搶。
        self._settle_after_cancel(self._cancel_all_motion("使用者按了降落"))
        # PX4 對 NAV_LAND 一律接受（Commander.cpp:1093 force=true），降在目前位置。
        r = self._send(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        if r != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            return self._finish(response, name, False, f"降落指令被拒絕：{self._ack_text(r)}")
        self._begin_task(name, 120.0)
        return self._finish(response, name, True, "已開始降落")

    # -------------------------------------------------------------------------
    #  Nav2 導航（goto）
    #
    #  和這個節點其他動作最大的差別：這裡**不送 VehicleCommand**。
    #  目標交給 Nav2，Nav2 算路徑、發 cmd_vel，cmd_vel_to_px4_node 翻成
    #  offboard setpoint。所以飛行中 PX4 會在 OFFBOARD 而不是 AUTO_LOITER。
    # -------------------------------------------------------------------------

    def _nav_set(self, **kw):
        """更新 api/status 的 nav 欄位（只有這裡會寫，避免多處各寫一半）。"""
        self._nav_info = dict(self._nav_info, **kw)

    def _cancel_nav(self, why):
        """取消進行中的 Nav2 目標。有取消到才回 True（和 _cancel_patrol 同慣例）。

        ⚠️ 一定要取消而不是只切 PX4 模式。Nav2 不知道有人按了 hover，
        它會繼續發 cmd_vel，bridge 繼續灌 offboard setpoint，
        結果是「切了 LOITER 但飛機還在往目標走」—— 兩個控制權持有者打架。
        """
        with self._nav_lock:
            h = self._nav_handle
            self._nav_handle = None
        if h is None:
            return False
        self.get_logger().info(f"[goto] 取消導航：{why}")
        # cancel_goal_async 不等它完成 —— 服務回呼不能卡在這裡。
        # Nav2 收到取消後會停發 cmd_vel，bridge 逾時就會自己處理
        # （模擬是銷毀 publisher 交還控制權，實機是原地保持）。
        try:
            h.cancel_goal_async()
        except Exception as e:                       # noqa: BLE001
            self.get_logger().warn(f"[goto] 取消請求送不出去：{e}")
        self._nav_set(state="canceled", message=why, distance_remaining=None)
        return True

    def _srv_goto(self, request, response):
        name = "goto"
        gx, gy = float(request.map_x), float(request.map_y)
        self.get_logger().info(f"[{name}] 收到指令 map=({gx:+.2f}, {gy:+.2f})")

        if not self._connected():
            return self._finish(response, name, False,
                                "沒有收到 PX4 的狀態，確認 agent 有連上")
        if not self._armed() or self.land.landed:
            return self._finish(response, name, False, "飛機在地上，先呼叫 api/takeoff")
        # ⚠️ 這個檢查是最常中的那一個。Nav2 沒跑的話 send_goal_async 會
        #    無聲無息地永遠不回來，看起來像「服務掛了」。
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            return self._finish(
                response, name, False,
                f"Nav2 沒有在跑（{self.nav_action} 找不到）。"
                "用 warehouse_all.launch.py 啟動，或單獨開 nav2.launch.py level:=full")

        # 既有的動作要先停掉，否則多個控制路徑同時在動。
        # keep 不填 —— 連舊的 nav 也要取消（送新目標會取代舊的，但狀態要收乾淨）。
        self._settle_after_cancel(self._cancel_all_motion(f"改用 {name}"))

        yaw = float(request.yaw)
        # 「不在乎朝向」的兩種寫法都接受。|yaw| > pi 這條是給 Node-RED 用的 ——
        # rosbridge 不能帶 NaN 給浮點欄位（型別檢查在
        # rosbridge_library/internal/message_conversion.py:462-465，字串 "nan"
        # 和 null 都會被拒絕），所以從那邊只能用一個「超出合法範圍」的數字。
        if abs(yaw) > math.pi:
            yaw = NAN
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        if math.isnan(yaw):
            # 四元數必須是單位長度，w=1 代表「朝東」。
            # 填這個不代表要求朝東 —— nav2 參數的 yaw_goal_tolerance 是 3.14，
            # 終點朝向本來就不檢查。給 NaN 會讓 Nav2 的 TF 計算整串變 NaN。
            goal.pose.pose.orientation.w = 1.0
        else:
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        timeout = float(request.timeout) or self.goto_timeout
        self._nav_seq += 1
        seq = self._nav_seq
        self._nav_set(state="running", goal=[round(gx, 3), round(gy, 3)],
                      seq=seq, distance_remaining=None, message="")
        self._begin_task(name, timeout, target=None)

        fut = self._nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback)
        fut.add_done_callback(lambda f, q=seq: self._nav_goal_response(f, q))
        return self._finish(response, name, True,
                            f"已送出導航目標 map=({gx:+.2f}, {gy:+.2f})，"
                            f"逾時 {timeout:.0f} 秒")

    def _nav_goal_response(self, fut, seq):
        """Nav2 回覆「接受／拒絕」這個目標。"""
        if seq != self._nav_seq:
            return                      # 已經被更新的 goto 取代了
        try:
            h = fut.result()
        except Exception as e:           # noqa: BLE001
            self._nav_set(state="failed", message=f"送出失敗：{e}")
            self._end_task(False, f"導航目標送出失敗：{e}")
            return
        if not h.accepted:
            self._nav_set(state="failed", message="Nav2 拒絕了這個目標")
            self._end_task(False, "Nav2 拒絕了這個目標")
            return
        with self._nav_lock:
            self._nav_handle = h
        h.get_result_async().add_done_callback(
            lambda f, q=seq: self._nav_result(f, q))

    def _nav_feedback(self, msg):
        fb = msg.feedback
        self._nav_set(distance_remaining=round(float(fb.distance_remaining), 2))

    def _nav_result(self, fut, seq):
        if seq != self._nav_seq:
            return
        with self._nav_lock:
            self._nav_handle = None
        try:
            status = fut.result().status
        except Exception as e:           # noqa: BLE001
            self._nav_set(state="failed", message=str(e))
            self._end_task(False, f"導航結果讀取失敗：{e}")
            return
        # GoalStatus：4=SUCCEEDED 5=CANCELED 6=ABORTED
        if status == 4:
            self._nav_set(state="done", message="已到達", distance_remaining=0.0)
            self._end_task(True, "已到達導航目標")
        elif status == 5:
            self._nav_set(state="canceled", message="已取消")
            self._end_task(False, "導航被取消")
        else:
            # ABORTED 最常見的原因是目標在障礙物裡或規劃不出路徑。
            # 不在這裡猜 —— Nav2 的 log 會寫，這裡只誠實回報。
            self._nav_set(state="failed", message=f"Nav2 回報失敗（status={status}）")
            self._end_task(False, f"導航失敗（Nav2 status={status}）"
                                  "，看 Nav2 的 log 查原因")

    # -------------------------------------------------------------------------
    #  追蹤視覺目標
    # -------------------------------------------------------------------------

    def _on_target(self, m):
        self.target = (float(m.point.x), float(m.point.y))
        self._t_target = time.monotonic()
        self._target_hist.append((self._t_target, *self.target))

    def _target_velocity(self):
        """估目標的水平速度（本地 NED，m/s）。估不出來回傳 (0.0, 0.0)。

        為什麼需要：跟隨的站位點要放在「目標後方」—— 後方是**沿目標的行進
        方向**，不是沿「飛機→目標」的連線。後者會讓命令的位移塌縮成
        (d - dist)，距離越接近設定值位移越小，等於一個增益小於 1 的比例
        控制器：對靜止目標正確，對等速移動的目標必定穩態落後。
        2026-10-08 實測：車 0.5 m/s 直線走，飛機只跟到 0.12 m/s。

        用「時間窗內最舊和最新的兩筆」做差分而不是相鄰兩筆 ——
        偵測是逐幀的，相鄰兩筆的時間差小、像素雜訊會被放大成很大的速度。

        ---------------------------------------------------------------------
        🔴 2026-10-19：門檻從「dt >= 0.2 秒」收緊到 0.6 秒 ＋ 至少 4 筆
        ---------------------------------------------------------------------
        舊門檻太鬆，直接造成一次摔機。鎖定目標的那一刻這個函式手上只有
        兩三筆、間隔剛好 0.2~0.3 秒的資料，而且那幾筆正好落在「從 3.2 m/s
        硬煞、機頭抬 22.7°」的姿態劇變裡（target_world_node 用最新姿態配
        0.1~0.2 秒前的影像，姿態一動算出來的地面點就亂跳）。
        估出來的方向是純雜訊，站位點被放到車的西邊 2.12 m —— 貨架的位置。
        飛機滿速往西開 1.9 秒撞上去。

        **回傳 (0,0) 不是「失敗」，是一個安全的答案**：呼叫端看到它會退回
        「沿飛機→車的視線前後移動」，那條路只會朝車或離開車，不會橫移。
        """
        h = self._target_hist
        if len(h) < TRACK_VEL_MIN_SAMPLES:
            return (0.0, 0.0)
        now = h[-1]
        # 只看時間窗內的。⚠️ 要數「窗內的筆數」而不是整個 deque 的長度 ——
        # deque 會留著上一次看到目標時的舊資料，那些不能算進可信度。
        recent = [r for r in h if now[0] - r[0] <= TRACK_VEL_WINDOW_S]
        if len(recent) < TRACK_VEL_MIN_SAMPLES:
            return (0.0, 0.0)
        old = recent[0]
        dt = now[0] - old[0]
        if dt < TRACK_VEL_MIN_WINDOW_S:     # 時間窗太短，差分不可靠
            return (0.0, 0.0)
        return ((now[1] - old[1]) / dt, (now[2] - old[2]) / dt)

    def _ideal_standoff(self, height):
        """讓目標落在畫面正中的跟隨距離（公尺）。

            最佳站位 = 高度 / tan(相機俯角)

        這就是光軸打到地面的位置。俯角 90（正下方）→ 0（正上方）；
        俯角 45、高度 3 → 3.0 m（和 45° 相機時代的舊預設 2.0 不同，
        舊值其實也不是最佳，只是避開盲區的折衷）。

        算不出來（俯角不在 0~90 之間、或高度無效）就回傳 None，
        由呼叫者退回 TRACK_DISTANCE_M。
        """
        pitch = float(self.camera_pitch_deg)
        if not (0.0 < pitch <= 90.0) or not math.isfinite(height) or height <= 0.0:
            return None
        t = math.tan(math.radians(pitch))
        if t <= 1e-6:
            return None
        return height / t

    def _target_age(self):
        return time.monotonic() - self._t_target if self.target is not None else None

    def _cancel_track(self, why):
        """請背景的追蹤停下來並等它結束。回傳有沒有真的取消到東西。"""
        t = self._track_thread
        if t is None or not t.is_alive():
            return False
        self.get_logger().info(f"[track] 取消：{why}")
        self._track_stop.set()
        t.join(timeout=5.0)
        return True

    def _track_say(self, message, ok=True):
        """把追蹤途中的階段變化寫進 api/status 的 last_message。

        為什麼需要：last_message 原本**只在服務回覆的那一瞬間寫一次**，所以
        整趟追蹤那一欄都凍在「開始追蹤…」，畫面看起來像卡在找目標
        （Oliver 2026-10-12 回報）。階段變化才是那一欄該顯示的東西。
        """
        self._last_cmd, self._last_ok, self._last_msg = "track", ok, message

    def _srv_track(self, request, response):
        name = "track"
        dur = float(request.duration) or TRACK_DURATION_S
        alt = float(request.altitude)
        # distance=0 的意思是「自動」：算出讓目標落在畫面正中的距離。
        # 它會隨高度變（高度 / tan(俯角)），所以不能寫成固定常數。
        dist = float(request.distance)
        auto_note = ""
        if dist == 0.0:
            h_for_calc = alt if alt != 0.0 else (self._altitude() or 0.0)
            ideal = self._ideal_standoff(h_for_calc)
            if ideal is None:
                dist = TRACK_DISTANCE_M
                auto_note = f"（算不出最佳距離，用預設 {dist:.2f} m）"
            else:
                dist = max(0.0, ideal)
                auto_note = (f"（自動：高度 {h_for_calc:.2f} m / "
                             f"tan({self.camera_pitch_deg:.0f}°) = {dist:.2f} m，"
                             "目標會落在畫面正中）")
        self.get_logger().info(
            f"[{name}] 收到指令（距離 {dist:.2f} m{auto_note}、{dur:.0f} 秒）")

        # 下限放寬到 0 —— 俯角 67° 的相機正下方看得到（-0.22~3.60 m），
        # 「飛到正上方」是合法的要求。45° 相機時代正下方是盲區，
        # 所以舊的下限是 0.5。
        if not (0.0 <= dist <= PATROL_MAX_RANGE_M):
            return self._finish(response, name, False, f"距離 {dist:.2f} m 不合理")
        if alt != 0.0 and not (self.min_takeoff_altitude <= alt <= self.max_takeoff_altitude):
            return self._finish(response, name, False,
                                f"高度 {alt:.2f} m 超出範圍 {self.min_takeoff_altitude}~"
                                f"{self.max_takeoff_altitude} m")
        if not self._connected():
            return self._finish(response, name, False, "沒有收到 PX4 的狀態，確認 agent 有連上")
        if not self._armed() or self.land.landed:
            return self._finish(response, name, False, "飛機在地上，請先起飛")
        if self.lpos is None or not self.lpos.xy_global or not self.lpos.z_global:
            return self._finish(response, name, False, "沒有有效的經緯度參考，無法換算目標點")

        # --- 搜尋環線（選填）：map 座標 → 本地 NED --------------------------
        mxs = [float(v) for v in request.search_x]
        mys = [float(v) for v in request.search_y]
        if len(mxs) != len(mys):
            return self._finish(response, name, False,
                                f"search_x 有 {len(mxs)} 個、search_y 有 {len(mys)} 個，長度要一樣")
        search = None
        if mxs:
            if len(mxs) < 3:
                return self._finish(response, name, False,
                                    f"搜尋環線至少要 3 個點，現在有 {len(mxs)}")
            if len(mxs) > PATROL_MAX_POINTS:
                return self._finish(response, name, False,
                                    f"搜尋環線最多 {PATROL_MAX_POINTS} 個點，現在有 {len(mxs)}")
            if not self.local_origin_map:
                return self._finish(
                    response, name, False,
                    "有給搜尋環線但節點的 local_origin_map 參數是空的，無法把 map 座標"
                    "換算成本地 NED。用 warehouse_all.launch.py 啟動，"
                    "或手動帶 local_origin_map:=\"<x>,<y>\"（和 px4_bridge 的 odom_origin 相同）")
            search = []
            for mx, my in zip(mxs, mys):
                ned = self._map_to_local(mx, my)
                if ned is None:
                    return self._finish(response, name, False,
                                        f"local_origin_map 的格式不對：{self.local_origin_map!r}"
                                        "（要 \"x,y\"）")
                if abs(ned[0]) > PATROL_MAX_RANGE_M or abs(ned[1]) > PATROL_MAX_RANGE_M:
                    return self._finish(
                        response, name, False,
                        f"搜尋航點 map({mx:+.1f},{my:+.1f}) 換算成離起飛點 "
                        f"北{ned[0]:+.1f} 東{ned[1]:+.1f}，超出 ±{PATROL_MAX_RANGE_M:.0f} m")
                search.append(ned)
            self.get_logger().info(
                f"[{name}] 搜尋環線 {len(search)} 點（map → 離起飛點 NED）："
                + "、".join(f"({mx:+.1f},{my:+.1f})→北{a:+.1f}東{b:+.1f}"
                            for (mx, my), (a, b) in zip(zip(mxs, mys), search)))

        age = self._target_age()
        if self.target is None:
            # 從開機到現在完全沒收到過目標訊息。指令還是接受 —— 視覺節點可能剛啟動，
            # 或目標還沒進到畫面，等一下就會出現。但先把最可能的原因印出來。
            self.get_logger().warn("[track] 還沒收到過任何目標位置，"
                                   "確認 vision.launch.py 有開、相機看得到目標")

        # 同一時間只能有一個東西在下目標點
        # 2026-10-07：原本只取消 patrol 和 track，漏了 nav ——
        # 導航中開始追蹤會變成 Nav2 和 DO_REPOSITION 同時在動。
        self._settle_after_cancel(self._cancel_all_motion("改成追蹤目標"))
        self._end_task(False, "被追蹤指令取代")

        height = alt if alt != 0.0 else (self._altitude() or 0.0)
        looking = age is None or age > TRACK_LOST_S
        self._track_stop.clear()
        self._track_info = {"active": True, "distance": None,
                            "lost_s": round(age, 1) if age is not None else None,
                            "searching": looking, "search_s": 0.0,
                            "phase": "searching" if looking else "following",
                            "search_index": None, "search_laps": None,
                            # 新一趟追蹤 → 還沒鎖定過，所以 reacquire 是 False
                            "want": round(dist, 2), "locked_s": None,
                            "reacquire": False}
        # 任務逾時要把「開頭等目標」的時間也算進去，否則一開始找久一點就被判逾時。
        # 追蹤途中每一次等待，_track_worker 會自己把 _task_deadline 往後挪。
        # 任務逾時：有搜尋環線時「最多繞兩圈」沒有固定秒數上限（RTF 會變），
        # 所以給一個寬鬆的外層保險，真正的放棄條件是圈數（見 _track_worker）。
        if search is not None:
            peri = sum(math.hypot(search[(i + 1) % len(search)][0] - search[i][0],
                                  search[(i + 1) % len(search)][1] - search[i][1])
                       for i in range(len(search)))
            # 周長 / 最慢合理速度(0.5) x 圈數 x 2 倍 RTF 餘裕
            budget = peri / 0.5 * TRACK_SEARCH_LAPS * 2.0
        else:
            budget = TRACK_ACQUIRE_S
        self._begin_task(name, dur + budget + 15.0)
        self._track_thread = threading.Thread(
            target=self._track_worker, args=(dist, height, dur, search), daemon=True)
        self._track_thread.start()
        if not looking:
            pre = ""
        elif search is not None:
            pre = f"先巡航 {len(search)} 點的環線找目標（最多 {TRACK_SEARCH_LAPS} 圈），"
        else:
            pre = f"先停在原地轉頭找目標（最多 {TRACK_ACQUIRE_S:.0f} 秒），"
        return self._finish(response, name, True,
                            f"開始追蹤：{pre}保持 {dist:.2f} m{auto_note}、"
                            f"高度 {height:.2f} m（搜尋用；跟隨的高度由 bridge "
                            f"的 flight_altitude 決定）、{dur:.0f} 秒")

    def _track_worker(self, dist, height, duration, search=None):
        """每秒把「目標前方 dist 公尺」的點送給 PX4。

        為什麼不是直接飛到目標正上方：真實任務通常要保持距離觀察，
        而且飛到正上方時相機（下傾 45°）反而看不到它，下一秒就追丟。

        為什麼用 DO_REPOSITION 而不是 offboard：
            這樣完全沿用巡航那一套（已經驗過），不用切模式、
            也沒有「斷流觸發失效保護」的風險。地面車很慢，1 Hz 更新足夠。

        看不到目標時有兩種行為（2026-10-08 新增後者）：

            search=None   停在原地**慢慢轉頭掃描**，最多 TRACK_ACQUIRE_S 秒。
                          空曠場地夠用：目標就在附近，只是暫時滑出 ±34° 的水平視野。

            search=[…]    **巡航那條環線去找**（本地 NED 航點），最多
                          TRACK_SEARCH_LAPS 圈。倉庫非得這樣：俯角 67° 的相機
                          在 3 m 高只看得到前方 0~3.6 m，而走道長 12 m ——
                          站在這頭轉一整圈也看不到那頭的車。原地掃描解決不了
                          「目標根本不在附近」這件事。

        兩種情況下「找」的時間都不算進 duration：要求追 60 秒就是真的追到 60 秒。
        開頭沒看到也接受指令 —— 視覺節點可能比指令晚一步、或目標還沒進畫面。
        """
        alt_amsl = float(self.lpos.ref_alt) + height
        deadline = None          # 追蹤的截止時間。第一次看到目標才開始算
        wait_since = None        # 這一次「看不到」是從什麼時候開始（None = 現在看得到）
        just_acquired = False    # 這一拍是不是「剛從搜尋切進跟隨」
        locked_since = None      # 這一次鎖定是從什麼時候開始（None = 現在沒鎖定）
        locked_once = False      # 這一趟有沒有鎖定過（決定搜尋訊息的講法）
        s_i = None               # 目前要飛的搜尋航點索引（None = 還沒選）
        s_first = True           # 這是不是「第一次」進搜尋（決定從哪個航點開始）
        # 環線的繞行方向。+1 = 照列表順序、-1 = 反向。
        #
        # 為什麼要能反轉（2026-10-10 Oliver 提的）：
        #   飛機和車刻意反向繞圈，所以**每一段都是對頭相遇**（接近 2.0+0.5
        #   = 2.5 m/s）—— 取得目標快（最久 19 秒），但很容易掠過。
        #   而掠過的那一刻我們明確知道一件事：**車就在身後**，只是相機往前看
        #   拍不到而已。這時如果繼續原方向，會以 2.5 m/s 遠離，要再繞
        #   31~47 m 才會重新遇到。
        #   反轉成同向就變成「從後面追」：接近速度 2.0-0.5 = 1.5 m/s，
        #   而且車在前方，正好在相機視野裡。
        s_dir = 1
        # 只反轉一次。反轉後兩者同向，不會再有對頭相遇；之後再丟失（例如車
        # 轉過彎角）繼續同方向走就會追上，再反轉一次反而會變回對頭。
        # 第一次相遇一定是對頭的（閉環上反向繞行，每一段都是對頭），
        # 所以「一次反轉」永遠是對的方向。
        s_flipped = False
        s_at_acquire = None      # 鎖定那一刻「正要飛去」的航點索引
        s_sent = None            # 已經把哪個索引送給 PX4 了（避免每秒重送同一個點）
        s_adv = 0                # 總共前進過幾個航點 → 圈數 = s_adv // len(search)
        cmd_vel_active = False   # 上一拍是不是在送 cmd_vel（切回搜尋時要先交棒）
        last_search_tick = 0.0   # 搜尋段上次真正動作的時間（見 TRACK_SEARCH_TICK_S）
        follow_since = None      # 這一段跟隨是什麼時候接手的（給 TRACK_HANDOFF_RAMP_S
                                 # 的速度斜坡用）。跟丟退回搜尋時要清掉 ——
                                 # 重新鎖定時又是一次「從高速中接手」，要重新斜坡。
        last_log = 0.0           # 上次印診斷的時間（見 TRACK_LOG_S）
        last_blocked = None      # 上一拍是哪個禁飛方框擋住指令（只在換的時候警告）
        b_dir = None             # 低通後的「站位方向」單位向量（從車指向站位點）。
                                 # None = 這一段跟隨還沒有方向 → 下一拍直接採用
                                 # 原始值，不要從舊的濾過來。跟丟退回搜尋時要清掉：
                                 # 重新鎖定時車可能已經在走別的方向了。
        b_dir_t = None           # b_dir 上次更新的時間（算低通係數用）
        try:
            while True:
                if self._track_stop.wait(TRACK_UPDATE_S):
                    return
                if self.lpos is None or not self.lpos.xy_valid:
                    self._end_task(False, "位置估計失效")
                    return

                age = self._target_age()
                lost = age is None or age > TRACK_LOST_S
                if not lost and locked_since is None:
                    # 「保持了多久」從**看得到目標的第一拍**開始算。
                    # ⚠️ 不能只在下面「從搜尋切回跟隨」那一段設 —— 按下追蹤時
                    #    車就在畫面裡的話，wait_since 從頭到尾都是 None，
                    #    那一段永遠不會跑，畫面就一直看不到保持時間。
                    locked_since = time.monotonic()
                    locked_once = True
                # ⚠️ 狀態要分三種，不能只用「有沒有距離」判斷（2026-09-29 Oliver 回報）：
                #     正常追蹤       distance 有值、searching False
                #     暫時看不到     還在追（還沒滿 TRACK_LOST_S），distance 留著上一次的值
                #     真的在找目標   searching True，飛機停住轉頭掃描
                #   之前把 distance 清成 None 當作「在找」，結果車子一靠近就掉出
                #   45° 相機的視野，明明在追卻顯示「尋找目標中」。
                # lost_s 是「上次看到目標是幾秒前」—— 它會跨趟累積（降落期間沒人看得到
                #   目標），所以「找了多久」要另外用 search_s 算，不能拿 lost_s 當那個用。
                self._track_info = {
                    "active": True,
                    "distance": self._track_info.get("distance"),
                    "lost_s": round(age, 1) if age is not None else None,
                    "searching": False,
                    "search_s": None,
                    # 2026-10-08：三段式追蹤的階段。searching / following
                    # （Node-RED 的顯示要能分辨「在找」和「在追」）
                    "phase": "following",
                    "search_index": None,
                    "search_laps": None,
                    # 畫面要分辨「追到了而且穩住了」：實際距離 / 要保持的距離，
                    # 加上已經保持多久。locked_since 在下面「剛看到目標」那一段
                    # 才設，所以鎖定的**那一拍** locked_s 還是 None。
                    "want": round(dist, 2),
                    "locked_s": (None if locked_since is None
                                 else round(time.monotonic() - locked_since, 1)),
                    "reacquire": locked_once}

                if lost:
                    self._track_info["phase"] = "searching"
                    # 跟丟了 → 「保持了多久」歸零（重新鎖定才重新計時）
                    locked_since = None

                    # --- 跟隨(offboard) → 搜尋(AUTO) 的模式交接 -----------------
                    # ⚠️ 不交接就直接送 DO_REPOSITION 的話，bridge 的
                    #    ensureOffboard() 會把 PX4 拉回 OFFBOARD，AUTO 和 OFFBOARD
                    #    互搶 —— 而且不會有任何錯誤訊息（和 _settle_after_cancel
                    #    處理 nav 的理由完全一樣）。
                    if cmd_vel_active:
                        cmd_vel_active = False
                        follow_since = None
                        # 方向濾波器也要清掉 —— 重新鎖定時車可能已經轉向了，
                        # 從舊方向慢慢濾過去會讓飛機先往錯的地方走 1.5 秒。
                        b_dir = b_dir_t = None
                        self._release_bridge_hold()
                        self.get_logger().info(
                            "[track] 跟丟 → 停送 cmd_vel，等 bridge 交還控制權")
                        # 停送 + 等超過 cmd_timeout_s，bridge 才會真的放手
                        if self._track_stop.wait(TRACK_MODE_SWITCH_S):
                            return
                        last_search_tick = 0.0     # 交接完立刻讓搜尋動起來

                    # 搜尋段維持 1 秒一拍（原地掃描是按「每拍轉 30°」設計的）
                    now_t = time.monotonic()
                    if now_t - last_search_tick < TRACK_SEARCH_TICK_S:
                        continue
                    last_search_tick = now_t

                    # --- 巡航搜尋（有給環線時走這條）------------------------
                    if search is not None:
                        if wait_since is None:
                            wait_since = time.monotonic()
                            self.get_logger().warn(
                                f"[track] 看不到目標，開始巡航 {len(search)} 點的環線找"
                                f"（最多 {TRACK_SEARCH_LAPS} 圈）")
                            # 「跟丟回頭找」和「一開始就在找」對使用者的意義
                            # 完全不同：前者代表車就在附近，不必重按一次追蹤。
                            self._track_say(
                                ("跟丟了，回頭巡航環線找" if locked_once
                                 else f"看不到目標，開始巡航 {len(search)} 點的環線找")
                                + f"（最多 {TRACK_SEARCH_LAPS} 圈）")
                        if s_i is None:
                            # ⚠️ 2026-10-10：第一次搜尋和重新搜尋的起點規則不同。
                            #
                            # 第一次 → **照列表的第 0 點**。
                            #   原本是「挑最近的」，但那會在使用者沒要求的地方多停
                            #   一次：從生成點 (-14,-9) 挑到的是 (0,-9)，於是底部
                            #   那段 y=-9 被切成「14 m → 停 → 5.6 m → 停」兩段。
                            #   而 y=±9 在貨架外面（貨架只到 ±7），車永遠不在那裡，
                            #   相機往前看掃的是空地 —— 那個中間的停頓完全沒意義。
                            #   照列表順序走就是一口氣 19.6 m 到 (5.6,-9) 再轉北。
                            #   **代價：列表的第 0 點必須能從飛機現在的位置直線到達**
                            #   （api/patrol 走直線、不避障）。倉庫的 (5.6,-9) 在
                            #   貨架南側，從生成點和從走道口都是直線可達，所以安全。
                            #
                            # 重新搜尋 → **挑最近的**。
                            #   丟失目標時飛機在環線的任何位置，硬飛回第 0 點可能很遠，
                            #   而且路上可能有貨架（直線會卡住）。從最近的接上才合理。
                            if s_at_acquire is not None and not s_flipped:
                                # 掠過之後：車在身後 → 反轉繞行方向，回頭走向
                                # 「剛才來的」那個航點（見 s_dir 的說明）。
                                s_i = (s_at_acquire - s_dir) % len(search)
                                s_dir = -s_dir
                                s_flipped = True
                                self.get_logger().info(
                                    f"[track] 反向繞圈：車在身後，改成從後面追"
                                    f"（回頭走向第 {s_i} 點，方向 {s_dir:+d}）")
                            elif s_first:
                                s_i = 0
                                self.get_logger().info(
                                    f"[track] 從環線的第 0 點開始找"
                                    f"（北{search[0][0]:+.1f} 東{search[0][1]:+.1f}）")
                            else:
                                s_i = min(range(len(search)),
                                          key=lambda k: math.hypot(
                                              search[k][0] - float(self.lpos.x),
                                              search[k][1] - float(self.lpos.y)))
                                self.get_logger().info(
                                    f"[track] 重新搜尋：從最近的第 {s_i} 個航點接上"
                                    f"（方向 {s_dir:+d}）")
                            s_first = False
                            s_sent = None
                        px, py = search[s_i]
                        if self._distance_to(px, py) < PATROL_ARRIVE_M:
                            s_i = (s_i + s_dir) % len(search)
                            s_adv += 1
                            s_sent = None
                            laps = s_adv // len(search)
                            if laps >= TRACK_SEARCH_LAPS:
                                self._end_task(
                                    False,
                                    f"巡航了 {laps} 圈（{s_adv} 個航點）還是沒有看到目標")
                                return
                            px, py = search[s_i]
                        if s_sent != s_i:
                            # 只在換航點時送 —— 和 _patrol_worker 一樣。
                            # 每秒重送同一個點會讓 navigator 不斷重設設定點。
                            lat, lon = self._local_to_global(px, py)
                            # 機頭朝行進方向：相機只看得到前方 0~3.6 m，
                            # 機頭不轉等於閉著眼睛飛（和 _patrol_worker 同一個理由）
                            yaw = math.atan2(py - float(self.lpos.y),
                                             px - float(self.lpos.x))
                            r = self._send(VehicleCommand.VEHICLE_CMD_DO_REPOSITION,
                                           p1=-1.0, p2=REPOSITION_CHANGE_MODE,
                                           p4=yaw, p5=lat, p6=lon, p7=alt_amsl)
                            if r != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                                self._end_task(
                                    False, f"搜尋航點被拒絕：{self._ack_text(r)}")
                                return
                            s_sent = s_i
                            self.get_logger().info(
                                f"[track] 搜尋航點 {s_i}："
                                f"北{px:+.1f} 東{py:+.1f}（第 "
                                f"{s_adv // len(search) + 1}/{TRACK_SEARCH_LAPS} 圈）")
                        self._track_info["searching"] = True
                        self._track_info["search_s"] = round(
                            time.monotonic() - wait_since, 1)
                        self._track_info["search_index"] = s_i
                        self._track_info["search_laps"] = s_adv // len(search)
                        continue

                    # --- 原地轉頭掃描（沒給環線時的舊行為）------------------
                    # 停在原地等 —— 不要往最後看到的方向衝，那是誤判時最危險的行為
                    if wait_since is None:
                        wait_since = time.monotonic()
                        self._pause_here()      # 先煞停水平移動，位置就定在這裡
                        self.get_logger().warn("[track] 看不到目標，停在原地轉頭找")
                        self._track_say(
                            ("跟丟了，停在原地轉頭找" if locked_once
                             else "看不到目標，停在原地轉頭找")
                            + f"（最多 {TRACK_ACQUIRE_S:.0f} 秒）")
                    else:
                        # 原地掃描：每秒把機頭往同一個方向轉一點，轉滿一圈就等於
                        # 用那 ±34° 的視野掃過全場。位置留 NaN → navigator 沿用現在的
                        # 設定點（navigator_main.cpp:336），所以只會轉頭、不會飄走。
                        yaw = float(self.lpos.heading) + math.radians(TRACK_SCAN_STEP_DEG)
                        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
                        r = self._send(VehicleCommand.VEHICLE_CMD_DO_REPOSITION,
                                       p1=-1.0, p2=REPOSITION_CHANGE_MODE, p4=yaw)
                        if r != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                            self._end_task(False, f"掃描轉向被拒絕：{self._ack_text(r)}")
                            return
                    searched = time.monotonic() - wait_since
                    self._track_info["searching"] = True
                    self._track_info["search_s"] = round(searched, 1)
                    if searched > TRACK_ACQUIRE_S:
                        self._end_task(False, f"等了 {TRACK_ACQUIRE_S:.0f} 秒還是沒有看到目標")
                        return
                    continue

                if wait_since is not None:
                    # 等待不算進追蹤時間，任務逾時也要跟著往後挪 ——
                    # 不挪的話 _check_task 會在我們還在追的時候把任務判定逾時
                    waited = time.monotonic() - wait_since
                    wait_since = None
                    # ⚠️ 2026-10-08 加這個旗標，是因為舊版的跟隨用
                    #    TRACK_GOAL_TOL_M 當閘門：「站位點離飛機不夠遠就不送指令」，
                    #    而剛鎖定那一拍 _target_hist 只有一兩筆、速度估計是 (0,0)，
                    #    站位點退回「沿連線後退」會算出很近的點，閘門幾乎一定擋住 ——
                    #    症狀是「第一次掃到不會鎖定，要繞第二圈」。
                    #
                    #    2026-10-16 改速度控制之後**那個閘門已經不存在**（每一拍
                    #    都送 cmd_vel），所以這個旗標對跟隨不再有作用。留著是因為
                    #    搜尋段還用 s_at_acquire 記「鎖定那一刻要去的航點」。
                    #
                    #    而「PX4 手上還握著搜尋航點」這件事也自動解決了：
                    #    第一則 cmd_vel 會讓 bridge 把 PX4 拉進 OFFBOARD
                    #    （ensureOffboard），AUTO 的航點就失效了。
                    just_acquired = True
                    # 速度斜坡的起點。2026-10-19 摔機就是在這一刻接手的 ——
                    # 當時飛機正以 3.2 m/s 飛搜尋段，第一則 cmd_vel 直接給 1.40，
                    # PX4 抬頭 22.7° 硬煞（詳見 TRACK_HANDOFF_RAMP_S 的說明）。
                    follow_since = time.monotonic()
                    # ⚠️ 跟隨階段的高度**不是** Track 的 altitude 決定的。
                    #    cmd_vel 沒有高度這個維度，定高是 cmd_vel_to_px4_node
                    #    用它自己的 flight_altitude 參數做的（launch 時設定，
                    #    warehouse_all 的 altitude 會傳下去）。
                    #    Track 的 altitude 只影響**搜尋**階段的航點高度。
                    #    兩者不同的話飛機一進入跟隨就會變高度 —— 印出來才看得見。
                    self.get_logger().info(
                        f"[track] 鎖定目標，改用速度控制跟隨"
                        f"（搜尋高度 {height:.2f} m；跟隨高度由 bridge 的 "
                        f"flight_altitude 決定，不是這個值）")
                    # 記下鎖定那一刻正要飛去的航點 —— 下次進搜尋時要靠它算出
                    # 「剛才是從哪一個航點過來的」，才能回頭走向車所在的那一側。
                    # ⚠️ 不能用「挑最近的」：飛機在 (5.6,+2) 往北、原本要去
                    #    (5.6,+9)，車在 (5.6,0) 往南 —— 最近的航點是 (5.6,+9)
                    #    （7 m）而不是 (5.6,-9)（11 m），挑最近的會繼續往北，
                    #    正好和車相反。
                    s_at_acquire = s_i
                    # 搜尋狀態歸零：圈數要重新算（這一次搜尋已經成功結束了）。
                    s_i = s_sent = None
                    s_adv = 0
                    self._track_info["search_index"] = None
                    self._track_info["search_laps"] = None
                    self.get_logger().info(f"[track] 看到目標了（找了 {waited:.0f} 秒）")
                    self._track_say(f"看到目標了（找了 {waited:.0f} 秒），開始跟隨")

                if deadline is None:
                    deadline = time.monotonic() + duration      # 從看到目標才開始計時
                elif time.monotonic() >= deadline:
                    self._end_task(True, f"追蹤 {duration:.0f} 秒結束")
                    return

                tx, ty = self.target
                dx, dy = tx - self.lpos.x, ty - self.lpos.y
                d = math.hypot(dx, dy)
                self._track_info["distance"] = round(d, 2)
                if d > PATROL_MAX_RANGE_M:
                    self._end_task(False, f"目標距離 {d:.1f} m 超出範圍")
                    return

                # --- 站位點 ---------------------------------------------------
                # ⚠️ 2026-10-08 改寫。原本是「從目標往飛機的方向退 dist」，
                #    那樣命令的位移是 (d - dist) —— 距離越接近設定值位移越小，
                #    等於增益 < 1 的比例控制器：對靜止目標正確，對等速移動的
                #    目標必定穩態落後。實測車 0.5 m/s，飛機只跟到 0.12 m/s，
                #    一路落後到超出相機的 3.6 m 取得距離而追丟。
                #
                # 現在改成「沿目標的行進方向跟在它後面」：位移不會塌縮，
                # 而且飛機會穩定待在車的正後方 —— 剛好是相機前方那個視窗。
                # ⚠️ 2026-10-19 再改一次：方向改成「混合 + 低通」。
                #    理由和量到的數字見 TRACK_DIR_TAU_S 的說明 ——
                #    站位點剛性掛在估出來的車行方向上，方向一抖整個站位點
                #    就繞著車瞬移，而那個方向本來就很抖。
                #
                # 下面都用 b = 「從車指向站位點」的單位向量來算，
                #     站位點 = 車 + b × dist
                # 兩個來源：
                #     b_vel = −v̂            車在動 → 跟在它後面
                #     b_los = −(車−飛機)/d   車沒在動 → 維持現在的視線方向
                vx, vy = self._target_velocity()
                vmag = math.hypot(vx, vy)
                b_raw = self._blend_standoff_dir(vx, vy, vmag, dx, dy, d)
                if b_raw is not None:
                    now_f = time.monotonic()
                    dt_f = (min(now_f - b_dir_t, 1.0)
                            if b_dir_t is not None else TRACK_UPDATE_S)
                    b_dir = self._filter_dir(b_dir, b_raw, dt_f)
                    b_dir_t = now_f

                if b_dir is not None:
                    gx = tx + b_dir[0] * dist
                    gy = ty + b_dir[1] * dist
                else:
                    # 既沒有車速也沒有視線（飛機正好在車的正上方）→ 待在原處
                    gx, gy = tx, ty

                # 機頭要轉向目標 —— 相機是鎖死在機身上的，不轉頭的話目標會從畫面
                # 側邊滑出去。2026-09-28 實測：目標繞圈時飛機從頭到尾朝同一個方向，
                # 第 12 秒就判定追丟，之後整整 48 秒停在原地不動。
                # DO_REPOSITION 的 param4 就是要保持的機頭角，單位弧度、NED 座標
                # （navigator_main.cpp:316 `rep->current.yaw = cmd.param4;`）
                bearing = math.atan2(dy, dx)        # NED：x 北、y 東 → atan2(東, 北)
                err = bearing - float(self.lpos.heading)
                err = (err + math.pi) % (2 * math.pi) - math.pi

                # --- 速度控制（2026-10-16 從 DO_REPOSITION 改過來）-------------
                #
                # 這裡**沒有「到位就不動」這件事**，和舊版最大的差別就在這。
                # 舊版用 TRACK_GOAL_TOL_M 判斷「站位點夠近就不送新指令」，
                # 因為每道 DO_REPOSITION 都會讓飛機衝一次；速度控制剛好相反 ——
                # 持續送才能維持速度，停送反而會讓 bridge 逾時放手。
                # 誤差小的時候比例控制自然會給出小速度，不需要死區。
                #
                # ⚠️ 看得到目標的每一拍都要送，不能 continue 跳過。
                #    斷流超過 bridge 的 cmd_timeout_s（0.5 秒）就會交還控制權。
                dn = gx - float(self.lpos.x)
                de = gy - float(self.lpos.y)
                err_d = math.hypot(dn, de)

                # 接手後的速度斜坡。上限從 0 線性拉到 TRACK_FOLLOW_SPEED ——
                # 搜尋段的 DO_REPOSITION 會讓飛機跑到 3 m/s 以上，直接塞一個
                # 1.4 m/s 的反向指令會讓 PX4 抬頭 20° 以上硬煞，而那個姿態劇變
                # 本身就會讓 target_world_node 算出來的目標位置亂跳
                # （它用最新姿態配 0.1~0.2 秒前的影像）。
                # 2026-10-19 摔機的起點就是這個硬煞。
                #
                # ⚠️ 斜坡的第一拍 cap 是 0，送出去的是**零速度**指令。
                #    模擬沒問題（config/px4_bridge.yaml 沒設 zero_cmd_deadband，
                #    declare 的預設是 0.0 → 零指令就是「速度 0」，正好是煞車）。
                #    實機的 drone_bringup/config/real/px4_bridge.yaml:77 設了
                #    0.02，所以那一拍會觸發 bridge 的「水平指令是零 → 位置鎖住
                #    原地」，下一拍又離開 —— log 上會多一對「進入/離開定點」。
                #    只有第一拍會這樣（10 Hz、1 秒斜坡），無害，但看到不要以為壞了。
                cap = TRACK_FOLLOW_SPEED
                if follow_since is not None:
                    el = time.monotonic() - follow_since
                    if el < TRACK_HANDOFF_RAMP_S:
                        cap *= max(0.0, el / TRACK_HANDOFF_RAMP_S)

                # 比例控制 + 封頂。**在 NED 做**，算完才轉機體 ——
                # 下一步的禁飛夾制必須在 NED（貨架面是垂直於 map 軸的平面，
                # 機體的「左右」會隨機頭轉而指向不同的實體方向）。
                # ⚠️ 封頂要**等比例縮放**兩軸，不能各自 clamp ——
                #    各自 clamp 會改變合速度的方向，飛機就不是朝站位點走了。
                # ⚠️ 縮放比例是 上限 / **誤差距離**，不是 上限 / (誤差×KP)。
                #    寫成後者的話實際上限會變成 TRACK_FOLLOW_SPEED / TRACK_KP
                #    （KP=1.2 時只有 1.17 m/s），飛機會跟得比設定慢而且不自知。
                #    2026-10-16 第一版就寫錯，用純算術測出來的。
                if err_d < 1e-6:
                    vn = ve = 0.0
                else:
                    k = (cap / err_d if err_d * TRACK_KP > cap else TRACK_KP)
                    vn, ve = dn * k, de * k

                # 禁飛方框 —— 跟隨段唯一的硬性保險（詳見 _clamp_no_fly）
                vn, ve, blocked = self._clamp_no_fly(vn, ve)
                fwd, left = self._ned_to_body(vn, ve)

                # 機頭轉向目標。容差內就不轉，免得在目標正前方時左右微抖。
                #
                # 🔴 符號要反號。err 是 **NED 的航向誤差**（繞下軸、順時針為正），
                #    而 cmd_vel 的 angular.z 是 ROS 慣例（繞上軸、**逆時針**為正）。
                #    cmd_vel_to_px4_node.cpp:347-349 會再反號一次轉回 PX4 的
                #    yawspeed，所以這裡不反的話等於反了兩次 ——
                #    飛機往**相反方向**轉頭，目標直接掉出 ±34° 的視野。
                #    2026-10-16 第一版就是這樣：追上之後必定跟丟，
                #    變成「追上→跟丟→重新搜尋」的循環，距離<3m 的時間從 86% 掉到 52%。
                yawrate = 0.0
                if abs(err) > TRACK_YAW_TOL_RAD:
                    yawrate = max(-TRACK_MAX_YAWRATE,
                                  min(TRACK_MAX_YAWRATE, -err * TRACK_YAW_KP))

                # --- 診斷（2026-10-19 加）------------------------------------
                # ⚠️ 2026-10-19 查「撞貨架」和「畫面左右晃」兩個問題，都是去解析
                #    PX4 的二進位飛行記錄（.ulg）才看得出來的 —— 很花時間，
                #    而且只有事後才查得到。這幾個數字就是當時需要的：
                #    車在哪、估出來的車向、站位點、誤差、實際送出去的速度。
                if blocked is not None and blocked != last_blocked:
                    # 被夾住是會摔機的那條路被攔下來了，不要只出現在節流過的行裡
                    self.get_logger().warn(
                        f"[track] 禁飛方框攔住了速度指令：{blocked}"
                        f"（站位點在 北{gx:+.1f} 東{gy:+.1f}）")
                last_blocked = blocked
                now_l = time.monotonic()
                if now_l - last_log >= TRACK_LOG_S:
                    last_log = now_l
                    if vmag >= TRACK_MIN_TARGET_SPEED:
                        vdir = f"{math.degrees(math.atan2(vy, vx)):+.0f}° {vmag:.2f}m/s"
                    elif vmag > 0.0:
                        vdir = f"太慢({vmag:.2f})"
                    else:
                        vdir = "資料不足"
                    # 原始方向和濾波後的方向都要印 —— 要比較才看得出低通有沒有
                    # 在作用。站位方向 b_dir 是「車→站位點」，反過來才是車行方向，
                    # 所以印的時候加 180° 讓兩個數字可以直接對照。
                    if b_dir is not None:
                        fdir = (f"{math.degrees(math.atan2(-b_dir[1], -b_dir[0])):+.0f}°")
                    else:
                        fdir = "無"
                    self.get_logger().info(
                        f"[track] 車(北{tx:+.1f} 東{ty:+.1f}) 車向 原始{vdir} 濾後{fdir}"
                        f"　站位(北{gx:+.1f} 東{gy:+.1f}) 差 {err_d:.2f}m"
                        f"　機頭差 {math.degrees(err):+.0f}°"
                        f"　→ 前{fwd:+.2f} 左{left:+.2f} 轉{math.degrees(yawrate):+.0f}°/s"
                        + (f"　🔴夾住 {blocked}" if blocked else ""))

                self._send_cmd_vel(fwd, left, yawrate)
                just_acquired = False
                cmd_vel_active = True
        finally:
            # ⚠️ 跟隨是靠持續送 cmd_vel 驅動的，執行緒一結束就沒有人在送了。
            #    模擬裡 bridge 逾時會自己放手，但室內實機的 hold_on_timeout=true
            #    會讓它一直維持原地定點、把 PX4 鎖在 OFFBOARD ——
            #    所以明確呼叫 release_hold 交棒（見 _release_bridge_hold）。
            #    放在 finally：不管是正常結束、被取消、還是丟例外都要交還。
            if cmd_vel_active:
                self._release_bridge_hold()
            # 追蹤結束的結果也要寫進 last_message，否則那一欄會凍在「開始追蹤…」，
            # 看不出來是追完了還是中途失敗。
            # 判 name == "track"：被別的指令接手時 _task 已經換成新指令了，
            # 那時候的結果不該被寫成追蹤的結果。
            t = self._task
            if t["name"] == "track" and t["state"] in ("done", "failed"):
                self._track_say(t["message"] or t["state"], ok=(t["state"] == "done"))
            self._track_info = {"active": False,
                                "distance": self._track_info.get("distance"),
                                "lost_s": None, "searching": False, "search_s": None,
                                "phase": "idle", "search_index": None,
                                "search_laps": None,
                                "want": None, "locked_s": None, "reacquire": False}

    # -------------------------------------------------------------------------
    #  巡航
    # -------------------------------------------------------------------------

    def _map_to_local(self, mx, my):
        """map 座標（ENU：x 東、y 北）→ PX4 本地座標（NED：x 北、y 東）。

        PX4 的本地原點是「生成／起飛位置」，在 map 上的位置由
        local_origin_map 參數給（和 px4_bridge 的 odom_origin 是同一個值）。

            ned_x(北) = map_y - origin_y
            ned_y(東) = map_x - origin_x

        ⚠️ 這個轉換和 px4_tf_node 的 NED→ENU 必須互為反函數。
        2026-10-08 用倉庫的搜尋環線對照過：
            spawn (map) = (-14.0, -9.0)
            map (+5.6, -9.0) → NED 北 +0.0 東 +19.6     （5.6-(-14) = 19.6）
            map (+5.6, +9.0) → NED 北 +18.0 東 +19.6
        座標轉換的雙向一致性是這個專案踩過最多次的雷，不要憑印象改。

        回傳 None 表示 local_origin_map 沒設定或格式不對。
        """
        parts = [v for v in self.local_origin_map.replace(" ", "").split(",") if v]
        if len(parts) < 2:
            return None
        try:
            ox, oy = float(parts[0]), float(parts[1])
        except ValueError:
            return None
        return (my - oy, mx - ox)

    def _local_to_map(self):
        """飛機現在的位置換成 map 座標，回傳 (x, y, 機頭角度)。

        是 _map_to_local 的反函數：
            map_x = origin_x + ned_east(y)
            map_y = origin_y + ned_north(x)

        機頭角度換成「map 的 x 軸（東）起算、逆時針為正」的度數 ——
        前端畫箭頭用的是螢幕座標，這樣換算最直接。
        PX4 的 heading 是 NED 的航向（北起算、順時針為正），
        兩者差一個 90° 和一個反號。

        算不出來（沒有 local_origin_map、位置無效）就回 None ——
        儀表板會顯示「位置未知」而不是畫在錯的地方。
        """
        if self.lpos is None or not self.lpos.xy_valid:
            return None
        parts = [v for v in self.local_origin_map.replace(" ", "").split(",") if v]
        if len(parts) < 2:
            return None
        try:
            ox, oy = float(parts[0]), float(parts[1])
        except ValueError:
            return None
        deg = 90.0 - math.degrees(float(self.lpos.heading))
        return (ox + float(self.lpos.y), oy + float(self.lpos.x),
                (deg + 180.0) % 360.0 - 180.0)

    def _ground_speed(self):
        """水平地速（m/s）。垂直速度對倉庫巡檢沒意義，所以不算進去。

        ⚠️ 上限要知道才看得懂儀表：Nav2 導航 1.2、跟隨 1.4（橋夾到 1.5），
           但**搜尋段的 DO_REPOSITION 實測到 3.6~4.4 m/s**
           （2026-10-19 的飛行記錄），所以儀表的刻度要做到 5。
        """
        if self.lpos is None or not self.lpos.xy_valid:
            return None
        return math.hypot(float(self.lpos.vx), float(self.lpos.vy))

    # -------------------------------------------------------------------------
    #  站位方向 —— 治「左右搖擺」和「鎖定時抽動」
    # -------------------------------------------------------------------------

    @staticmethod
    def _blend_standoff_dir(vx, vy, vmag, dx, dy, d):
        """算「從車指向站位點」的單位向量，站位點 = 車 + b × dist。

        兩個來源按車速權重**連續混合**：
            b_vel = −v̂             車在動 → 跟在它後面
            b_los = −(車−飛機)/d    車沒在動 → 維持現在的視線方向
            w     = min(1, vmag / TRACK_MIN_TARGET_SPEED)
            b     = normalize(w·b_vel + (1−w)·b_los)

        🔴 為什麼不是「超過門檻用前者、否則用後者」的硬切換：
           車速估計在門檻上下抖的時候，硬切換會讓站位點在兩套算法之間跳。
           2026-10-19 的 log（鎖定後第 3~5 拍，dist = 2.12 m）：
               車向 -149° 0.23m/s → 站位 北+4.3  差 4.42 m
               車向 太慢(0.14)    → 站位 北+0.2  差 0.22 m
               車向 +170° 0.39m/s → 站位 北+4.2  差 3.32 m
           半秒內站位點跳了 3.9 m —— 那就是「剛看到目標時抽動一下」。
           連續混合在門檻附近沒有斷點，所以不會跳。

        兩個來源都算不出來（車速是 0 而且飛機正好在車的正上方）回 None。
        """
        if vmag <= 1e-9 and d <= 1e-6:
            return None
        w = min(1.0, vmag / TRACK_MIN_TARGET_SPEED) if vmag > 1e-9 else 0.0
        bx = by = 0.0
        if vmag > 1e-9:
            bx -= w * vx / vmag
            by -= w * vy / vmag
        if d > 1e-6:
            bx -= (1.0 - w) * dx / d
            by -= (1.0 - w) * dy / d
        bn = math.hypot(bx, by)
        # 混合後長度塌成 0 = 兩個來源正好相反（車**朝著飛機**開過來，
        # 而且速度剛好在門檻的一半）。這時候沒有合理的中間方向，
        # 用車速那一邊 —— 它才是「車要往哪去」的資訊。
        if bn <= 1e-9:
            return (-vx / vmag, -vy / vmag) if vmag > 1e-9 else None
        return (bx / bn, by / bn)

    @staticmethod
    def _filter_dir(prev, raw, dt):
        """站位方向的一階低通。prev 是 None（這一段跟隨的第一拍）就直接用 raw。

        🔴 要濾**向量**再正規化，不能濾角度。
           角度在 ±180° 會跳：車往南走時原始方向在 +179° 和 −179° 之間跳，
           濾角度會算出 0°（正北）—— 飛機會直接衝到車前面去。
           向量內插沒有這個問題。

        為什麼需要（2026-10-19 從 log 量到的，車是直線往北走、真實方向 0°）：
            車向 -12° → 左-0.70      車向 +14° → 左+0.75
        方向每半秒在 ±13° 之間翻邊，橫向速度指令一比一跟著翻。
        τ = TRACK_DIR_TAU_S = 1.5 秒，壓得掉 1 秒週期的翻邊，
        又遠遠跟得上車真正的轉彎（環線四個轉角之間隔 20 秒以上）。
        """
        if prev is None:
            return raw
        a = 1.0 - math.exp(-max(dt, 1e-3) / TRACK_DIR_TAU_S)
        fx = prev[0] + (raw[0] - prev[0]) * a
        fy = prev[1] + (raw[1] - prev[1]) * a
        fn = math.hypot(fx, fy)
        # 濾完長度塌成 0 = 新舊方向正好相反（車 180° 大轉彎）。
        # 沒有「中間方向」可言，直接跳到新的 —— 濾波不該讓飛機卡在
        # 一個哪邊都不是的方向上。
        return (fx / fn, fy / fn) if fn > 1e-6 else raw

    # -------------------------------------------------------------------------
    #  禁飛方框 —— 跟隨段唯一的硬性保險
    # -------------------------------------------------------------------------

    def _parse_no_fly(self, text):
        """剖析 no_fly_boxes 參數 → PX4 本地 NED 的禁飛方框清單。

        回傳 [(n_min, e_min, n_max, e_max, 名稱), ...]，**餘裕已經膨脹進去**。
        沒設定、格式不對、或沒有 local_origin_map 都回空清單
        （＝跟隨段沒有這道保險，啟動訊息會說出來）。

        輸入是 map 座標（ENU）攤平的一串，每 4 個一組：
            x_min, y_min, x_max, y_max

        ⚠️ **兩個軸會互換**。換算關係見 _map_to_local：
                ned_north = map_y - origin_y     ned_east = map_x - origin_x
            所以 map 的 **y** 區間變成 north 區間、map 的 **x** 區間變成 east
            區間。寫成 (n_min, e_min) = (my0, mx0) 很容易看漏，
            這裡刻意用 _map_to_local 轉兩個角點而不是自己減，
            就是為了不要出現第二份座標換算程式碼。
        """
        nums = []
        for tok in str(text).replace(" ", "").split(","):
            if not tok:
                continue
            try:
                nums.append(float(tok))
            except ValueError:
                self.get_logger().error(
                    f"no_fly_boxes 裡有不是數字的東西：{tok!r} → 整組不用，"
                    "跟隨段沒有禁飛保險")
                return []
        if not nums:
            return []
        if len(nums) % 4:
            self.get_logger().error(
                f"no_fly_boxes 有 {len(nums)} 個數字，不是 4 的倍數 → 整組不用，"
                "跟隨段沒有禁飛保險")
            return []
        m = float(self.no_fly_margin)
        out = []
        # ⚠️ map 座標的原始方框要另外留一份給儀表板的地圖畫。
        #    不能拿下面算好的 NED 版本去畫 —— 那個已經膨脹過餘裕、而且軸是反的。
        #    這樣場地幾何只有一個來源（warehouse_geometry.yaml），
        #    改場景重跑 gen_warehouse.py 之後地圖會自動跟著對。
        self._scene_boxes = []
        for i in range(0, len(nums), 4):
            mx0, my0, mx1, my1 = nums[i:i + 4]
            mx0, mx1 = min(mx0, mx1), max(mx0, mx1)
            my0, my1 = min(my0, my1), max(my0, my1)
            self._scene_boxes.append([mx0, my0, mx1, my1])
            # 兩個角點各自轉換。map→NED 只是平移（沒有旋轉），所以
            # 「map 的最小角」轉完還是「NED 的最小角」，不用再 min/max 一次。
            lo = self._map_to_local(mx0, my0)
            hi = self._map_to_local(mx1, my1)
            if lo is None or hi is None:
                self.get_logger().error(
                    "有給 no_fly_boxes 但 local_origin_map 是空的 → 無法換算成 "
                    "PX4 本地座標，跟隨段沒有禁飛保險")
                return []
            out.append((lo[0] - m, lo[1] - m, hi[0] + m, hi[1] + m,
                        f"map x[{mx0:+.1f},{mx1:+.1f}] y[{my0:+.1f},{my1:+.1f}]"))
        return out

    @staticmethod
    def _box_entry_axis(p, v, box, horizon):
        """掃掠線段 p → p + v·horizon 會不會進到 box 裡？會的話回「先碰到的那一軸」。

        p、v 是 (北, 東)，box 是 (lo_n, lo_e, hi_n, hi_e, …)，回傳 0、1 或 None。

        標準的 AABB slab test：每一軸算出「位於該軸區間內」的時間區間
        [near_i, far_i]，取 near = max(near_i)、far = min(far_i)。
        兩者還有交集就表示線段會穿進方框，而 **near 最大的那一軸**就是會先
        碰上的那一面 —— 砍掉那一軸的速度，平行那一面的分量就留下來了。

        ⚠️ 是測「整條線段有沒有穿進去」而不是「終點在不在裡面」——
           後者在速度夠快時會直接穿過薄的方框而測不到。
        ⚠️ p 已經在方框裡的情況**這裡不處理**（會回傳一個軸），
           由呼叫端先判斷，否則連「往外飛」都會被砍掉。
        """
        near, far, axis = -math.inf, math.inf, None
        for i in (0, 1):
            lo, hi = box[i], box[i + 2]
            if abs(v[i]) < 1e-9:
                # 這一軸不動：在區間外就永遠穿不進去，在區間內就不構成限制
                if p[i] < lo or p[i] > hi:
                    return None
                continue
            a = (lo - p[i]) / v[i]
            b = (hi - p[i]) / v[i]
            if a > b:
                a, b = b, a
            if a > near:
                near, axis = a, i
            far = min(far, b)
        if axis is None or near > far or near >= horizon or far <= 0.0:
            return None
        return axis

    def _clamp_no_fly(self, vn, ve):
        """把速度指令夾成「照這個速度再飛 no_fly_lookahead_s 秒也進不了禁飛方框」。

        回傳 (vn, ve, 擋住它的方框名稱 or None)。

        ---------------------------------------------------------------------
        這是什麼、不是什麼
        ---------------------------------------------------------------------
        **是**：「不要把飛機開進已知的牆裡」的硬性下限。方框來自
                warehouse_geometry.yaml（四排貨架 ＋ 四面外牆內緣）。
        **不是**：避障系統。目標車、別的飛機、臨時生成的模型都不在方框裡。
                真正的避障是 Nav2，而跟隨段刻意不走 Nav2
                （DO_REPOSITION/cmd_vel 都繞過 costmap）。

        為什麼需要：2026-10-19 摔機。站位點算錯方向，飛機以滿速 1.40 m/s
        往西開了 1.9 秒撞上貨架，而整個跟隨迴圈當時唯一的位置保險是
        PATROL_MAX_RANGE_M（50 m）—— 在 36×24 m 的倉庫裡等於沒有。

        ---------------------------------------------------------------------
        只砍法線方向，不砍整個速度
        ---------------------------------------------------------------------
        砍掉整個速度的話，車沿著走道走、飛機一靠近貨架就整個停住 → 直接跟丟。
        只砍「朝著那一面」的分量，飛機還能**貼著貨架面沿走道滑行**跟著車。

        ⚠️ 必須在 NED 做。貨架面是垂直於 map 軸的平面，而機體的「左右」
           會隨機頭轉而指向不同的實體方向 —— 夾機體座標的左右是錯的。
        """
        if not self._no_fly or self.lpos is None:
            return vn, ve, None
        p = (float(self.lpos.x), float(self.lpos.y))
        horizon = max(float(self.no_fly_lookahead_s), 1e-3)
        v = [float(vn), float(ve)]
        blocked = None
        # 最多三輪：一輪最多把一個軸歸零，兩個軸都歸零之後進不了任何方框，
        # 所以一定會收斂。多輪是因為砍掉一個軸之後可能換成另一個方框擋在前面。
        for _ in range(3):
            hit = None
            for box in self._no_fly:
                if all(box[i] <= p[i] <= box[i + 2] for i in (0, 1)):
                    # 已經在**膨脹後**的方框裡（餘裕被吃掉了，實體障礙還有一段
                    # 距離）。這時候不能照常夾 —— 連「往外飛」都會被砍掉，
                    # 飛機就永遠出不來。只砍「往更裡面」的分量。
                    for i in (0, 1):
                        out = (box[i] - p[i]
                               if (p[i] - box[i]) < (box[i + 2] - p[i])
                               else box[i + 2] - p[i])
                        if v[i] * out < 0.0:
                            v[i] = 0.0
                            hit = f"{box[4]}（已在框內，只准往外）"
                    continue
                axis = self._box_entry_axis(p, v, box, horizon)
                if axis is not None:
                    v[axis] = 0.0
                    hit = box[4]
                    break
            if hit is None:
                break
            blocked = hit
        return v[0], v[1], blocked

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

    # -------------------------------------------------------------------------
    #  控制權互斥（L3，2026-10-07）
    # -------------------------------------------------------------------------

    def _cancel_all_motion(self, why, keep=()):
        """取消所有「會持續驅動飛機」的動作，keep 裡列的除外。

        回傳實際取消掉的名字集合（空集合 = 本來就沒有東西在動）。

        ⚠️ 新增一種會持續驅動飛機的動作時，一定要加進這個函式 ——
        這是唯一的節流點。以前是每個 handler 各自呼叫 _cancel_patrol /
        _cancel_track / _cancel_nav 的不同組合，結果就漂掉了：
        2026-10-07 稽核發現 _emergency_land、_srv_track、_srv_patrol
        各漏了一到兩個，而「兩個控制權持有者同時在動」不會印任何錯誤訊息。

        ⚠️ 不要改成 `or` 短路。三個都必須被求值，否則第一個回 True 之後
        後面兩個就不會被取消了。
        """
        done = set()
        if "patrol" not in keep and self._cancel_patrol(why):
            done.add("patrol")
        if "track" not in keep and self._cancel_track(why):
            done.add("track")
        if "nav" not in keep and self._cancel_nav(why):
            done.add("nav")
        return done

    def _settle_after_cancel(self, canceled):
        """取消之後把飛機交回「沒有人在驅動」的狀態。

        兩類動作要用不同手段收尾，這是 L3 的核心：

        patrol          走 PX4 內建模式，最後送出去的目標點還留在 PX4 裡，
                        要用 _pause_here()（全 NaN 的 DO_REPOSITION）清掉。

        track           **兩種都是**（2026-10-16 起）：搜尋階段走 DO_REPOSITION、
                        跟隨階段走 cmd_vel/offboard。所以兩種收尾都要做 ——
                        _track_worker 的 finally 會呼叫 release_hold 交還 offboard，
                        這裡再補一次 _pause_here() 清掉搜尋階段留下的航點。
                        多做一次是安全的（_pause_here 對沒有目標點的狀態無害），
                        漏做才危險。

        nav（Nav2）     走 offboard。取消 action 只讓 Nav2 停發 cmd_vel，
                        但 cmd_vel_to_px4_node 的 ensureOffboard()
                        （cmd_vel_to_px4_node.cpp:444-458）只要 cmd_timeout_s
                        內收到過 cmd_vel 就會主動送 DO_SET_MODE 把 PX4 拉回
                        OFFBOARD —— 會把我們接著要設的 AUTO_LOITER 推翻。

        2026-10-07 實測到的競態（l2b 那一輪的 bridge log）：
            946.916  bridge: 有人在送 cmd_vel 但飛機不在 offboard → 切回 OFFBOARD
            946.948  controller_server: Goal was canceled. Stopping the robot.
            947.467  bridge: 超過 0.50 秒沒收到 cmd_vel，已銷毀 publisher
        那次 hover 之所以成功，只是因為 Nav2 剛好在 0.5 秒後就停了、bridge 跟著
        放手。cmd_vel 要是不停（L4 的視覺追蹤就是這樣），hover 會輸。

        所以取消到 nav 時要等過 bridge 的逾時再往下走。
        """
        if canceled & {"patrol", "track"}:
            self._pause_here()
        # track 的跟隨階段和 nav 一樣走 offboard，都要等 bridge 放手之後
        # 才能把飛機交給別的模式（見上面的說明）。
        if canceled & {"nav", "track"}:
            # 比 bridge 的 cmd_timeout_s 預設 0.5 秒多一點餘裕。
            # 這裡用 sleep 是可以的 —— 服務在互斥群組，訂閱在可重入群組，
            # 睡著期間狀態還是在更新（和 _send 等 ack 同一個道理）。
            time.sleep(self.cancel_settle_s)

    def _send_cmd_vel(self, forward=0.0, left=0.0, yawrate=0.0):
        """發一則 cmd_vel 給 cmd_vel_to_px4_node。

        ⚠️ 座標是**機體 FLU**（x 前、y 左、z 上），不是 NED ——
        cmd_vel_to_px4_node.cpp:178 印的是「前 %f 左 %f 轉 %f」。
        用 NED 丟進去的話飛機會往莫名其妙的方向走，而且不會報錯。

        ⚠️ 高度不在這裡控制。bridge 自己用 flight_altitude 參數定高
        （cmd_vel_to_px4_node.cpp:243-246「收到 cmd_vel，開始接管（定高 %.2f m）」），
        所以 linear.z 填了也沒用。
        """
        m = Twist()
        m.linear.x = float(forward)
        m.linear.y = float(left)
        m.linear.z = 0.0
        m.angular.z = float(yawrate)
        self.cmd_vel_pub.publish(m)

    def _release_bridge_hold(self):
        """請 cmd_vel_to_px4_node 放手，把控制權交還給 PX4 的 AUTO 模式。

        ⚠️ 模擬裡不呼叫也會自己放手（hold_on_timeout 預設 false，逾時走
        「銷毀 publisher」那條路），但**室內實機必須呼叫** ——
        那邊 hold_on_timeout 要設 true（否則交接時 setpoint 中斷、飛機會掉），
        而 true 的時候 bridge 會一直維持原地定點並把 PX4 鎖在 OFFBOARD，
        接下來的 DO_REPOSITION 會被它推翻，**而且不會有任何錯誤訊息**。

        呼叫之後還能重新接管：release_requested_ 是一次性的，
        收到新的 cmd_vel 就會清掉（cmd_vel_to_px4_node.cpp:231-234）。

        服務不在也不當成錯誤 —— 這個節點在沒有 bridge 的場景也要能跑。
        """
        if not self._release_cli.service_is_ready():
            return False
        try:
            fut = self._release_cli.call_async(Trigger.Request())
            # 不等結果：服務在可重入群組，而我們只需要「請求送出去了」。
            # 真正保證交棒的是底下那段等待（比 cmd_timeout_s 長）。
            del fut
            return True
        except Exception as e:                       # noqa: BLE001
            self.get_logger().warn(f"[track] release_hold 送不出去：{e}")
            return False

    def _ned_to_body(self, dn, de):
        """世界 NED 的位移 → 機體 FLU 的（前, 左）。

        機體前方在 NED 是 (cos h, sin h)、右方是 (-sin h, cos h)，
        而 cmd_vel 的 y 是**左**，所以左 = -右。
        """
        h = float(self.lpos.heading)
        c, sn = math.cos(h), math.sin(h)
        forward = dn * c + de * sn
        right = -dn * sn + de * c
        return forward, -right

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
        # 2026-10-07：原本只取消 patrol，連 track 都沒取消（Nav2 之前就有的洞）。
        self._settle_after_cancel(self._cancel_all_motion("收到新的巡航指令"))
        self._end_task(False, "被新的巡航指令取代")

        # 高度：0 代表維持現在的高度
        height = alt if alt != 0.0 else (self._altitude() or 0.0)
        alt_amsl = float(self.lpos.ref_alt) + height

        # 每個點抓 30 秒，再加 60 秒裕度。逾時只是讓「等待完成」不要無限等。
        self._begin_task(name, 60.0 + 30.0 * len(xs) * loops)
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
                    # 機頭朝「這一段的行進方向」（2026-10-08 新增 param4）。
                    #
                    # ⚠️ 原本沒帶 param4，預設是 NaN，PX4 收到 NaN 就是「不控制偏航」
                    #    → 機頭留在原地不動。那對巡航本身沒差，但相機裝在機頭上：
                    #    俯角 67° 的相機只看得到前方 0~3.6 m 的地面，機頭卡住的話
                    #    看的是「出發時的方向」而不是「要去的方向」，
                    #    用巡航搜尋目標時等於閉著眼睛飛。
                    #
                    # param4 的單位是弧度、世界 NED（navigator_main.cpp:316
                    # `rep->current.yaw = cmd.param4;`）。本地座標 x 是北、y 是東，
                    # 所以方位角＝atan2(東分量, 北分量)。
                    # _track_worker 用的是同一個寫法。
                    # _distance_to 在位置無效時回傳 0.0（不是 None），
                    # 所以「> PATROL_ARRIVE_M」已經涵蓋「位置有效」這個條件。
                    if self._distance_to(px, py) > PATROL_ARRIVE_M:
                        yaw = math.atan2(py - float(self.lpos.y), px - float(self.lpos.x))
                    else:
                        # 已經在點上（或位置無效）→ 不要硬塞一個方位，
                        # 給 NaN 讓 PX4 維持現狀，免得原地亂轉。
                        yaw = NAN
                    r = self._send(VehicleCommand.VEHICLE_CMD_DO_REPOSITION,
                                   # param1 是水平速度，-1 代表用 PX4 預設
                                   p1=(speed if speed != 0.0 else -1.0),
                                   p2=REPOSITION_CHANGE_MODE,
                                   p4=yaw,
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
        if self._task["name"] == "patrol":
            self._end_task(ok, message)
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

        # ⚠️ 2026-10-07（L3）：導航中改高度必須先取消導航。
        #    看起來「高度和水平導航是兩回事」，但實作上不是：
        #      ① Nav2 是 2D 的，高度由 cmd_vel_to_px4_node 的 flight_altitude 持有
        #      ② DO_CHANGE_ALTITUDE 會把 PX4 切成 AUTO_LOITER（Commander.cpp:782），
        #         而 bridge 的 ensureOffboard() 會再把它要回 OFFBOARD → 互搶
        #    所以行為定義成「新指令接手」，和這個節點其他動作一致。
        #    不做成「導航中拒絕改高度」是因為那樣 Node-RED 會多一個要處理的錯誤，
        #    而使用者的意圖很清楚。訊息裡會講明導航被取消了，不是靜默。
        canceled = self._cancel_all_motion(f"改用 {name}")
        if canceled:
            self._end_task(False, f"被 {name} 取代")
        self._settle_after_cancel(canceled)

        alt_amsl = float(self.lpos.ref_alt) + h
        now = self._altitude()
        now_note = "目前高度未知" if now is None else f"目前 {now:.2f} m"

        r = self._send(VehicleCommand.VEHICLE_CMD_DO_CHANGE_ALTITUDE, p1=alt_amsl)
        if r != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            return self._finish(response, name, False, f"調整高度被拒絕：{self._ack_text(r)}")
        self._begin_task(name, 60.0, target=h)
        note = f"已送出：{now_note} → {h:.2f} m"
        if canceled:
            note += "（已取消 " + "、".join(sorted(canceled)) + "）"
        return self._finish(response, name, True, note)


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
