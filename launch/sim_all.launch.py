# =============================================================================
#  sim_all.launch.py — 一次開「模擬」：XRCE agent + PX4 SITL（含 Gazebo）
#
#  用法：
#      ros2 launch drone_nodered sim_all.launch.py                 # 開 Gazebo 視窗
#      ros2 launch drone_nodered sim_all.launch.py headless:=true  # 不開視窗，只算物理
#      ros2 launch drone_nodered sim_all.launch.py agent:=false    # agent 已經自己開著
#      ros2 launch drone_nodered sim_all.launch.py target_speed:=0 # 目標車不要動
#      ros2 launch drone_nodered sim_all.launch.py world:=empty model:=x500   # 回到空地圖
#
#  為什麼這兩個要跟底座（nodered_all.launch.py）分開：
#    1. 實機時 PX4 是飛控板上的韌體，根本沒有東西要開；agent 則跑在樹莓派上、走序列埠，
#       跟這裡的 udp4 不一樣。這支整包只有模擬用得到。
#    2. agent 和 PX4 要一起活。agent 掛掉時這條連線就廢了，綁在同一支 launch，
#       一次重開就是兩個乾淨重來，不會出現「agent 是舊的、PX4 是新的」半連線狀態。
#
#  順序：agent 先開（ExecuteProcess 由上而下啟動），PX4 起來時才連得上。
#        agent 晚到也不會怎樣，PX4 會重試，只是腳本那句「已連上 XRCE Agent」會逾時。
#
#  收工：Ctrl+C。腳本是用 FOREGROUND=1 開的，會自己收掉 PX4 和 Gazebo
#        （原因見 start_empty_sitl.sh 檔頭）。
# =============================================================================

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    sitl = os.path.join(get_package_prefix("drone_nodered"), "lib", "drone_nodered",
                        "start_empty_sitl.sh")
    headless = LaunchConfiguration("headless")

    return LaunchDescription([
        DeclareLaunchArgument("agent", default_value="true",
                              description="要不要一起開 MicroXRCEAgent"),
        DeclareLaunchArgument("agent_port", default_value="8888",
                              description="PX4 SITL 預設就是連 udp4 8888"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="true = 不開 Gazebo 視窗（物理照算）"),

        # --- 世界和目標車 ---------------------------------------------------------
        # 預設就開「有洋紅目標車」的世界 + 下傾 45° 相機的機體，這樣按一顆按鈕
        # 出來的地圖就能直接做視覺追蹤，不用再自己下環境變數。
        # 想回到最單純的空地圖：world:=empty model:=x500（沒有相機，也不會有車）
        DeclareLaunchArgument("world", default_value="empty_target",
                              description="Gazebo 世界（empty_target 裡面有目標車）"),
        DeclareLaunchArgument("model", default_value="x500_depth_tilt",
                              description="機體："
                                          "x500_depth_tilt 相機在機身上方、下傾 45°（舊的追蹤 demo）／"
                                          "x500_cam_down 相機在機腹支架上、下傾 67°、無盲區（倉庫走道追蹤）／"
                                          "x500 沒有相機"),
        # 預設留空 = 讓 start_empty_sitl.sh 依世界決定：
        #   empty_target → circle（空地，繞圈讓飛機一直有得追）
        #   warehouse    → aisle （有貨架，繞圈會直接撞進去）
        # 寫死 "circle" 的話會蓋掉那個判斷，倉庫裡的車就會往貨架衝。
        DeclareLaunchArgument("target_mode", default_value="",
                              description="目標車怎麼走：留空＝依世界決定／"
                                          "circle 繞圈／line 直線來回／aisle 走道巡迴"),
        DeclareLaunchArgument("target_speed", default_value="0.8",
                              description="目標車速度 m/s（0 = 不動）"),
        DeclareLaunchArgument("target_radius", default_value="5",
                              description="繞圈半徑 m（0.8 m/s 繞一圈約 39 秒）"),
        # ⚠️ 這個一定要在外層宣告，不能寫在下面 include 的 launch_arguments 裡。
        #    launch_arguments 是依序套用的 SetLaunchConfiguration，而替換是**延遲求值**：
        #    下面會先把 model 改寫成「<名字>_0」（topic 路徑要那個 _0），
        #    之後才輪到 camera_pitch 的運算式求值 —— 那時它讀到的 model 已經是
        #    x500_cam_down_0，比對 'x500_cam_down' 就不成立，靜靜地退回 45。
        #    2026-10-07 實測：外層 LogInfo 印 67.0，但節點收到 45。
        #    放在外層宣告，預設值在 model 還是原名的時候就算完，而且使用者仍可覆寫。
        DeclareLaunchArgument(
            "camera_pitch",
            default_value=PythonExpression(
                ["'67.0' if '", LaunchConfiguration("model"),
                 "' == 'x500_cam_down' else '45.0'"]),
            description="相機俯角（度）。預設跟著 model 走，不用自己記兩者要配對"),
        # 用哪個偵測器。color 找洋紅方盒、yolo 認真車外觀 ——
        # 要和世界裡的目標對得上（gen_warehouse.py 的 --target 決定世界裡是哪一台）。
        DeclareLaunchArgument("detector", default_value="color",
                              choices=["color", "yolo"],
                              description="偵測器：color 找洋紅、yolo 認真車"),
        DeclareLaunchArgument("vision", default_value="true",
                              description="要不要一起開視覺（顏色偵測 → 目標世界座標）"),

        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", LaunchConfiguration("agent_port")],
            # agent 每來一個 topic 就印一行，會把 drone_api_node 的訊息洗掉 → 只寫進 log 檔
            output="log",
            # ⚠️ 刻意不設 sigterm_timeout / sigkill_timeout（2026-10-07 實測過）。
            #
            # MicroXRCEAgent 在「沒有控制終端」的那條路徑上不理 SIGINT，所以曾經
            # 懷疑要靠這兩個參數讓 launch 升級到 SIGTERM。但那兩個參數只在
            # launch 自己跑 shutdown 流程時才會被讀到 —— 而 ros2 launch（Humble）
            # 沒有控制終端時收到 SIGINT 根本不進 shutdown，參數永遠不會被用上。
            # 加了只是看起來有做事。
            #
            # 實測結果（詳見 docs/warehouse_runbook.md 的「關機」）：
            #   終端機 Ctrl-C        → launch 進 shutdown，agent 跟著死（exit -2），全乾淨
            #   systemd 群組 SIGTERM → agent 吃 SIGTERM，全乾淨
            #   kill -INT 單一 PID   → launch 完全沒反應（和 agent 無關）
            # 兩條真實使用路徑都是乾淨的，所以這裡不需要任何訊號設定。
            condition=IfCondition(LaunchConfiguration("agent")),
        ),
        ExecuteProcess(
            cmd=["bash", sitl],
            additional_env={
                # 關鍵：沒有這個，腳本開完就結束，launch 會以為模擬結束
                "FOREGROUND": "1",
                # HEADLESS 是「有設就生效」，所以 false 時要傳空字串而不是 "false"
                "HEADLESS": PythonExpression(["'1' if '", headless, "'.lower() in ('true','1') else ''"]),
                "WORLD": LaunchConfiguration("world"),
                "SIM_MODEL": LaunchConfiguration("model"),
                "TARGET_MODE": LaunchConfiguration("target_mode"),
                "TARGET_SPEED": LaunchConfiguration("target_speed"),
                "TARGET_RADIUS": LaunchConfiguration("target_radius"),
            },
            output="screen",
            # 給腳本時間跑收尾（關 PX4、關 Gazebo 最多 10 秒）
            sigterm_timeout="20",
            sigkill_timeout="25",
        ),

        # --- 視覺（顏色偵測 → 目標世界座標）---------------------------------------
        # 為什麼跟模擬綁在一起，而不是放在底座（nodered_all）：
        #   影像是從 Gazebo 橋接過來的，沒有模擬就沒有畫面；模擬關掉，這些節點
        #   留著也只是空轉。實機的相機是另一條路（不同的 launch），不受影響。
        # 延後 5 秒再開：parameter_bridge 在 gz 還沒起來時會連不上而空等，
        #   晚一點開 log 乾淨得多（PX4 完全就緒約 20 秒，這裡不用等那麼久）。
        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory("drone_nodered"),
                    "launch", "vision.launch.py")),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    # ⚠️ 模型在世界裡的名字會多一個「_0」（PX4 的第 0 架）。
                    #    影像 topic 是 /world/<世界>/model/<名字>/…，少了 _0 就訂不到。
                    "model": PythonExpression(
                        ["'", LaunchConfiguration("model"), "' + '_0'"]),
                    # 俯角：值在外層就算好了（見上面 DeclareLaunchArgument 的說明），
                    # 這裡只是把它轉交進去。不能在這裡用 LaunchConfiguration("model")
                    # 重算 —— 上一行已經把 model 改寫成「_0」版本了。
                    "camera_pitch": LaunchConfiguration("camera_pitch"),
                    # detector 一路轉交到 vision.launch.py。那邊會依它挑偵測器節點，
                    # 以及 target_world 的 target_height_m（兩種目標的中心高度不同）。
                    "detector": LaunchConfiguration("detector"),
                }.items(),
            ),
        ], condition=IfCondition(LaunchConfiguration("vision"))),
    ])
