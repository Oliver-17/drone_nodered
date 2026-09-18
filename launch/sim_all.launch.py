# =============================================================================
#  sim_all.launch.py — 一次開「模擬」：XRCE agent + PX4 SITL（含 Gazebo）
#
#  用法：
#      ros2 launch drone_nodered sim_all.launch.py                 # 開 Gazebo 視窗
#      ros2 launch drone_nodered sim_all.launch.py headless:=true  # 不開視窗，只算物理
#      ros2 launch drone_nodered sim_all.launch.py agent:=false    # agent 已經自己開著
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

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
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

        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", LaunchConfiguration("agent_port")],
            # agent 每來一個 topic 就印一行，會把 drone_api_node 的訊息洗掉 → 只寫進 log 檔
            output="log",
            condition=IfCondition(LaunchConfiguration("agent")),
        ),
        ExecuteProcess(
            cmd=["bash", sitl],
            additional_env={
                # 關鍵：沒有這個，腳本開完就結束，launch 會以為模擬結束
                "FOREGROUND": "1",
                # HEADLESS 是「有設就生效」，所以 false 時要傳空字串而不是 "false"
                "HEADLESS": PythonExpression(["'1' if '", headless, "'.lower() in ('true','1') else ''"]),
            },
            output="screen",
            # 給腳本時間跑收尾（關 PX4、關 Gazebo 最多 10 秒）
            sigterm_timeout="20",
            sigkill_timeout="25",
        ),
    ])
