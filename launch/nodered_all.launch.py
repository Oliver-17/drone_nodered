# =============================================================================
#  nodered_all.launch.py — 一次開「底座」：rosbridge + drone_api_node + Node-RED
#
#  用法：
#      ros2 launch drone_nodered nodered_all.launch.py                  # 三個都開
#      ros2 launch drone_nodered nodered_all.launch.py nodered:=false   # Node-RED 已經自己開著
#      ros2 launch drone_nodered nodered_all.launch.py rosbridge:=false drone_api:=false  # 只開 Node-RED
#
#  這裡的東西「模擬和實機都一樣」，所以 PX4 SITL、Gazebo、XRCE agent 都不在這支裡面
#  （那些只有模擬才有，放在 sim_all.launch.py）。實機時：
#      樹莓派 → rosbridge:=true drone_api:=true nodered:=false
#      筆電   → rosbridge:=false drone_api:=false nodered:=true
#              （Node-RED 積木的 ROSBRIDGE_URL 改指到 ws://<樹莓派IP>:9090）
#
#  為什麼要有三個開關：上面那兩行就是原因 —— 同一份 launch 檔，兩台電腦各開一半。
#
#  ⚠️ Node-RED 預設用你自己的 ~/.node-red（流程、帳號密碼都在那）。
#     要用乾淨的環境測試時傳 nodered_user_dir:=/tmp/xxx nodered_port:=1881。
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _nodered(context, *args, **kwargs):
    """組 node-red 的指令列。

    用 OpaqueFunction 而不是直接寫 ExecuteProcess，是因為 --userDir 是「有才加」的參數：
    傳空字串給它 node-red 會當成目錄名稱，開在一個叫 '' 的地方。
    要判斷字串空不空，就得先把 LaunchConfiguration 解析成真正的字串，只有這裡做得到。
    """
    if LaunchConfiguration("nodered").perform(context).lower() not in ("true", "1"):
        return []

    cmd = ["node-red", "--port", LaunchConfiguration("nodered_port").perform(context)]
    user_dir = LaunchConfiguration("nodered_user_dir").perform(context)
    if user_dir:
        cmd += ["--userDir", user_dir]

    return [ExecuteProcess(
        cmd=cmd,
        # 不印到畫面：Node-RED 自己很會講話，會把 drone_api_node 的訊息洗掉。
        # 要看的話開瀏覽器進編輯器，或去 launch 的 log 檔。
        output="log",
        # Node-RED 收到 SIGINT 會正常關閉（它自己的服務檔 nodered.service 也是用 SIGINT）
        sigterm_timeout="10",
    )]


def generate_launch_description():
    ns = LaunchConfiguration("namespace")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="MAV1",
                              description="PX4 的 namespace，服務也會放在這底下"),
        DeclareLaunchArgument("target_system", default_value="1",
                              description="PX4 的 MAV_SYS_ID，多機時是 instance+1"),
        DeclareLaunchArgument("min_takeoff_altitude", default_value="0.5"),
        DeclareLaunchArgument("max_takeoff_altitude", default_value="5.0",
                              description="起飛高度上限（公尺）。室內實機建議 1.5"),

        DeclareLaunchArgument("rosbridge", default_value="true"),
        DeclareLaunchArgument("drone_api", default_value="true"),
        DeclareLaunchArgument("nodered", default_value="true"),

        DeclareLaunchArgument("rosbridge_address", default_value="127.0.0.1",
                              description="127.0.0.1 只給本機；跨電腦（實機）要 0.0.0.0"),
        DeclareLaunchArgument("nodered_port", default_value="1880"),
        DeclareLaunchArgument("nodered_user_dir", default_value="",
                              description="留空＝用 ~/.node-red；測試時才指定"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("nodered_ros2"), "launch", "rosbridge.launch.py")),
            launch_arguments={"address": LaunchConfiguration("rosbridge_address")}.items(),
            condition=IfCondition(LaunchConfiguration("rosbridge")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("drone_nodered"), "launch", "drone_api.launch.py")),
            launch_arguments={
                "namespace": ns,
                "target_system": LaunchConfiguration("target_system"),
                "min_takeoff_altitude": LaunchConfiguration("min_takeoff_altitude"),
                "max_takeoff_altitude": LaunchConfiguration("max_takeoff_altitude"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("drone_api")),
        ),
        OpaqueFunction(function=_nodered),
    ])
