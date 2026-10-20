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

        # api/track 的搜尋環線用 map 座標，節點要靠這個換算成 PX4 本地 NED。

        # 空字串 = 沒有這個資訊 → 帶搜尋環線的請求會被明確拒絕。

        DeclareLaunchArgument("local_origin_map", default_value="",

                              description='PX4 本地原點在 map 的座標，"x,y"。'

                                          "要和 px4_bridge 的 odom_origin 一致"),

        DeclareLaunchArgument("camera_pitch_deg", default_value="67.0",

                              description="相機俯角（度）。api/track 的 "

                                          "distance=0 用它算最佳跟隨距離"),

        # api/track 的**跟隨段**是 cmd_vel 直通 offboard、不經過 Nav2 ——
        # 那一段沒有 costmap、沒有膨脹層、沒有任何避障。
        # 2026-10-19 摔機一次（站位點算錯方向，滿速往西開 1.9 秒撞上貨架）。
        # 空字串 = 沒有保險（節點啟動時會印黃色警告）。
        # 底座是獨立服務，warehouse_all 內部那條線接不到這裡 ——
        # 值由 scripts/install_services.sh 從 warehouse_geometry.yaml 填進
        # drone-nodered.service 的 ExecStart。

        DeclareLaunchArgument("no_fly_boxes", default_value="",

                              description="跟隨段的禁飛方框，map 座標攤平成一串，"

                                          "每 4 個一組 x_min,y_min,x_max,y_max"),

        # 目標車的出生位置（map 座標 "x,y"）。只影響儀表板：還沒偵測到車時
        # 地圖畫這個位置當「車應該在哪」。空字串 = 偵測到才會有車。
        # 值由 scripts/install_services.sh 從 warehouse_geometry.yaml 的
        # target_spawn 填進 drone-nodered.service 的 ExecStart。

        DeclareLaunchArgument("target_home_map", default_value="",

                              description='目標車的出生位置，map 座標 "x,y"。'

                                          "只給儀表板畫「還沒看到車時車在哪」"),

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
                # 場景參數（2026-10-08、2026-10-19 加第三個）。底座服務是
                # 「不知道場景」的，所以這些由啟動的人注入 ——
                # drone-nodered.service 的 ExecStart 帶進來，
                # 值由 install_services.sh 從 warehouse_geometry.yaml 填。
                # ⚠️ local_origin_map 必須和 px4_bridge 的 odom_origin 相同，
                #    不一致時 api/track 的搜尋航點會整組平移且**不會報錯**。
                # ⚠️ no_fly_boxes 漏掉的話跟隨段完全沒有避障（見上面的說明）。
                #    節點啟動會印黃色警告，不要忽略它。
                "local_origin_map": LaunchConfiguration("local_origin_map"),
                "camera_pitch_deg": LaunchConfiguration("camera_pitch_deg"),
                "no_fly_boxes": LaunchConfiguration("no_fly_boxes"),
                "target_home_map": LaunchConfiguration("target_home_map"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("drone_api")),
        ),
        OpaqueFunction(function=_nodered),
    ])
