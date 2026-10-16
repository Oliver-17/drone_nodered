# =============================================================================
#  drone_api.launch.py — 啟動 drone_api_node（起飛／盤旋／降落服務）
#
#  用法：
#      ros2 launch drone_nodered drone_api.launch.py
#      ros2 launch drone_nodered drone_api.launch.py namespace:=MAV2 target_system:=2
#      ros2 launch drone_nodered drone_api.launch.py max_takeoff_altitude:=1.5   # 室內實機
#
#  前置條件：PX4 在跑、MicroXRCEAgent 已連上（看得到 /<namespace>/fmu/out/*）
# =============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ns = LaunchConfiguration("namespace")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="MAV1",
                              description="PX4 的 namespace，服務也會放在這底下"),
        DeclareLaunchArgument("target_system", default_value="1",
                              description="PX4 的 MAV_SYS_ID，多機時是 instance+1"),
        DeclareLaunchArgument("min_takeoff_altitude", default_value="0.5",
                              description="起飛高度下限（公尺）"),
        DeclareLaunchArgument("max_takeoff_altitude", default_value="5.0",
                              description="起飛高度上限（公尺）。室內實機建議 1.5"),
        # 看門狗：狀況不對自動降落。實機要依電池狀況調 battery_min
        DeclareLaunchArgument("watchdog_enabled", default_value="true"),
        DeclareLaunchArgument("battery_min", default_value="0.30",
                              description="剩餘電量低於這個比例就自動降落（0~1）"),
        DeclareLaunchArgument("altitude_margin_m", default_value="1.0",
                              description="高度超過上限這麼多就算失控爬升"),
        # api/track 的搜尋環線用 map 座標，節點要知道「PX4 本地原點在 map 的哪裡」
        # 才換算得出來。⚠️ 必須和 px4_bridge 的 odom_origin 完全相同。
        # 空字串 = 沒有這個資訊 → 只能用原地轉頭掃描（會明確拒絕帶環線的請求）。
        DeclareLaunchArgument("local_origin_map", default_value="",
                              description='PX4 本地原點在 map 的座標，"x,y"。'
                                          "要和 px4_bridge 的 odom_origin 一致"),
        # 相機俯角（度）。只用來算 api/track 的 distance=0「最佳跟隨距離」
        # （= 高度 / tan(俯角)，光軸打到地面的位置）。
        # ⚠️ 要和 vision.launch.py 的 camera_pitch 一致：
        #    x500_cam_down → 67、x500_depth_tilt → 45。
        DeclareLaunchArgument("camera_pitch_deg", default_value="67.0",
                              description="相機安裝俯角（度）。算最佳跟隨距離用"),
        DeclareLaunchArgument("watchdog_debounce_s", default_value="1.0",
                              description="異常要連續成立這麼久才動作，避免瞬間雜訊誤觸發"),
        Node(
            package="drone_nodered", executable="drone_api_node.py",
            name="drone_api_node", namespace=ns,
            parameters=[{
                "px4_namespace": ns,
                "target_system": ParameterValue(LaunchConfiguration("target_system"),
                                                value_type=int),
                "min_takeoff_altitude": ParameterValue(
                    LaunchConfiguration("min_takeoff_altitude"), value_type=float),
                "max_takeoff_altitude": ParameterValue(
                    LaunchConfiguration("max_takeoff_altitude"), value_type=float),
                "watchdog_enabled": ParameterValue(
                    LaunchConfiguration("watchdog_enabled"), value_type=bool),
                "battery_min": ParameterValue(
                    LaunchConfiguration("battery_min"), value_type=float),
                "altitude_margin_m": ParameterValue(
                    LaunchConfiguration("altitude_margin_m"), value_type=float),
                "watchdog_debounce_s": ParameterValue(
                    LaunchConfiguration("watchdog_debounce_s"), value_type=float),
                # 字串參數不用 ParameterValue 包 —— value_type 推斷本來就會是 str
                "local_origin_map": LaunchConfiguration("local_origin_map"),
                "camera_pitch_deg": ParameterValue(
                    LaunchConfiguration("camera_pitch_deg"), value_type=float),
            }],
            output="screen", emulate_tty=True,
        ),
    ])
