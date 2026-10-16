# =============================================================================
#  vision.launch.py — 模擬的相機接進 ROS，並啟動偵測與定位
#
#  用法：
#      ros2 launch drone_nodered vision.launch.py
#      ros2 launch drone_nodered vision.launch.py debug_image:=false   # 省一點頻寬
#
#  前置：模擬要用有相機的機體和有目標的世界
#      WORLD=empty_target SIM_MODEL=x500_depth ~/…/scripts/start_empty_sitl.sh
#
#  這支做三件事：
#      1. 把 Gazebo 的影像搬到 ROS（gz 和 ROS 是兩套不同的通訊系統，要橋接）
#      2. 啟動偵測器（drone_vision，通用套件）—— detector:=color 或 yolo
#      3. 啟動目標定位（同上，輸出「相機前方幾公尺的一個點」）
#
#  ---------------------------------------------------------------------------
#  detector:=color / yolo
#  ---------------------------------------------------------------------------
#      color  找洋紅色塊。對 magenta_target 幾乎不會錯，所以也是出問題時的對照組。
#      yolo   VisDrone 權重，認得真車外觀的 hatchback_target。
#  兩個節點的 topic 介面完全一樣，所以下游（locator / world / api）不用改。
#
#  ⚠️ 兩件事要跟著一起換，不然會安靜地算錯：
#      ① target_height_m —— 射線要和「目標中心離地多高」的平面交會。
#         洋紅方塊中心 0.275 m、真車中心 0.1175 m（車高 0.235 的一半，
#         模型原點在車輪著地面）。差這 0.16 m 在俯角 67° 下會讓算出來的
#         地面點往遠處偏約 6.7 cm，**而且不會報錯**。
#      ② YOLO 的直譯器 —— ultralytics 裝在獨立 venv（系統 numpy 1.21 太舊，
#         而升級 numpy 會殺死 cv_bridge，連帶整個工作區的 Python 節點）。
#         所以 yolo 那個 Node 用 prefix 指定 venv 的 python，使用者不用先 source。
#         venv 路徑可以用 yolo_python 參數覆寫。
#
#  為什麼橋接放在 drone_nodered 而不是 drone_vision：
#      這裡的 topic 名稱是「模擬世界和機體名稱」決定的（世界叫 empty_target、
#      機體叫 x500_depth_0），換成實機那些名字完全不一樣。
#      drone_vision 要保持通用，不能綁死模擬的命名。
# =============================================================================

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
# LaunchConfiguration 送出去是字串，而 target_world_node 把 camera_pitch_deg
# 宣告成 double —— 直接傳會丟 InvalidParameterTypeException。要用 ParameterValue 轉型。
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")

    # Gazebo 給 RGB 的 topic 是「作用域全名」：世界/模型/連桿/感測器。
    # 深度相機的 topic 則是模型檔裡自己指定的短名稱 /depth_camera。
    rgb = PythonExpression(["'/world/' + '", world, "' + '/model/' + '", model,
                            "' + '/link/camera_link/sensor/IMX214/image'"])
    rgb_info = PythonExpression(["'/world/' + '", world, "' + '/model/' + '", model,
                                 "' + '/link/camera_link/sensor/IMX214/camera_info'"])

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="empty_target"),
        DeclareLaunchArgument("model", default_value="x500_depth_tilt_0"),
        # 相機的安裝俯角（度，往下為正）。必須和機體模型的 camera_link <pose> 一致 ——
        # 不一致的話偵測到的框會換算出錯誤的地面座標，而且**不會報錯**，
        # 只會看到目標的世界座標固定偏一個方向。
        #   x500_depth_tilt → 45.0（0.785 rad）
        #   x500_cam_down   → 67.0（1.1694 rad）
        DeclareLaunchArgument("camera_pitch", default_value="45.0",
                              description="相機俯角（度）。要和機體模型的 camera_link 一致"),
        DeclareLaunchArgument("namespace", default_value="MAV1"),
        DeclareLaunchArgument("debug_image", default_value="true",
                              description="要不要發布畫了框的影像（RViz 看得到）"),
        DeclareLaunchArgument("detector", default_value="color",
                              choices=["color", "yolo"],
                              description="用哪個偵測器：color 找洋紅、yolo 認真車"),
        DeclareLaunchArgument(
            "yolo_weights",
            default_value=os.path.expanduser(
                "~/yolo_data/weights/yolov8s-visdrone.pt"),
            description="YOLO 權重。s 夠用又快；要更準可換 yolov8l-visdrone.pt"),
        DeclareLaunchArgument(
            "yolo_python",
            default_value=os.path.expanduser("~/yolo-venv/bin/python3"),
            description="跑 YOLO 節點的直譯器（ultralytics 在獨立 venv 裡）"),
        DeclareLaunchArgument("yolo_conf", default_value="0.25",
                              description="YOLO 的信心門檻"),
        # 目標中心離地多高。必須跟著 detector 換 —— 見檔頭的警告。
        #   magenta_target  0.275（車體中心 0.075 + 方塊相對車體 0.2）
        #   hatchback_target 0.1175（車高 0.235 的一半，原點在車輪著地面）
        DeclareLaunchArgument(
            "target_height",
            default_value=PythonExpression(
                ["'0.1175' if '", LaunchConfiguration("detector"),
                 "' == 'yolo' else '0.275'"]),
            description="目標中心離地高度（m）。預設依 detector 自動選"),

        # --- 1) 影像橋接 ---------------------------------------------------------
        # 用 parameter_bridge 而不是 image_bridge：一次可以帶三個 topic（含 camera_info），
        # 而 image_bridge 只處理影像。
        # 語法：<gz topic>@<ROS 型別>[<gz 型別>   ——「[」代表只從 gz 流向 ROS
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="camera_bridge",
             arguments=[
                 [rgb, "@sensor_msgs/msg/Image[gz.msgs.Image"],
                 [rgb_info, "@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
                 "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
                 "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
             ],
             # 改名成短一點、和實機無關的名字，後面的節點就不用跟著模擬的命名走
             remappings=[(rgb, "/camera/image"), (rgb_info, "/camera/camera_info"),
                         ("/depth_camera", "/depth/image"),
                         ("/camera_info", "/depth/camera_info")],
             output="screen"),

        # --- 2a) 顏色偵測（detector:=color）----------------------------------------
        Node(package="drone_vision", executable="color_detector_node.py",
             name="color_detector", output="screen", emulate_tty=True,
             condition=IfCondition(PythonExpression(
                 ["'", LaunchConfiguration("detector"), "' == 'color'"])),
             parameters=[{"image_topic": "/camera/image",
                          "publish_debug_image": LaunchConfiguration("debug_image")}]),

        # --- 2b) YOLO 偵測（detector:=yolo）----------------------------------------
        # ⚠️ prefix 指定 venv 的直譯器。ROS 的節點預設用系統 python 跑，
        #    而 ultralytics 只裝在 venv 裡（見檔頭）。用 prefix 的好處是
        #    使用者不用先 source，systemd 服務也能直接用。
        Node(package="drone_vision", executable="yolo_detector_node.py",
             name="yolo_detector", output="screen", emulate_tty=True,
             prefix=LaunchConfiguration("yolo_python"),
             condition=IfCondition(PythonExpression(
                 ["'", LaunchConfiguration("detector"), "' == 'yolo'"])),
             parameters=[{"image_topic": "/camera/image",
                          "weights": LaunchConfiguration("yolo_weights"),
                          "confidence": ParameterValue(
                              LaunchConfiguration("yolo_conf"), value_type=float),
                          "publish_debug_image": LaunchConfiguration("debug_image")}]),

        # --- 4) 目標的世界座標（射線 + 高度 → NED 位置）------------------------------
        # 這個節點知道無人機（要讀 PX4 的位置姿態），所以放在 drone_nodered，
        # 不能放進通用的 drone_vision。
        Node(package="drone_nodered", executable="target_world_node.py",
             name="target_world", output="screen", emulate_tty=True,
             # target_height_m：射線要和「離地多高的水平面」交會。
             # 洋紅方塊的中心在 0.275 m（車體中心 0.075 + 方塊相對車體 0.2，
             # magenta_target/model.sdf:40）。設低 0.075 m 的話射線要多降一點才碰到
             # 平面，算出來的位置會往遠處偏 0.075/tan(56°) ≈ 5 cm。
             parameters=[{"px4_namespace": LaunchConfiguration("namespace"),
                          "camera_pitch_deg": ParameterValue(
                              LaunchConfiguration("camera_pitch"), value_type=float),
                          "target_height_m": ParameterValue(
                              LaunchConfiguration("target_height"), value_type=float)}]),

        # --- 3) 目標定位（框 + 深度 → 相機座標的 3D 點）-----------------------------
        Node(package="drone_vision", executable="target_locator_node.py",
             name="target_locator", output="screen", emulate_tty=True,
             parameters=[{"rgb_info_topic": "/camera/camera_info",
                          "depth_topic": "/depth/image",
                          "depth_info_topic": "/depth/camera_info"}]),
    ])
