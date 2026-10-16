"""壞的寫法：把依 model 的判斷寫在 include 的 launch_arguments 裡。"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="x500_cam_down"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(HERE, "inner.launch.py")),
            launch_arguments={"model": LaunchConfiguration("model")}.items()),
        TimerAction(period=1.0, actions=[
            LogInfo(msg=["    BAD  俯角 = ", PythonExpression(
                ["'67.0' if '", LaunchConfiguration("model"),
                 "' == 'x500_cam_down' else '45.0'"])]),
        ]),
    ])
