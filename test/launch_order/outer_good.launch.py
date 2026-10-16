"""好的寫法：提到外層 DeclareLaunchArgument，在改寫之前求值。"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="x500_cam_down"),
        DeclareLaunchArgument("pitch", default_value=PythonExpression(
            ["'67.0' if '", LaunchConfiguration("model"),
             "' == 'x500_cam_down' else '45.0'"])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(HERE, "inner.launch.py")),
            launch_arguments={"model": LaunchConfiguration("model")}.items()),
        TimerAction(period=1.0, actions=[
            LogInfo(msg=["    GOOD 俯角 = ", LaunchConfiguration("pitch")]),
        ]),
    ])
