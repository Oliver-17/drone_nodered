"""假的內層：模擬 sim_all 把 model 改寫成「<model>_0」。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="x500"),
        SetLaunchConfiguration(
            "model", PythonExpression(["'", LaunchConfiguration("model"), "' + '_0'"])),
        LogInfo(msg=["    inner: model 被改寫成 ", LaunchConfiguration("model")]),
    ])
