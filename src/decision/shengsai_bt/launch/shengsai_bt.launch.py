from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("shengsai_bt").find("shengsai_bt")
    default_params = package_share + "/config/shengsai.yaml"

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Shengsai mission parameter file",
        ),
        Node(
            package="shengsai_bt",
            executable="shengsai_bt_node",
            name="shengsai_bt",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
