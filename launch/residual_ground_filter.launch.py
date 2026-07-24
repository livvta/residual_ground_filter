from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("residual_ground_filter"))
    default_params = str(package_share / "config" / "residual_ground_filter.param.yaml")

    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic",
                default_value="/patchworkpp/nonground",
                description="过滤前的非地面点云",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/patchworkpp/nonground_filtered",
                description="过滤后的点云",
            ),
            DeclareLaunchArgument(
                "params_file", default_value=default_params, description="参数文件"
            ),
            Node(
                package="residual_ground_filter",
                executable="residual_ground_filter_node",
                name="residual_ground_filter",
                output="screen",
                parameters=[params_file],
                remappings=[("input", input_topic), ("output", output_topic)],
            ),
        ]
    )
