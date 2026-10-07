"""
Launch the fake fixed-time SPaT publisher (planning simulator only).

    ros2 launch glosa_speed_advisor fake_spat.launch.py
    ros2 launch glosa_speed_advisor fake_spat.launch.py param_file:=/path/to/my_fake.param.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def text(argument):
    """Launch argument as a string parameter (also when it is empty)."""
    return ParameterValue(LaunchConfiguration(argument), value_type=str)


def generate_launch_description():
    share = get_package_share_directory("glosa_speed_advisor")

    args = [
        DeclareLaunchArgument(
            "param_file",
            default_value=os.path.join(share, "config", "fake_spat.param.yaml"),
            description="YAML file with the fake signal plan"),
        DeclareLaunchArgument(
            "spat_topic",
            default_value="/v2i/spat/raw",
            description="Output: fake SPaT (v2i_spat_msgs/SpatPacket)"),
        DeclareLaunchArgument(
            "traffic_signals_topic",
            default_value="/perception/traffic_light_recognition/traffic_signals",
            description="Output: Autoware traffic signals, used only when a signal has "
                        "traffic_light_group_id > 0"),
    ]

    node = Node(
        package="glosa_speed_advisor",
        executable="fake_spat_node",
        name="fake_spat",
        output="screen",
        parameters=[
            LaunchConfiguration("param_file"),
            {
                "topics.spat": text("spat_topic"),
                "topics.traffic_signals": text("traffic_signals_topic"),
            },
        ],
    )

    return LaunchDescription(args + [node])
