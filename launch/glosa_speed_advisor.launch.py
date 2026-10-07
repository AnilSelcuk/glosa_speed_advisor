"""
Launch the GLOSA speed advisor.

Topics and the map file are set here (launch arguments), everything else in
config/glosa.param.yaml.

Examples
    ros2 launch glosa_speed_advisor glosa_speed_advisor.launch.py
    ros2 launch glosa_speed_advisor glosa_speed_advisor.launch.py map_file:=/path/to/lanelet2_map.osm
    ros2 launch glosa_speed_advisor glosa_speed_advisor.launch.py spat_topic:=/fake/spat
    ros2 launch glosa_speed_advisor glosa_speed_advisor.launch.py param_file:=/path/to/my.param.yaml
    ros2 launch glosa_speed_advisor glosa_speed_advisor.launch.py --show-args
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
            default_value=os.path.join(share, "config", "glosa.param.yaml"),
            description="YAML file with all GLOSA settings (rules, thresholds, approaches)"),

        DeclareLaunchArgument(
            "map_file",
            default_value="",
            description="Path to lanelet2_map.osm. Needed when an approach in the YAML file "
                        "uses traffic_light_group_id (stop line taken from the map)"),

        # ---------------------------------------------------------- inputs
        DeclareLaunchArgument(
            "spat_topic",
            default_value="/v2i/spat/raw",
            description="Input: SPaT from the V2I bridge (v2i_spat_msgs/SpatPacket)"),
        DeclareLaunchArgument(
            "odometry_topic",
            default_value="/localization/kinematic_state",
            description="Input: ego pose and speed (nav_msgs/Odometry)"),

        # --------------------------------------------------------- outputs
        DeclareLaunchArgument(
            "velocity_limit_topic",
            default_value="/planning/scenario_planning/max_velocity_candidates",
            description="Output: velocity limit for Autoware "
                        "(autoware_internal_planning_msgs/VelocityLimit)"),
        DeclareLaunchArgument(
            "clear_velocity_limit_topic",
            default_value="/planning/scenario_planning/clear_velocity_limit",
            description="Output: removes our velocity limit "
                        "(autoware_internal_planning_msgs/VelocityLimitClearCommand)"),
        DeclareLaunchArgument(
            "status_topic",
            default_value="/glosa_speed_advisor/status",
            description="Output: human-readable decision text (std_msgs/String)"),
    ]

    node = Node(
        package="glosa_speed_advisor",
        executable="glosa_node",
        name="glosa_speed_advisor",
        output="screen",
        parameters=[
            LaunchConfiguration("param_file"),
            # Topic names from the launch arguments (they win over the YAML file).
            {
                "topics.spat": text("spat_topic"),
                "topics.odometry": text("odometry_topic"),
                "topics.velocity_limit": text("velocity_limit_topic"),
                "topics.clear_velocity_limit": text("clear_velocity_limit_topic"),
                "topics.status": text("status_topic"),
                "map.osm_file": text("map_file"),
            },
        ],
    )

    return LaunchDescription(args + [node])
