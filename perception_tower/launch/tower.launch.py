"""Launch the perception_tower node.

Arguments:
  params_file    : path to a parameter YAML (defaults to this package's
                   config/tower_params.yaml).
  mock_hardware  : "true"/"false" -> run with FakeServo/FakeFairy/FakeCamera
                   instead of real serial + sensor subscriptions.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    mock_hardware = LaunchConfiguration("mock_hardware")

    default_params = PathJoinSubstitution(
        [FindPackageShare("perception_tower"), "config", "tower_params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the tower parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "mock_hardware",
                default_value="false",
                description="Run with fake hardware (no serial/sensors).",
            ),
            Node(
                package="perception_tower",
                executable="tower_node",
                name="tower_node",
                output="screen",
                parameters=[params_file, {"mock_hardware": mock_hardware}],
            ),
        ]
    )
