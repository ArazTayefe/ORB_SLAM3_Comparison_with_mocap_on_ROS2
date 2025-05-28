from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_path = PathJoinSubstitution([
        FindPackageShare("slam_tools"),
        "rviz",
        "drone_slam_vs_mocap.rviz"
    ])

    return LaunchDescription([
        Node(
            package="slam_tools",
            executable="slam_pose_publisher",
            name="slam_pose_publisher",
            output="screen"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", config_path],
            output="screen"
        )
    ])
