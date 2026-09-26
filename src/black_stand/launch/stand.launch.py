from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="black_stand",
            executable="stand_mujoco_sim",
            on_exit=Shutdown(),
        ),
        Node(
            package="black_stand",
            executable="stand_controller",
        ),
    ])
