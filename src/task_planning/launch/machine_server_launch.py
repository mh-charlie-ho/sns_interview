import os
import launch
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(get_package_share_directory('task_planning'),
                          'config', 'params.yaml')
    return launch.LaunchDescription([
        Node(package="conveyor",
             executable="conveyor_server",
             name="conveyor_server",
             output="screen",
             emulate_tty=True),
        Node(package="robot",
             executable="robot_server",
             name="robot_server",
             output="screen",
             emulate_tty=True),
        Node(package="stove",
             executable="stove_server",
             name="stove_server",
             output="screen",
             emulate_tty=True),
    ])
