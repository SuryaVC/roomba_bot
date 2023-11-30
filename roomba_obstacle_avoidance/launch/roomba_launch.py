import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    record_topics = DeclareLaunchArgument('record_topics', default_value='False', choices=['True', 'False'])  
    roomba_node = Node(
            package='roomba_obstacle_avoidance',
            executable='roomba_avoidance',
            name='roomba_avoidance'
        )
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch'),
                                      '/turtlebot3_world.launch.py'])
    )
    recorder_node = ExecuteProcess(
        condition = IfCondition(LaunchConfiguration('record_topics')),
        cmd = ['ros2', 'bag', 'record', '-o', 'roomba_bag_topics_list', '-a' , '-x "/camera.+"'],
        shell=True
    )
    return LaunchDescription([
        gazebo_world,
        record_topics,
        roomba_node,
        recorder_node
    ])