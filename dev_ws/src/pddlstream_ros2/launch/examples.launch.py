import os
from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
 
  return LaunchDescription([
    Node(package='pddlstream_ros2', executable='blocksworld', output='screen')
  ])

