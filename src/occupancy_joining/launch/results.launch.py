from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    args=['--result',LaunchConfiguration('result'),'--duration',LaunchConfiguration('duration'),'--domain-id',LaunchConfiguration('domain_id')]
    return LaunchDescription([
        DeclareLaunchArgument('result'),
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('duration',default_value='0'),
        DeclareLaunchArgument('domain_id',default_value=EnvironmentVariable('ROS_DOMAIN_ID',default_value='88')),
        Node(package='occupancy_joining',executable='joining_viewer.py',arguments=args,condition=IfCondition(LaunchConfiguration('gui')),output='screen'),
        Node(package='occupancy_joining',executable='joining_viewer.py',arguments=args+['--no-rviz'],condition=UnlessCondition(LaunchConfiguration('gui')),output='screen'),
    ])
