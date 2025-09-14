import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Package paths
    world_file = '/mnt/macshare/Lane-Following-Robot/src/lane_following_robot/worlds/lane_world.world'
    xacro_file = '/mnt/macshare/Lane-Following-Robot/src/lane_following_robot/urdf/lane_robot.urdf.xacro'
    
    # Process robot description
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    return LaunchDescription([
        # Start Gazebo server
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', 
                 '-s', 'libgazebo_ros_factory.so', world_file],
            output='screen'
        ),
        
        # Start Gazebo client
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        
        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-entity', 'lane_robot',
                      '-topic', '/robot_description',
                      '-x', '0', '-y', '0', '-z', '0.1']
        ),
    ])