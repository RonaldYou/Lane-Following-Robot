import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, TimerAction
from launch_ros.actions import Node
import xacro
import tempfile

def spawn_robot(context, *args, **kwargs):
    xacro_file = '/mnt/macshare/Lane-Following-Robot/src/lane_following_robot/urdf/lane_robot.urdf.xacro'
    
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
        f.write(robot_urdf)
        temp_urdf_file = f.name
    
    return [
        ExecuteProcess(
            cmd=[
                'ign', 'service', '-s', '/world/lane_world/create',
                '--reqtype', 'ignition.msgs.EntityFactory',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '5000',
                '--req', f'sdf_filename: "{temp_urdf_file}" name: "lane_robot" pose: {{position: {{x: -9.0, y: 0.0, z: 0.05}} orientation: {{x: 0, y: 0, z: 0, w: 1}}}}'
            ],
            output='screen'
        )
    ]

def generate_launch_description():
    world_file = '/mnt/macshare/Lane-Following-Robot/src/lane_following_robot/worlds/lane_world.world'
    xacro_file = '/mnt/macshare/Lane-Following-Robot/src/lane_following_robot/urdf/lane_robot.urdf.xacro'
    
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v', '4', world_file],
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
        
        # Spawn robot
        TimerAction(
            period=3.0,
            actions=[OpaqueFunction(function=spawn_robot)]
        ),
        
        # Camera debug node
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='lane_following_robot',
                    executable='camera_debug',
                    name='camera_debug',
                    output='screen',
                )
            ]
        ),
        
        # RViz (no config file needed)
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                )
            ]
        ),
    ])