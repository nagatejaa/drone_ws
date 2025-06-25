from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    world_path = os.path.join(
        '/home/nagateja/ws_drone/src/drone_control/drone_launch',
        'my_iris_runway.sdf'
    )

    ardupilot_path = '/home/nagateja/ardupilot/ArduCopter'

    return LaunchDescription([



        # Terminal 2: Gazebo with drone SDF world
        ExecuteProcess(
            cmd=['gz', 'sim', '-v4', '-r', world_path],
            output='screen'
        ),

        # Terminal 3: MAVROS bridge
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('mavros'),
                    'launch',
                    'apm.launch'
                ])
            ),
            launch_arguments={
                'fcu_url': 'udp://:14540@127.0.0.1:14557'
            }.items()
        ),
    ])
