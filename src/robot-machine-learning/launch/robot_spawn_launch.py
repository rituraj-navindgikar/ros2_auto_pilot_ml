import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    pkg_name = 'robot-machine-learning'
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('robot-machine-learning'), 'worlds', 'circle-room.world'
        ),
        description='World file to load in Gazebo'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]), launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',  
            '-entity', 'my_bot',            
            '-x', '-7',
            '-y', '-3',
            '-z', '0.0',                   
            # '-R', '0.0',                   
            # '-P', '0.0',                   
            # '-Y', '0'                   
        ],
        output='screen'
    )


    ld = LaunchDescription()
    ld.add_action(declare_world)
    ld.add_action(rsp)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    


    return ld 

    # LaunchDescription([
    #     rsp,
    #     gazebo,
    #     spawn_entity,
    # ])


