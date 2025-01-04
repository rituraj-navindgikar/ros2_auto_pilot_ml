import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


'''
FILE NAME                  SPAWN COORDINATES
m-shape-circle-room.world ;      -6 -7
circle-room.world         ;      -5 -5
square.world              ;      -6 -7
triangle.world            ;      -5 -5
eight.world               ;      
maze.world                ;        
u-shape.world             ;
s-shape.world             ; 

hair-pin-bend.world
'''


WORLD_FILE = 'maze.world'

spawn_x = '-6.0'
spawn_y = '-7.0'
spawn_z = '0.0'

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
            get_package_share_directory('robot-machine-learning'), 'worlds', WORLD_FILE
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
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,                   
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


