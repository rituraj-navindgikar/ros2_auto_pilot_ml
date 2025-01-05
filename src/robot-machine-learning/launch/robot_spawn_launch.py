import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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

def generate_launch_description():
    pkg_name = 'robot-machine-learning'
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Declare the world argument
    world_arg = DeclareLaunchArgument(
        'world', default_value='square.world', description='World file name to load in Gazebo'
    )
    world_file_path = PathJoinSubstitution([
        FindPackageShare('robot-machine-learning'),
        'worlds',
        LaunchConfiguration('world')
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={'world': world_file_path}.items()
    )

    # Declare the world argument - refer above for spawn points
    spawn_x = DeclareLaunchArgument('spawn_x', default_value='-6', description='Spawn x coordinate in Gazebo')
    spawn_y = DeclareLaunchArgument('spawn_y', default_value='-7', description='Spawn y coordinate in Gazebo')
    spawn_z = DeclareLaunchArgument('spawn_z', default_value='0', description='Spawn z coordinate in Gazebo')
    roll = DeclareLaunchArgument('roll', default_value='0', description='Roll')
    pitch = DeclareLaunchArgument('pitch', default_value='0', description='Pitch')
    yaw = DeclareLaunchArgument('yaw', default_value='0', description='Yaw')

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',  
            '-entity', 'my_bot',            
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),                   
            '-R', LaunchConfiguration('roll'),
            '-P', LaunchConfiguration('pitch'),
            '-Y', LaunchConfiguration('yaw')
        ],
        output='screen'
    )

    ld = LaunchDescription()
    # arguments
    ld.add_action(world_arg)
    ld.add_action(spawn_x)
    ld.add_action(spawn_y)
    ld.add_action(spawn_z)
    ld.add_action(roll)
    ld.add_action(pitch)
    ld.add_action(yaw)

    # launch
    ld.add_action(rsp)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    


    return ld 

    # LaunchDescription([
    #     rsp,
    #     gazebo,
    #     spawn_entity,
    # ])


