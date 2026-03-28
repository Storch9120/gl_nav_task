import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent, RegisterEventHandler, LogInfo
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution)
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_gl_base = get_package_share_directory('gl_base')
    pkg_gl_navigation = get_package_share_directory('gl_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    nav_params_file = os.path.join(pkg_gl_base, 'config', 'nav_burger.yaml')
    map_file = os.path.join(pkg_gl_navigation, 'config', 'small_room_map.yaml')


    # * Spawn the gazebo world and the robot
    gl_room = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gl_base, 'launch', 'gl_room.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # * Spawn the nav2 nodes
    gl_nav = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,  # empty string here makes it subscrbe to the online map topic
                'use_sim_time': use_sim_time,
                'params_file': nav_params_file}.items(),
    )

    # * Occ Grid Publisher to publish over mapserver
    occ_grid_pub = Node(
        package='gl_navigation',
        executable='occupancy_grid_publisher',
        name='occ_grid_pub',
        output='screen'
    )

    # * Open the rviz configuration file
    rviz_config_dir = os.path.join(pkg_gl_navigation, 'rviz', 'gl_rrt.rviz')
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir]
    )

    # * Run RRT Planner Node
    gl_rrt_planner = Node(
        package='gl_navigation',
        executable='rrt_planner',
        name='rrt_planner'
    )

    ld = LaunchDescription()

    ld.add_action(gl_room)
    ld.add_action(gl_nav)
    ld.add_action(rviz_cmd)
    ld.add_action(gl_rrt_planner)

    return ld