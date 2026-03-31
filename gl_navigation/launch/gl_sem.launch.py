import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_gl_base = get_package_share_directory('gl_base')
    pkg_gl_navigation = get_package_share_directory('gl_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    nav_params_file = os.path.join(pkg_gl_navigation, 'config', 'nav_burger.yaml')
    map_file = os.path.join(pkg_gl_navigation, 'config', 'small_room_map.yaml')


    # * Spawn the gazebo world and the robot
    gl_room = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gl_base, 'launch', 'gl_room.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # * Open the rviz configuration file
    rviz_config_dir = os.path.join(pkg_gl_navigation, 'rviz', 'gl_nav_main.rviz')
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir]
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

    delayed_nav = RegisterEventHandler(
        OnProcessStart(
            target_action=rviz_cmd,
            on_start=[gl_nav]
        )
    )


    # ? Occ Grid Publisher to publish over mapserver
    # occ_grid_pub = Node(
    #     package='gl_navigation',
    #     executable='occupancy_grid_publisher.py',
    #     name='occupancy_grid_publisher',
    #     output='screen'
    # )

    # * Run RRT Planner Node
    gl_rrt_planner = Node(
        package='gl_navigation',
        executable='rrt_planner',
        name='rrt_planner'
    )

    ld = LaunchDescription()

    ld.add_action(gl_room)
    ld.add_action(rviz_cmd)
    ld.add_action(delayed_nav)
    ld.add_action(gl_rrt_planner)
    # ld.add_action(occ_grid_pub)

    return ld