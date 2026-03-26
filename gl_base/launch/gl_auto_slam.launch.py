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
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    autostart = LaunchConfiguration('autostart', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager', default='false')

    slam_params_file = os.path.join(pkg_gl_base, 'config', 'mapper_params_online_async.yaml')
    nav_params_file = os.path.join(pkg_gl_base, 'config', 'nav_burger.yaml')


    # * Spawn the gazebo world and the robot
    gl_room = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gl_base, 'launch', 'gl_room.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # * Spawn the slam_toolbox nodes
    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
            slam_params_file,
            {
            'use_lifecycle_manager': use_lifecycle_manager,
            'use_sim_time': use_sim_time
            }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    # * Spawn the nav2 nodes
    gl_nav = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': "" ,  # empty string here makes it subscrbe to the online map topic
                'use_sim_time': use_sim_time,
                'params_file': nav_params_file}.items(),
    )


    # * Open the rviz configuration file
    rviz_config_dir = os.path.join(pkg_gl_base, 'rviz', 'gl_slam.rviz')
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir]
    )

    # * Run Frontier Guidance Node

    gl_frontier_det = Node(
        package='gl_base',
        executable='frontier_guidance',
        name='frontier_node'
    )

    ld = LaunchDescription()

    ld.add_action(gl_room)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)
    ld.add_action(gl_nav)
    ld.add_action(rviz_cmd)
    ld.add_action(gl_frontier_det)

    return ld