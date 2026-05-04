import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from launch.actions import (
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart

def generate_launch_description():

    package_name = "webot_description"

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            get_package_share_directory(package_name),
            "worlds",
            "shapes.sdf",
        ),
        description="World file"
    )
    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                "rsp.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": ["-r ", LaunchConfiguration("world")]
        }.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "webot"],
        output="screen",
    )

    bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["--ros-args", "-p", "config_file:=" + os.path.join(
        get_package_share_directory(package_name),
        "config",
        "bridge.yaml"
    )],
    output="screen"
)
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
 
    ackermann_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_steering_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )
 
    # Start joint_state_broadcaster after spawn completes
    start_joint_state_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
 
    # Start ackermann_steering_controller after joint_state_broadcaster loads
    start_ackermann_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ackermann_steering_controller_spawner],
        )
    )

    return LaunchDescription([
        world_arg,
        rsp,
        gazebo,
        spawn_entity,
        bridge,
        start_joint_state_broadcaster_after_spawn,
        start_ackermann_controller_after_jsb,
    ])