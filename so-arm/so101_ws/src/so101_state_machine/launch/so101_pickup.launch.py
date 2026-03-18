from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    perception_log_level = LaunchConfiguration('perception_log_level')
    bt_log_level = LaunchConfiguration('bt_log_level')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )

    declare_perception_log = DeclareLaunchArgument(
        'perception_log_level', default_value='info'
    )

    declare_bt_log = DeclareLaunchArgument(
        'bt_log_level', default_value='info'
    )

    perception_node = Node(
        package='so101_state_machine',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', perception_log_level],
    )

    bt_node = Node(
        package='so101_state_machine',
        executable='bt_node',
        name='bt_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', bt_log_level],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_perception_log,
        declare_bt_log,
        LogInfo(msg='Launching nodes'),
        perception_node,
        bt_node,
    ])