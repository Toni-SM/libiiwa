from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_name_launch_arg = DeclareLaunchArgument(
        "robot_name", default_value=TextSubstitution(text="iiwa")
    )

    # FollowJointTrajectory action: /controller_name/action_namespace
    controller_name_launch_arg = DeclareLaunchArgument(
        "controller_name", default_value=TextSubstitution(text="iiwa_controller")
    )
    action_namespace_launch_arg = DeclareLaunchArgument(
        "action_namespace", default_value=TextSubstitution(text="follow_joint_trajectory")
    )
    follow_all_trajectory_launch_arg = DeclareLaunchArgument(
        "follow_all_trajectory", default_value=TextSubstitution(text="True")
    )
    trajectory_update_threshold_launch_arg = DeclareLaunchArgument(
        "trajectory_update_threshold", default_value=TextSubstitution(text="0.5")
    )

    # libiiwa configuration (see LibIiwa class)
    libiiwa_ip_launch_arg = DeclareLaunchArgument(
        "libiiwa_ip", default_value=TextSubstitution(text="0.0.0.0")
    )
    libiiwa_port_launch_arg = DeclareLaunchArgument(
        "libiiwa_port", default_value=TextSubstitution(text="12225")
    )

    servo_interface_launch_arg = DeclareLaunchArgument(
        "servo_interface", default_value=TextSubstitution(text="True")
    )
    
    # node configuration
    verbose_launch_arg = DeclareLaunchArgument(
        "verbose", default_value=TextSubstitution(text="False")
    )

    # ROS2 node
    node = Node(
        package='libiiwa_ros2',
        namespace='libiiwa_ros2',
        executable='node',
        name='iiwa',
        parameters=[{
            "robot_name": LaunchConfiguration('robot_name'),
            "controller_name": LaunchConfiguration('controller_name'),
            "action_namespace": LaunchConfiguration('action_namespace'),
            "follow_all_trajectory": LaunchConfiguration('follow_all_trajectory'),
            "trajectory_update_threshold": LaunchConfiguration('trajectory_update_threshold'),
            "libiiwa_ip": LaunchConfiguration('libiiwa_ip'),
            "libiiwa_port": LaunchConfiguration('libiiwa_port'),
            "servo_interface": LaunchConfiguration('servo_interface'),
            "verbose": LaunchConfiguration('verbose')
        }],
        output='screen',
    )

    return LaunchDescription([
        robot_name_launch_arg,
        controller_name_launch_arg,
        action_namespace_launch_arg,
        follow_all_trajectory_launch_arg,
        trajectory_update_threshold_launch_arg,
        libiiwa_ip_launch_arg,
        libiiwa_port_launch_arg,
        servo_interface_launch_arg,
        verbose_launch_arg,
        node
    ])
