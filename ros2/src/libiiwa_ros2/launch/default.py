from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # launch arguments
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
    # libiiwa configuration (see LibIiwa class)
    libiiwa_ip_launch_arg = DeclareLaunchArgument(
        "libiiwa_ip", default_value=TextSubstitution(text="0.0.0.0")
    )
    libiiwa_port_launch_arg = DeclareLaunchArgument(
        "libiiwa_port", default_value=TextSubstitution(text="12225")
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
            "libiiwa_ip": LaunchConfiguration('libiiwa_ip'),
            "libiiwa_port": LaunchConfiguration('libiiwa_port')
        }],
        output='screen'
    )

    return LaunchDescription([
        robot_name_launch_arg,
        controller_name_launch_arg,
        action_namespace_launch_arg,
        libiiwa_ip_launch_arg,
        libiiwa_port_launch_arg,
        node
    ])
