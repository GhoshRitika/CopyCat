from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(name='use_jsp', default_value='gui',
                              choices=['true', 'false', 'gui'],
                              description='Enable joint_state_publisher_node'),
        DeclareLaunchArgument(name='model',
                              default_value=str(get_package_share_path('allegro_driver')
                                                / 'urdf/AllegroHandRight.xacro'),
                              description='Path to robot urdf file'),
        DeclareLaunchArgument(name='use_rviz', default_value='true',
                              choices=['true', 'false'],
                              description='Enable visualization in rviz'),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=LaunchConfigurationEquals('use_jsp', 'false')),
            
        Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=LaunchConfigurationEquals('use_jsp', 'gui')),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description':
                        Command([ExecutableInPackage("xacro", "xacro"), " ",
                                 PathJoinSubstitution(
                                 [FindPackageShare("allegro_driver"),
                                  "urdf/AllegroHandRight.urdf.xacro"]),
                                 ]
                                )}
                        ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=LaunchConfigurationEquals('use_rviz', 'true'),
            on_exit=Shutdown())
    ])