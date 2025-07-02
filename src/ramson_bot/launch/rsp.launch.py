from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('ramson_bot')

    use_sim_time = LaunchConfiguration('use_sim_time')
    xacro_file = LaunchConfiguration('xacro_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_xacro_file_cmd = DeclareLaunchArgument(
        'xacro_file',
        default_value=os.path.join(pkg_dir, 'urdf', 'diff_drive.urdf.xacro'),
        description='Absolute path to the robot xacro file'
    )

    robot_description = Command([
        'xacro ', xacro_file
    ])

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        remappings=remappings,
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'robot_description': robot_description}
        ],
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_xacro_file_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])
