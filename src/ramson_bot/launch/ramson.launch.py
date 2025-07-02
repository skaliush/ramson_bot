import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    ramson_dir = get_package_share_directory('ramson_bot')
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    graph_filepath = LaunchConfiguration('graph')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_lidar = LaunchConfiguration('use_lidar')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether run a SLAM'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(ramson_dir, 'maps', 'my_new_mapppp.yaml'),
    )

    declare_graph_file_cmd = DeclareLaunchArgument(
        'graph',
        default_value=os.path.join(ramson_dir, 'graphs', 'my_graph.geojson'),
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(ramson_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(ramson_dir, 'rviz', 'nav2_view.rviz'),
        description='Full path to the RVIZ config file to use',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Whether to start RVIZ'
    )

    declare_use_lidar_cmd = DeclareLaunchArgument(
        'use_lidar', default_value='true', description='Whether to start Lidar node'
    )

    joy_teleop_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ramson_dir, 'launch', 'joy_teleop.launch.py')
        )
    )

    rsp_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ramson_dir, 'launch', 'rsp.launch.py')
        )
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'rviz_config': rviz_config_file,
        }.items(),
    )

    lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('urg_node2'),
            'launch',
            'urg_node2.launch.py'
        )),
        condition=IfCondition(use_lidar),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'slam': slam,
            'map': map_yaml_file,
            'graph': graph_filepath,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'True',
            'use_respawn': 'False',
            'use_keepout_zones': 'False',
            'use_speed_zones': 'False',
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_graph_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(declare_use_lidar_cmd)
    ld.add_action(lidar_cmd)

    # ld.add_action(robot_state_publisher_node)
    # ld.add_action(joint_state_publisher_node)
    ld.add_action(joy_teleop_cmd)
    ld.add_action(rsp_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)

    return ld
