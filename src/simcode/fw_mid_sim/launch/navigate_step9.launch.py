from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def make_navigation(context):
    project = Path(__file__).resolve().parents[1]
    params = [LaunchConfiguration('params').perform(context)]
    map_file = LaunchConfiguration('map').perform(context)
    stage = LaunchConfiguration('stage').perform(context)
    tree = str(project / 'behavior_trees' / 'navigate_to_pose_step9.xml')
    if stage == 'traffic':
        params.append(str(project / 'config/nav2_step9_traffic.yaml'))

    managed = [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[*params, {'yaml_filename': map_file}], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=params, output='screen'),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=params, output='screen'),
    ]
    names = ['map_server', 'amcl', 'planner_server']
    extras = []

    if stage == 'traffic':
        managed.append(Node(
            package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor', parameters=params, output='screen'))
        names.append('collision_monitor')

    velocity_remap = []
    if stage in ('smooth', 'traffic'):
        velocity_remap = [('cmd_vel', '/cmd_vel_nav')]
        managed.append(Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', parameters=params,
            remappings=velocity_remap, output='screen'))
        names.append('velocity_smoother')
        extras.append(Node(
            package='fw_mid_sim', executable='kinematics_gate', output='screen'))
        extras.append(Node(
            package='fw_mid_sim', executable='navigation_eta',
            parameters=[{'use_sim_time': True}], output='screen'))

    managed.extend([
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', parameters=params,
             remappings=velocity_remap, output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[*params, {
                 'default_nav_to_pose_bt_xml': tree,
                 'default_nav_through_poses_bt_xml': tree,
             }], output='screen'),
    ])
    names.extend(['controller_server', 'bt_navigator'])

    return [
        *managed, *extras,
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', parameters=[{
                 'use_sim_time': True,
                 'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                 'node_names': names,
             }], output='screen'),
    ]


def generate_launch_description():
    project = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('params', default_value=str(project / 'config/nav2_step9.yaml')),
        DeclareLaunchArgument('stage', default_value='direct',
                              choices=['direct', 'smooth', 'traffic']),
        DeclareLaunchArgument('autostart', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=make_navigation),
    ])
