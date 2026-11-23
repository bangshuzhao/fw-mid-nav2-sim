from pathlib import Path

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_sim(context):
    """
    自定义构造函数，由OpaqueFunction调用
    context：launch运行时上下文，在这里可以拿到LaunchConfiguration解析后的参数
    """
    # 找到fw_mid_sim功能包的share目录，返回Path对象
    share = Path(get_package_share_directory('fw_mid_sim'))
    # 1. 拼接xacro文件完整路径：share/urdf/fw_mid.urdf.xacro，转成字符串传给xacro
    # 2. xacro.process_file() 解析、展开宏，得到xml文档对象
    # 3. .toxml() 转为纯字符串，就是完整URDF，交给robot_state_publisher
    xml = xacro.process_file(str(share / 'urdf/fw_mid.urdf.xacro')).toxml()
    sim_time = {'use_sim_time': True}

    gazebo_cmd = [
        'ign', 'gazebo', '--force-version', '6', '-r',
        str(share / 'worlds/workshop.sdf'),
    ]
    if LaunchConfiguration('gui').perform(context) == 'false':
        gazebo_cmd.append('-s')
    # 构造ExecuteProcess动作：启动Gazebo进程
    gazebo = ExecuteProcess(
        cmd=gazebo_cmd,
        output='screen',
        additional_env={
            'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': get_package_prefix('gz_ros2_control') + '/lib',
        },
    )
    # 启动 robot_state_publisher（RSP）节点
    robot = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[sim_time, {'robot_description': xml}],
    )
    # 启动ros_gz_bridge的parameter_bridge，做Gazebo和ROS2的消息桥接
    clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )
    # 启动ros_gz_sim的create节点：把机器人模型spawn（生成）进Gazebo世界
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-world', 'fw_mid_workshop', '-name', 'fw_mid',
                   '-topic', '/robot_description', '-z', '0.025'],
    )
    # 启动controller_manager的spawner，加载ros2_control控制器
    controllers = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', 'steering_controller', 'drive_controller',
                   '--controller-manager-timeout', '25', '--activate-as-group'],
    )
    # 启动你自己写的base_control节点（正逆运动学、odom积分的Python程序）
    base = Node(
        package='fw_mid_sim',
        executable='base_control',
        output='screen',
        parameters=[sim_time],
    )
    # 闭包函数：生成一个回调函数
    def after_success(next_action):
        return lambda event, context: [next_action] if event.returncode == 0 else []
    # 返回本launch所有动作列表
    return [
        gazebo, robot, clock,# 这三个并行立刻启动：Gazebo、RSP、时钟桥
        # 事件1：监听spawn进程退出，spawn成功 → 启动controllers控制器加载器
        RegisterEventHandler(OnProcessExit(
            target_action=spawn, on_exit=after_success(controllers))),
        # 事件2：监听controllers进程退出，控制器加载成功 → 启动base_control控制节点
        RegisterEventHandler(OnProcessExit(
            target_action=controllers, on_exit=after_success(base))),
        spawn,# 同时启动spawn，生成小车模型
    ]

# ROS2 launch的入口函数，launch框架会自动调用这个函数
def generate_launch_description():
    return LaunchDescription([
        # 声明启动参数gui，默认true，只能填true/false
        # 启动时可以覆盖：ros2 launch fw_mid_sim sim.launch.py gui:=false
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=make_sim),
    ])