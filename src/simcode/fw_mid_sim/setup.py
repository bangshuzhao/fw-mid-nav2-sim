
from setuptools import setup

package_name = 'fw_mid_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/fw_mid.urdf.xacro']),
        ('share/' + package_name + '/config', [
            'config/geometry.yaml', 'config/controllers.yaml', 'config/sensors.yaml',
            'config/bridge.yaml', 'config/estimation.yaml', 'config/slam.yaml',
            'config/nav2_step9.yaml', 'config/nav2_step9_traffic.yaml', 'config/traffic_bridge.yaml',
        ]),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py', 'launch/navigate_step9.launch.py']),
        ('share/' + package_name + '/behavior_trees', ['behavior_trees/navigate_to_pose_step9.xml']),
        ('share/' + package_name + '/maps', ['maps/narrow_office.yaml', 'maps/narrow_office.pgm']),
        ('share/' + package_name + '/worlds', [
            'worlds/workshop.sdf', 'worlds/narrow_office.sdf', 'worlds/office_floor.obj',
            'worlds/wood_floor_diffuse.jpg', 'worlds/wood_floor_nor_gl.jpg',
        ]),
    ],
    install_requires=['setuptools'],
        entry_points={
        'console_scripts': [
            'base_control = fw_mid_sim.base_control:main',
            'h30_sensor = fw_mid_sim.h30_sensor:main',
            'kinematics_gate = fw_mid_sim.navigation_io:run_gate',
            'navigation_eta = fw_mid_sim.navigation_io:run_eta',
        ],
    },
)
