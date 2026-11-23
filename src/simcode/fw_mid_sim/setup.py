
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
        ('share/' + package_name + '/config',['config/geometry.yaml', 'config/controllers.yaml']),
    ],
    install_requires=['setuptools'],
)