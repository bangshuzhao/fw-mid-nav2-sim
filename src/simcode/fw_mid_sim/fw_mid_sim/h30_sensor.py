from math import radians
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class H30Sensor(Node):
    """
    H30 IMU 仿真适配节点
    作用：订阅Gazebo仿真输出的 /imu/gazebo，做后处理、限幅、填充协方差，
    输出符合真实H30硬件规范的 /imu/data_raw 给EKF
    """
    def __init__(self):
        super().__init__('h30_sensor')
        # 获取包的share目录，读取传感器配置yaml（和你仿真xacro共用一套参数）
        share = Path(get_package_share_directory('fw_mid_sim'))
        h30 = yaml.safe_load((share / 'config/sensors.yaml').read_text())['h30']

        # 陀螺仪满量程：deg/s → rad/s
        self.gyro_limit = radians(h30['gyro_full_scale_deg_s'])
        # 加速度计满量程：g → m/s²
        self.accel_limit = 9.80665 * h30['accel_full_scale_g']

        # 陀螺仪噪声密度：deg/s/sqrt(Hz) → rad/s/sqrt(Hz)
        gyro_density = radians(h30['gyro_noise_deg_s_sqrt_hz'])
        # 加速度噪声密度：ug/sqrt(Hz) → m/s²/sqrt(Hz)
        accel_density = h30['accel_noise_ug_sqrt_hz'] * 9.80665e-6

        # 方差 = 噪声密度² × 带宽，和Gazebo噪声模型保持一致
        self.gyro_variance = gyro_density**2 * h30['gyro_bandwidth_hz']
        self.accel_variance = accel_density**2 * h30['accel_bandwidth_hz']

        # 发布：处理后的IMU原始数据，EKF/robot_localization 订阅这个话题
        self.publisher = self.create_publisher(
            Imu, '/imu/data_raw', qos_profile_sensor_data
        )
        # 订阅Gazebo桥接出来的原始仿真IMU话题 /imu/gazebo
        self.create_subscription(
            Imu, '/imu/gazebo', self.forward, qos_profile_sensor_data
        )

    def forward(self, msg):
        """
        回调函数：收到Gazebo IMU消息后做处理
        1. 固定frame_id
        2. 丢弃仿真给出的姿态（H30本身不输出姿态，只有陀螺+加速度）
        3. 对陀螺、加速度做硬件限幅
        4. 填充对角协方差矩阵，给EKF用
        """
        # 强制frame_id为imu_link，和TF对齐
        msg.header.frame_id = 'imu_link'

        # 重点：H30是6轴IMU，本身不输出姿态四元数
        # 所以把姿态置为无效，orientation_covariance[0]=-1 告诉EKF不要使用姿态
        msg.orientation.x = msg.orientation.y = msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0

        # 三轴循环：对加速度、角速度做硬件饱和限幅
        for axis in ('x', 'y', 'z'):
            a = getattr(msg.linear_acceleration, axis)
            w = getattr(msg.angular_velocity, axis)
            # 加速度钳位：[-accel_limit, accel_limit]
            setattr(msg.linear_acceleration, axis,
                    max(-self.accel_limit, min(self.accel_limit, a)))
            # 角速度钳位：[-gyro_limit, gyro_limit]
            setattr(msg.angular_velocity, axis,
                    max(-self.gyro_limit, min(self.gyro_limit, w)))

        # 填充协方差对角元：只给x/y/z轴，非对角项保持0
        # 索引 0,4,8 对应 3x3协方差矩阵对角线：xx, yy, zz
        for index in (0, 4, 8):
            msg.linear_acceleration_covariance[index] = self.accel_variance
            msg.angular_velocity_covariance[index] = self.gyro_variance

        # 发布处理完成的IMU原始数据
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = H30Sensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
