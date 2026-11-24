from math import atan2, cos, hypot, pi, sin
from pathlib import Path
from time import monotonic
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
WHEEL_POSITIONS = (
    (0.20, 0.21),    # FL
    (0.20, -0.21),   # FR
    (-0.20, 0.21),   # RL
    (-0.20, -0.21),  # RR
)
WHEEL_RADIUS = 0.12


def wheel_targets(vx, wz, current_angles,
                  positions=WHEEL_POSITIONS, radius=WHEEL_RADIUS):
    steering = []
    drive = []

    for (x, y), current in zip(positions, current_angles):
        forward = vx - wz * y
        left = wz * x
        speed = hypot(forward, left)

        if speed < 1e-5:
            steering.append(current)
            drive.append(0.0)
            continue

        angle = atan2(left, forward)

        # 轮子反转即可沿同一条线倒行，转角保持在 ±90°。
        if angle > pi / 2:
            angle -= pi
            speed = -speed
        elif angle < -pi / 2:
            angle += pi
            speed = -speed

        steering.append(angle)
        drive.append(speed / radius)

    return steering, drive
def body_velocity(angles, rates,
                  positions=WHEEL_POSITIONS, radius=WHEEL_RADIUS):
    sum_forward = 0.0
    sum_left = 0.0
    yaw_numerator = 0.0
    yaw_denominator = 0.0

    for (x, y), angle, rate in zip(positions, angles, rates):
        forward = radius * rate * cos(angle)
        left = radius * rate * sin(angle)

        sum_forward += forward
        sum_left += left
        yaw_numerator += x * left - y * forward
        yaw_denominator += x * x + y * y

    count = len(positions)
    return (
        sum_forward / count,
        sum_left / count,
        yaw_numerator / yaw_denominator,
    )
class BaseControl(Node):
    def __init__(self):
        # 调用父类Node构造，节点名字 fw_mid_base_control
        super().__init__('fw_mid_base_control')
        # 找到fw_mid_sim功能包的share目录，用来读yaml配置
        share = Path(get_package_share_directory('fw_mid_sim'))
        # 读取 geometry.yaml，里面存 half_wheelbase、half_track、wheel_radius
        geometry = yaml.safe_load((share / 'config/geometry.yaml').read_text())
        # half_wheelbase：前后轮到中心的X距离；half_track：左右轮到中心Y距离
        a, b = geometry['half_wheelbase'], geometry['half_track']
        # 从yaml重新覆盖四轮坐标，不再用上面硬编码的 WHEEL_POSITIONS
        self.positions = ((a, b), (a, -b), (-a, b), (-a, -b))
        self.radius = geometry['wheel_radius']
        # 声明ROS参数：最小转弯半径，默认0.45m；用来限制最大曲率
        self.turn_radius = self.declare_parameter(
            'minimum_turning_radius', 0.45).value
        # 是否广播 odom → base_footprint 的TF变换，默认打开
        publish_tf = self.declare_parameter('publish_odom_tf', True).value
        # 期望车体速度（从cmd_vel来）
        self.vx = self.wz = 0.0
        self.cmd_time = float('-inf')
        # 保存最新读到的四轮转向角（来自joint_states反馈）
        self.angles = None
        # 里程计积分位姿：odom坐标系下 x,y,yaw
        self.x = self.y = self.yaw = 0.0
        self.last_stamp = None
        # 发给转向组控制器：4个目标转角 Float64MultiArray
        self.steering = self.create_publisher(
            Float64MultiArray, 'steering_controller/commands', 10)
        # 发给驱动组控制器：4个目标驱动角速度 Float64MultiArray
        self.drive = self.create_publisher(
            Float64MultiArray, 'drive_controller/commands', 10)
        # 发布odom里程计消息
        self.odom = self.create_publisher(Odometry, 'odom', 10)
        # TF广播器，可选关闭
        self.tf = TransformBroadcaster(self) if publish_tf else None

        # 订阅 /cmd_vel，回调 receive_cmd：接收上层速度指令
        self.create_subscription(Twist, 'cmd_vel', self.receive_cmd, 10)
        # 订阅 /joint_states，回调 receive_feedback：读取Gazebo关节反馈
        self.create_subscription(JointState, 'joint_states', self.receive_feedback, 10)
        # ========== 定时器：控制主循环 50Hz（0.02s一次） ==========
        self.create_timer(0.02, self.control)

    def receive_cmd(self, msg):
        self.vx = msg.linear.x
        # 曲率限制：wz最大不能超过 vx / min_turn_radius
        # 曲率 k = ω / v；最小转弯半径 R = v/ω → ω_max = v/R
        limit = abs(self.vx) / self.turn_radius
        # 钳位wz，不能超出这个极限，防止转向角超机械极限
        self.wz = max(-limit, min(limit, msg.angular.z))
        self.cmd_time = monotonic()

    def control(self):
        # 指令中断就停止驱动，转向保持当前目标 先给方向，再按实际转向误差放开轮速。
        if self.angles is None or monotonic() - self.cmd_time > 0.30:
            # 驱动轮全部置0；转向不主动动（保持当前角度）
            self.drive.publish(Float64MultiArray(data=[0.0] * 4))
            return
        # 调用正运动学，算出目标转角、目标驱动角速度
        angles, rates = wheel_targets(
            self.vx, self.wz, self.angles, self.positions, self.radius)
        # 下发转向角目标
        self.steering.publish(Float64MultiArray(data=angles))

        # 轮子还没对准时，先降低驱动轮速。
        # 找四个轮子，【目标转角和当前实际转角之间最大误差】
        error = max(abs(a - b) for a, b in zip(angles, self.angles))
        # 速度缩放因子：
        # error >=0.4rad → scale=0，轮子不动，等转向打到位
        # error <=0.1rad → scale=1，全速跑
        # 中间线性插值：(0.4-error)/0.3
        scale = max(0.0, min(1.0, (0.40 - error) / 0.30))
        # 驱动轮速度乘以scale：转向没对齐前，限速甚至停转，防止打滑
        self.drive.publish(Float64MultiArray(data=[scale * q for q in rates]))
        # 如果轮子角度还没转到位就直接给全速，仿真里会疯狂打滑、运动学失效。先等舵打到位再慢慢给速度。
    def receive_feedback(self, msg): # 读取Gazebo关节反馈 即从实际轮子反推整车运动
        # 保存当前四轮转向角
        self.angles = [msg.position[i] for i in range(8)]
        # 保存当前四轮驱动角速度
        self.rates = [msg.velocity[i] for i in range(8)]
        indices = {name: i for i, name in enumerate(msg.name)}
        wheels = ('fl', 'fr', 'rl', 'rr')
        # 依次取出四个转向关节的位置 → self.angles（给control下一轮用）
        self.angles = [msg.position[indices[f'{w}_steer_joint']] for w in wheels]
        # 依次取出四个驱动关节的速度 → self.rates（给control下一轮用）
        rates = [msg.velocity[indices[f'{w}_drive_joint']] for w in wheels]
        # 调用逆运动学，从轮子真实状态，反推车体真实 vx, vy, wz
        vx, vy, wz = body_velocity(self.angles, rates, self.positions, self.radius)

        stamp = msg.header.stamp
        t = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self.last_stamp is not None:
            dt = (t - self.last_stamp) * 1e-9
            if dt > 0.0:
                # 里程计积分位姿：odom坐标系下 x,y,yaw
                heading = self.yaw + 0.5 * wz * dt
                # 车体速度(vx,vy)旋转到odom世界坐标系，累加到位姿
                self.x += (vx * cos(heading) - vy * sin(heading)) * dt
                self.y += (vx * sin(heading) + vy * cos(heading)) * dt
                # 偏航角积分：yaw
                self.yaw += wz * dt

        self.last_stamp = t
        # 组装并且发布odom消息 + TF
        self.publish_odom(stamp, vx, vy, wz)

    def publish_odom(self, stamp, vx, vy, wz):
        # 组装并且发布odom消息 + TF 
        #body_velocity() 给的是现在走多快。要知道这段时间走了多少，就用相邻两次关节反馈的时间戳求出 dt。例如估算到 wz=0.30 rad/s，两次反馈相隔 0.10 s，车头就转了约 0.03 rad。
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'

        pose = msg.pose.pose
        pose.position.x, pose.position.y = self.x, self.y
        # yaw转四元数（2D平面，绕Z轴）
        pose.orientation.z = sin(self.yaw / 2)
        pose.orientation.w = cos(self.yaw / 2)

        # twist部分放【车体坐标系】的速度
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz

        for i, variance in enumerate((0.04, 0.04, 1e3, 1e3, 1e3, 0.06)):
            msg.pose.covariance[7 * i] = variance

        for i, variance in enumerate((0.02, 0.02, 1e3, 1e3, 1e3, 0.04)):
            msg.twist.covariance[7 * i] = variance
        self.odom.publish(msg)
        # 如果开启tf广播，发布 odom → base_footprint 的TF变换
        if self.tf is not None:
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = msg.child_frame_id
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = pose.orientation
            self.tf.sendTransform(transform)


def main():
    # ROS2初始化
    rclpy.init()
    # 实例化控制节点
    node = BaseControl()
    # 节点自旋，持续处理回调、定时器
    rclpy.spin(node)
    # 退出时清理
    node.destroy_node()
    rclpy.shutdown()
