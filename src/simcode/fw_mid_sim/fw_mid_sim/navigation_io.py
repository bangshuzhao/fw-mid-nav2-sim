from collections import deque
from math import ceil, isfinite

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker

class KinematicsGate(Node):
    def __init__(self):
        super().__init__('fw_mid_kinematics_gate')
        self.radius = self.declare_parameter('minimum_turning_radius', 0.45).value
        self.output = self.create_publisher(Twist, '/cmd_vel_feasible', 10)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self.project, 10)

    def project(self, source):
        cmd = Twist()
        cmd.linear.x = source.linear.x

        # 低速时同步限制偏航角速度，保持最小转弯半径。
        limit = abs(cmd.linear.x) / self.radius
        cmd.angular.z = max(-limit, min(limit, source.angular.z))
        self.output.publish(cmd)


def run_gate():
    rclpy.init()
    node = KinematicsGate()
    rclpy.spin(node)

class NavigationEta(Node):
    def __init__(self):
        super().__init__('office_navigation_eta')
        self.samples = deque()
        self.goal = None
        self.recoveries = 0
        self.terminal = set()
        self.rate = None
        self.eta = None

        self.marker = self.create_publisher(Marker, '/office_navigation/eta', 1)
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            '/navigate_to_pose/_action/feedback', self.feedback, 10)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self.status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def status(self, message):
        self.terminal = {
            bytes(entry.goal_info.goal_id.uuid)
            for entry in message.status_list
            if entry.status in (
                GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_ABORTED,
                GoalStatus.STATUS_CANCELED,
            )
        }
        if self.goal in self.terminal:
            self.goal = None
            self.marker.publish(self.make_marker(Marker.DELETE))

    def feedback(self, message):
        sample = message.feedback
        goal = bytes(message.goal_id.uuid)
        if goal in self.terminal or not sample.current_pose.header.frame_id:
            return

        elapsed = sample.navigation_time.sec + sample.navigation_time.nanosec * 1e-9
        distance = sample.distance_remaining
        if not isfinite(distance) or distance < 0:
            return

        previous = self.samples[-1] if self.samples else None
        if (goal != self.goal or sample.number_of_recoveries != self.recoveries or
                (previous and (elapsed < previous[0] or abs(distance - previous[1]) > 0.6))):
            self.samples.clear()
            self.rate = self.eta = None
        self.goal = goal
        self.recoveries = sample.number_of_recoveries

        # 最多约每 0.45 秒采样一次，保留最近 3.5 秒的推进记录。
        if self.samples and elapsed - self.samples[-1][0] < 0.45:
            return
        self.samples.append((elapsed, distance))
        while len(self.samples) > 1 and elapsed - self.samples[0][0] > 3.5:
            self.samples.popleft()

        duration = elapsed - self.samples[0][0]
        progress = self.samples[0][1] - distance
        if duration < 1.25 or not any(gap > 0.08 for _, gap in self.samples):
            text = 'Estimating arrival'
        elif distance <= 0.08:
            text = 'Aligning / stopping'
        elif progress < max(0.045, 0.025 * duration):
            self.rate = self.eta = None
            text = 'Waiting for progress'
        else:
            speed = min(0.48, progress / duration)
            self.rate = speed if self.rate is None else 0.8 * self.rate + 0.2 * speed
            estimate = distance / self.rate
            self.eta = estimate if self.eta is None else 0.78 * self.eta + 0.22 * estimate
            text = f'ETA ~{2 * ceil(self.eta / 2)} s' if self.eta < 120 else 'ETA >2 min'

        marker = self.make_marker(Marker.ADD)
        marker.header.frame_id = sample.current_pose.header.frame_id
        marker.pose.position.x = sample.current_pose.pose.position.x
        marker.pose.position.y = sample.current_pose.pose.position.y
        marker.pose.position.z = sample.current_pose.pose.position.z + 1.16
        marker.text = text
        self.marker.publish(marker)

    @staticmethod
    def make_marker(action):
        marker = Marker(
            ns='office_navigation', id=0,
            type=Marker.TEXT_VIEW_FACING, action=action)
        marker.header.frame_id = 'map'
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.20
        marker.color.r = 1.0
        marker.color.g = 0.95
        marker.color.b = 0.73
        marker.color.a = 1.0
        marker.lifetime.sec = 2
        return marker


def run_eta():
    rclpy.init()
    node = NavigationEta()
    rclpy.spin(node)