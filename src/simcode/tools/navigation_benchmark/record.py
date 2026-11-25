#!/usr/bin/env python3
"""Record one FW-mid benchmark as timestamped JSON, compressed with gzip."""
import argparse
from collections import defaultdict
import gzip
import json
from math import atan2, cos, pi, sin
from pathlib import Path
from time import monotonic

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as PathMessage
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState


REFERENCE_OFFICE_ROUTES = (
    ('east', 6.81, 2.90, pi / 2),
    ('west', 1.63, 2.72, pi / 2),
    ('revisit_east', 6.81, 2.90, pi / 2),
    ('return', 0.05, 0.05, 0.0),
)
OFFICE_ROUTES = (
    ('east', 6.35, .70, 0.),
    ('west', 1.60, .70, 0.),
    ('revisit_east', 6.35, .70, 0.),
    ('return', .05, .05, 0.),
)
ROUTES = {
    'office': OFFICE_ROUTES,
    'office-reference': REFERENCE_OFFICE_ROUTES,
    'crossing': (('center_east', .80, 0., 0.),),
}
WHEELS = ('fl_steer_joint', 'fr_steer_joint', 'rl_steer_joint', 'rr_steer_joint')


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def planar(pose):
    q = pose.orientation
    yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return pose.position.x, pose.position.y, yaw


def stamped_goal(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x, pose.pose.position.y = x, y
    pose.pose.orientation.z = sin(yaw / 2)
    pose.pose.orientation.w = cos(yaw / 2)
    return pose


class Recorder(Node):
    def __init__(self, scenario):
        super().__init__('fw_mid_benchmark', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.data = defaultdict(list)
        self.latest = {}
        self.recording = False
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        for key, topic in (('truth', '/ground_truth/odom'), ('ekf', '/odometry/filtered')):
            self.watch(Odometry, topic, lambda msg, key=key: self.odom(key, msg))
        for topic in ('/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel_feasible'):
            self.watch(Twist, topic, lambda msg, key=topic: self.command(key, msg))
        self.watch(PathMessage, '/plan', self.plan)
        self.watch(JointState, '/joint_states', self.steering)
        if scenario == 'crossing':
            self.watch(Twist, '/cmd_vel', lambda msg: self.command('/cmd_vel', msg))
            self.watch(
                Odometry, '/traffic/crossing_center/truth',
                lambda msg: self.odom('pedestrian', msg),
            )
            self.watch(Contacts, '/traffic/crossing_center/contacts', self.contacts)

    @property
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def watch(self, kind, topic, callback):
        self.create_subscription(kind, topic, callback, qos_profile_sensor_data)

    def add(self, key, stamp, *values):
        self.latest[key] = (stamp, *values)
        if self.recording:
            self.data[key].append(self.latest[key])

    def odom(self, key, msg):
        velocity = msg.twist.twist
        self.add(key, seconds(msg.header.stamp), *planar(msg.pose.pose),
                 velocity.linear.x, velocity.linear.y, velocity.angular.z)

    def command(self, key, msg):
        self.add(key, self.now, msg.linear.x, msg.angular.z)

    def plan(self, msg):
        self.add('plan', seconds(msg.header.stamp), [planar(p.pose) for p in msg.poses])

    def steering(self, msg):
        indices = {name: i for i, name in enumerate(msg.name)}
        if all(name in indices and indices[name] < len(msg.position) for name in WHEELS):
            self.add('steering', seconds(msg.header.stamp), *(msg.position[indices[name]] for name in WHEELS))

    def contacts(self, msg):
        for item in msg.contacts:
            self.add('contact', seconds(msg.header.stamp), item.collision1.name, item.collision2.name)

    def feedback(self, msg):
        data = msg.feedback
        self.add('feedback', self.now, *planar(data.current_pose.pose),
                 data.distance_remaining, data.number_of_recoveries)

    def wait(self, condition, timeout):
        deadline = monotonic() + timeout
        while rclpy.ok() and monotonic() < deadline:
            if condition():
                return True
            rclpy.spin_once(self, timeout_sec=.05)
        return condition()

    def ready(self, scenario):
        client = self.create_client(GetState, '/bt_navigator/get_state')
        deadline = monotonic() + 45
        active = False
        while monotonic() < deadline and rclpy.ok():
            if client.wait_for_service(timeout_sec=1):
                future = client.call_async(GetState.Request())
                if self.wait(future.done, 2) and future.result().current_state.id == 3:
                    active = True
                    break
                self.wait(lambda: False, .25)
        self.destroy_client(client)
        required = {'truth', 'ekf', 'steering'}
        if scenario == 'crossing':
            required.add('pedestrian')
        if not active or not self.client.wait_for_server(timeout_sec=5):
            raise RuntimeError('bt_navigator did not activate; check navigation.log')
        if not self.wait(lambda: self.now > 0 and required <= self.latest.keys(), 10):
            raise RuntimeError(f'Missing clock or data: {sorted(required - self.latest.keys())}')
        if scenario == 'crossing':
            if abs(self.latest['pedestrian'][2] + 1.35) > .05:
                raise RuntimeError('Restart the narrow simulation to reset the pedestrian')
            if not self.wait(lambda: self.count_publishers('/traffic/crossing_center/contacts') > 0, 5):
                raise RuntimeError('Pedestrian contact bridge unavailable')

    def trial_route(self, route, timeout):
        name, *goal = route
        self.data.clear()
        for key in ('truth', 'ekf', 'steering', 'pedestrian'):
            if key in self.latest:
                self.data[key].append(self.latest[key])
        self.recording = True
        start = self.now
        request = NavigateToPose.Goal()
        request.pose = stamped_goal(*goal)
        request.pose.header.stamp = self.get_clock().now().to_msg()
        sent = self.client.send_goal_async(request, feedback_callback=self.feedback)
        if not self.wait(sent.done, 10) or not sent.result().accepted:
            raise RuntimeError(f'Navigation goal not accepted: {name}')
        handle = sent.result()
        result = handle.get_result_async()
        status = -1
        if self.wait(result.done, timeout):
            status = result.result().status
        else:
            cancel = handle.cancel_goal_async()
            self.wait(cancel.done, 5)
            self.wait(result.done, 5)
        end = self.now
        # Keep a bounded settling window before starting the next navigation goal.
        def settled():
            stamp, *angles = self.latest['steering']
            return self.now >= end + 6.0 or (
                stamp >= end + 1.0 and max(abs(angle) for angle in angles) <= .015
            )

        self.wait(settled, 10)
        self.recording = False
        print(f'{name}: status={status}, sim={end - start:.2f}s', flush=True)
        return dict(name=name, goal=goal, start=start, end=end, status=status, data=dict(self.data))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', choices=ROUTES)
    parser.add_argument('output', type=Path)
    parser.add_argument('--timeout', type=float, default=110, help='wall-clock seconds per navigation goal')
    args = parser.parse_args()
    with gzip.open(args.output, 'xt', encoding='utf-8') as stream:
        rclpy.init()
        node = Recorder(args.scenario)
        runs = []
        try:
            node.ready(args.scenario)
            for route in ROUTES[args.scenario]:
                runs.append(node.trial_route(route, args.timeout))
                if runs[-1]['status'] != GoalStatus.STATUS_SUCCEEDED:
                    break
        finally:
            json.dump(dict(scenario=args.scenario, runs=runs), stream, allow_nan=False, separators=(',', ':'))
            node.destroy_node()
            rclpy.try_shutdown()
    return 0 if len(runs) == len(ROUTES[args.scenario]) and all(
        run['status'] == GoalStatus.STATUS_SUCCEEDED for run in runs) else 1


if __name__ == '__main__':
    raise SystemExit(main())
