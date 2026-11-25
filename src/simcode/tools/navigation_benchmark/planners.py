#!/usr/bin/env python3
"""Request equal start/goal pairs from the two office global planners."""
import argparse
from math import hypot
from pathlib import Path
from time import monotonic

from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
import numpy as np
import rclpy
from rclpy.action import ActionClient

from analyze import FOOTPRINT, OFFICE_ORIGIN, WORLD, collisions, reversals, transform, write_csv
from record import OFFICE_ROUTES, Recorder, planar, seconds, stamped_goal


def request_path(node, client, planner, start, goal):
    request = ComputePathToPose.Goal()
    request.planner_id, request.use_start = planner, True
    request.start, request.goal = stamped_goal(*start), stamped_goal(*goal)
    begin = monotonic()
    future = client.send_goal_async(request)
    if not node.wait(future.done, 8) or not future.result().accepted:
        raise RuntimeError(f'{planner} did not accept the request')
    handle = future.result()
    future = handle.get_result_async()
    if not node.wait(future.done, 12):
        canceled = handle.cancel_goal_async()
        node.wait(canceled.done, 5)
        raise RuntimeError(f'{planner} did not finish')
    response = future.result()
    return response.status, (monotonic() - begin) * 1000, response.result


def measure(poses, goal, geometry):
    if len(poses) < 2:
        return {}
    path = np.asarray([planar(item.pose) for item in poses])
    dx, dy = np.diff(path[:, :2], axis=0).T
    travel = np.hypot(dx, dy)
    heading = path[:-1, 2]
    longitudinal = dx * np.cos(heading) + dy * np.sin(heading)
    direction = np.sign(longitudinal[travel > .005])
    ox, oy = OFFICE_ORIGIN
    gap = min(transform(FOOTPRINT, x + ox, y + oy, yaw).distance(geometry) for x, y, yaw in path)
    return dict(path_m=float(travel.sum()), reversing_points=reversals(direction), min_clearance_m=gap,
                goal_xy_m=hypot(path[-1, 0] - goal[0], path[-1, 1] - goal[1]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--world', type=Path, default=WORLD, help='office SDF used for clearance')
    parser.add_argument('--repeats', type=int, default=3, choices=range(1, 21), metavar='1..20')
    args = parser.parse_args()
    geometry = collisions(args.world)
    rows = []
    rclpy.init()
    node = Recorder('office')
    try:
        node.ready('office')
        client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        if not client.wait_for_server(timeout_sec=8):
            raise RuntimeError('Planner server unavailable')
        start = (0., 0., 0.)
        for route, *goal in OFFICE_ROUTES:
            for repeat in range(1, args.repeats + 1):
                for planner in ('GridBased', 'OfficeLattice'):
                    status, wall_ms, result = request_path(node, client, planner, start, goal)
                    row = dict(route=route, planner=planner, repeat=repeat, status=status, round_trip_ms=wall_ms)
                    if status == GoalStatus.STATUS_SUCCEEDED:
                        row.update(planning_ms=1000 * seconds(result.planning_time),
                                   **measure(result.path.poses, goal, geometry))
                    rows.append(row)
                    print(f'{route} {planner} #{repeat}: status={status}', flush=True)
            start = goal
    finally:
        if rows:
            write_csv(args.output, rows)
        node.destroy_node()
        rclpy.try_shutdown()
    if any(not any(row['status'] == GoalStatus.STATUS_SUCCEEDED and row['planner'] == planner
                   for row in rows) for planner in ('GridBased', 'OfficeLattice')):
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
