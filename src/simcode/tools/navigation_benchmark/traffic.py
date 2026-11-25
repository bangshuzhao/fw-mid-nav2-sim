#!/usr/bin/env python3
"""Measure the central crossing from time-aligned robot and pedestrian truth."""
import argparse
from pathlib import Path

import numpy as np

from analyze import goal_error, read_recording, write_csv


LANE_HALF_WIDTH = .60
PERSON_RADIUS = .23
SUCCEEDED = 4


def first_time(stamps, mask):
    matches = stamps[mask]
    return float(matches[0]) if len(matches) else None


def score(run):
    data = run['data']
    robot = np.asarray(data['truth'])
    person = np.asarray(data['pedestrian'])
    start = max(run['start'], robot[0, 0], person[0, 0])
    end = min(robot[-1, 0], person[-1, 0])
    stamps = np.arange(start, end, .02)
    if not len(stamps):
        raise ValueError('Robot and pedestrian data have no common sample window')
    x, y, yaw, vx, vy = [np.interp(stamps, robot[:, 0], column) for column in
                        (robot[:, 1], robot[:, 2], np.unwrap(robot[:, 3]), robot[:, 4], robot[:, 5])]
    px, py = [np.interp(stamps, person[:, 0], person[:, i]) for i in (1, 2)]
    dx, dy = px - x, py - y
    local_x = dx * np.cos(yaw) + dy * np.sin(yaw)
    local_y = -dx * np.sin(yaw) + dy * np.cos(yaw)
    clearance = np.hypot(np.maximum(np.abs(local_x) - .34, 0),
                         np.maximum(np.abs(local_y) - .275, 0)) - PERSON_RADIUS
    speed = np.hypot(vx, vy)
    lane_edge = LANE_HALF_WIDTH + PERSON_RADIUS
    entered = first_time(stamps, py >= -lane_edge)
    center = first_time(stamps, py >= 0)
    exited = first_time(stamps, py > lane_edge)
    stopped = None
    approach = first_time(stamps, (speed >= .12) & (stamps <= entered)) if entered is not None else None
    if approach is not None:
        encounter = (stamps >= entered) & (py <= lane_edge) & (np.abs(dx) <= 1.6)
        stopped = first_time(stamps, encounter & (speed <= .035))
    resumed = first_time(stamps, (stamps > stopped) & (speed >= .12)) if stopped is not None else None
    feasible = np.asarray(data.get('/cmd_vel_feasible', []))
    command_stop = None
    if len(feasible) and entered is not None and exited is not None:
        for stamp, vx_cmd, wz_cmd in data.get('/cmd_vel', []):
            i = np.searchsorted(feasible[:, 0], stamp, side='right') - 1
            if (entered <= stamp <= exited and abs(vx_cmd) + abs(wz_cmd) < 1e-6 and
                    i >= 0 and stamp - feasible[i, 0] <= .15 and feasible[i, 1] > .05):
                command_stop = stamp
                break
    goal_xy, goal_yaw = goal_error(robot[-1], run['goal'], (0., 0.))
    travel = float(person[-1, 2] - person[0, 2])
    gap = float(clearance.min())
    contacts = len(data.get('contact', []))
    stop_before_center = stopped is not None and center is not None and stopped <= center
    passed = (run['status'] == SUCCEEDED and travel >= 2.6 and contacts == 0 and gap > 0
              and command_stop is not None and stop_before_center and exited is not None and resumed is not None)
    return dict(route=run['name'], status=run['status'], passed=passed, sim_s=run['end'] - run['start'],
                pedestrian_travel_m=travel, min_clearance_m=gap, contact_events=contacts,
                entry_sim_s=entered, zero_command_sim_s=command_stop, stop_sim_s=stopped, center_sim_s=center, exit_sim_s=exited,
                resume_sim_s=resumed, stop_before_center=stop_before_center,
                entry_to_stop_s=stopped - entered if stopped is not None else None,
                exit_to_resume_s=resumed - exited if resumed is not None and exited is not None else None,
                goal_xy_m=goal_xy, goal_yaw_deg=goal_yaw,
                recoveries=max((sample[-1] for sample in data.get('feedback', [])), default=None))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    recording = read_recording(args.recording)
    if recording['scenario'] != 'crossing' or len(recording['runs']) != 1:
        parser.error('expected one crossing run')
    result = score(recording['runs'][0])
    write_csv(args.output, [result])
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
