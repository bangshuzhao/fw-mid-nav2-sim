#!/usr/bin/env python3
"""Measure the recorded office runs against the static world collision geometry."""
import argparse
import csv
import gzip
import json
from math import degrees, hypot, pi
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
from shapely import affinity
from shapely.geometry import LineString, Point, box
from shapely.ops import unary_union


WORLD = Path(__file__).resolve().parents[2] / 'fw_mid_sim/worlds/office.sdf'
OFFICE_ORIGIN = (-4.50, -1.15)
FOOTPRINT = box(-.34, -.275, .34, .275)


def read_recording(path):
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        return json.load(stream)


def write_csv(path, rows):
    fields = list(dict.fromkeys(field for row in rows for field in row))
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def pose(element):
    return tuple(map(float, element.findtext('pose', '0 0 0 0 0 0').split()))


def transform(shape, x, y, yaw):
    rotated = affinity.rotate(shape, yaw, origin=(0, 0), use_radians=True)
    return affinity.translate(rotated, x, y)


def collisions(world):
    shapes = []
    for model in ElementTree.parse(world).findall('./world/model'):
        if model.get('name') == 'oak_floor':
            continue
        mx, my, mz, _, _, ma = pose(model)
        for link in model.findall('link'):
            lx, ly, lz, _, _, la = pose(link)
            for item in link.findall('collision'):
                x, y, z, _, _, yaw = pose(item)
                size = item.findtext('geometry/box/size')
                if size is not None:
                    width, depth, height = map(float, size.split())
                    shape = box(-width / 2, -depth / 2, width / 2, depth / 2)
                else:
                    radius = float(item.findtext('geometry/cylinder/radius'))
                    height = float(item.findtext('geometry/cylinder/length'))
                    shape = Point(0, 0).buffer(radius, resolution=16)
                elevation = mz + lz + z
                if elevation - height / 2 <= .92 and elevation + height / 2 >= .02:
                    shapes.append(transform(transform(transform(shape, x, y, yaw), lx, ly, la), mx, my, ma))
    return unary_union(shapes)


def goal_error(sample, goal, origin):
    _, x, y, yaw = sample[:4]
    gx, gy, angle = goal
    return hypot(x - origin[0] - gx, y - origin[1] - gy), abs(degrees((yaw - angle + pi) % (2 * pi) - pi))


def reversals(speed, threshold=.035):
    directions = np.sign(speed[np.abs(speed) >= threshold])
    return int(np.count_nonzero(np.diff(directions)))


def score(run, obstacles):
    data = run['data']
    truth = np.asarray(data['truth'])
    motion = truth[(truth[:, 0] >= run['start']) & (truth[:, 0] <= run['end'])]
    if len(motion) < 2:
        raise ValueError(f"Too few truth samples for {run['name']}")
    stamps = np.arange(motion[0, 0], motion[-1, 0], .05)
    vx = np.interp(stamps, motion[:, 0], motion[:, 4])
    goal_xy, goal_yaw = goal_error(truth[-1], run['goal'], OFFICE_ORIGIN)
    footprints = [transform(FOOTPRINT, x, y, yaw) for _, x, y, yaw, *_ in motion]
    clearance = [body.distance(obstacles) for body in footprints]
    steering = [sample for sample in data['steering'] if sample[0] >= run['end']]
    settled = [sample[0] for sample in steering if max(abs(angle) for angle in sample[1:]) <= .015]
    row = dict(route=run['name'], status=run['status'], sim_s=run['end'] - run['start'],
               recoveries=max((f[-1] for f in data.get('feedback', [])), default=None),
               min_clearance_m=min(clearance), overlapping_samples=sum(body.intersects(obstacles) for body in footprints),
               reversals=reversals(vx), goal_xy_m=goal_xy, goal_yaw_deg=goal_yaw,
               final_steering_max_deg=max(abs(degrees(a)) for a in steering[-1][1:]) if steering else None,
               first_centered_after_goal_s=settled[0] - run['end'] if settled else None)
    if len(vx) >= 5:
        smoothed = np.convolve(vx, np.ones(5) / 5, mode='valid')
        if len(smoothed) >= 3:
            row['longitudinal_jerk_p95_mps3'] = float(np.percentile(np.abs(np.diff(smoothed, n=2)) / .05**2, 95))
    plans = data.get('plan', [])
    if plans and len(plans[0][1]) >= 2:
        origin_x, origin_y = OFFICE_ORIGIN
        line = LineString([(x + origin_x, y + origin_y) for x, y, _ in plans[0][1]])
        errors = [line.distance(Point(x, y)) for _, x, y, *_ in motion]
        row['path_error_rms_m'] = float(np.sqrt(np.mean(np.square(errors))))
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--world', type=Path, default=WORLD, help='office SDF used for clearance')
    args = parser.parse_args()
    recording = read_recording(args.recording)
    if recording['scenario'] not in ('office', 'office-reference') or not recording['runs']:
        parser.error('expected an office recording with at least one run')
    geometry = collisions(args.world)
    write_csv(args.output, [score(run, geometry) for run in recording['runs']])


if __name__ == '__main__':
    main()
