from math import atan2, cos, hypot, pi, sin

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