from geometry_msgs.msg import Twist
import numpy as np

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def short_angle(angle):
    if angle > np.pi:
        angle = angle - 2 * np.pi
    if angle < -np.pi:
        angle = angle + 2 * np.pi
    assert abs(angle) <= np.pi
    return angle

def compute_speed(diff, max_speed, min_speed, gain):
    speed = abs(diff) * gain
    speed = min(max_speed, max(min_speed, speed))
    return np.copysign(speed, diff)

def drive_to_goal(cur_x, cur_y, cur_theta, publisher, goal_x, goal_y,
                   heading0_tol = 0.05,
                   range_tol = 0.05, theta_range=(-np.pi/2, np.pi/2), speed_range=(0.04, 0.75), theta_gain=1.0, speed_gain=0.5):
    """Return True iff we are at the goal, otherwise drive there"""

    twist = Twist()


    x_diff = goal_x - cur_x
    y_diff = goal_y - cur_y
    dist = np.sqrt(x_diff * x_diff + y_diff * y_diff)
    if dist > range_tol:
        # turn to the goal
        heading = np.arctan2(y_diff, x_diff)
        diff = short_angle(heading - cur_theta)
        if (abs(diff) > heading0_tol):
            twist.angular.z = compute_speed(diff, theta_range[1], theta_range[0], theta_gain)
            publisher.publish(twist)
            return twist, False

        twist.linear.x = compute_speed(dist, speed_range[1], speed_range[0], speed_gain)
        publisher.publish(twist)
        return twist, False

    publisher.publish(twist)
    return twist, True


