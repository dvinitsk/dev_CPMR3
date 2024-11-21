import math
import csv
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw.
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

class MoveToGoal(Node):
    def __init__(self, waypoints_file='/home/dvinitsk/dev_CPMR3/ros2_ws/waypoints_gazebo.csv'):
        super().__init__('drive_to_goal')
        self.get_logger().info(f'{self.get_name()} created')

        # Load waypoints from file
        self.waypoints = self.load_waypoints(waypoints_file)
        if not self.waypoints:
            self.get_logger().error("No waypoints loaded. Shutting down node.")
            rclpy.shutdown()
            return
        
        self.current_waypoint_index = 0
        self._goal_x, self._goal_y = self.waypoints[0]
        
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
        self.get_logger().info(f"Initial goal: ({self._goal_x}, {self._goal_y})")

        self._subscriber = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        
        self.moving_to_waypoint = True

    def load_waypoints(self, waypoints_file):
        """Load waypoints from a CSV file."""
        waypoints = []
        try:
            with open(waypoints_file, 'r') as f:
                csv_reader = csv.reader(f)
                next(csv_reader)  # Skip the header
                for row in csv_reader:
                    if len(row) >= 2:
                        waypoints.append((float(row[0]), float(row[1])))
            self.get_logger().info(f"Loaded waypoints from {waypoints_file}")
        except Exception as e:
            self.get_logger().error(f"Error reading waypoints: {e}")
        return waypoints

    def move_to_next_waypoint(self):
        """Update goal to next waypoint if available"""
        self.current_waypoint_index += 1
        if self.current_waypoint_index < len(self.waypoints):
            self._goal_x, self._goal_y = self.waypoints[self.current_waypoint_index]
            self.moving_to_waypoint = True
            self.get_logger().info(f"Moving to waypoint {self.current_waypoint_index}: ({self._goal_x:.2f}, {self._goal_y:.2f})")
            return True
        else:
            self.get_logger().info("Reached final waypoint!")
            return False

    def _listener_callback(self, msg, vel_gain=5.0, max_vel=0.2, max_pos_err=0.05):
        if not self.moving_to_waypoint:
            return

        # Get the robot's current position and orientation
        pose = msg.pose.pose
        cur_x = pose.position.x
        cur_y = pose.position.y
        o = pose.orientation
        roll, pitch, yaw = euler_from_quaternion(o)
        cur_t = yaw

        # Calculate the difference and distance to the goal
        x_diff = self._goal_x - cur_x
        y_diff = self._goal_y - cur_y
        dist = math.sqrt(x_diff**2 + y_diff**2)

        # Debugging information
        self.get_logger().info(
            f"Current pose: ({cur_x:.2f}, {cur_y:.2f}, {cur_t:.2f}) | "
            f"Goal: ({self._goal_x:.2f}, {self._goal_y:.2f}) | "
            f"Distance to goal: {dist:.2f}"
        )

        twist = Twist()

        if dist > max_pos_err:
            # Calculate linear velocity based on distance and orientation
            heading = math.atan2(y_diff, x_diff)
            angle_diff = heading - cur_t
            twist.angular.z = max(min(angle_diff * vel_gain, max_vel), -max_vel)
            twist.linear.x = max(min(dist * vel_gain, max_vel), -max_vel)
            self.get_logger().info(
                f"Driving: Distance {dist:.2f}, Heading difference {angle_diff:.2f}, "
                f"Linear velocity {twist.linear.x:.2f}, Angular velocity {twist.angular.z:.2f}"
            )
        else:
            # Goal reached, move to the next waypoint
            self.moving_to_waypoint = False
            if not self.move_to_next_waypoint():
                self.get_logger().info("Navigation complete!")
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        # Publish the velocity command
        self._publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

