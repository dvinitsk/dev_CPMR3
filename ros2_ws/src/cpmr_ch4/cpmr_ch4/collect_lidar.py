import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
import cv2
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


class CollectLidar(Node):
    _WIDTH = 513
    _HEIGHT = 513
    _M_PER_PIXEL = 0.05

    def __init__(self):
        super().__init__('collect_lidar')
        self.get_logger().info(f'{self.get_name()} created')

        self._map = np.zeros((CollectLidar._HEIGHT, CollectLidar._WIDTH), dtype=np.uint8)

        self.cur_t = None
        self.create_subscription(Odometry, "/odom", self._odom_callback, 1)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 1)


    def _scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        self.get_logger().info(f"lidar ({angle_min},{angle_max},{angle_increment},{len(ranges)})")
        #understand what ranges outputs, and then fill up self._map accordingly
       
        for i in range(len(ranges)):
            #polar to cartesian calculations
         
            angle = angle_min + i * angle_increment
            r = ranges[i]
           
            #x and y distances to obstacle from robots perspective
            robot_x = r * math.cos(angle)
            robot_y = r * math.sin(angle)
           
            #changing robots coordinate frame to world's coordinate frame (expressed as some float values)
            #where world_x and world_y represent the obstacle's position in meters
            if self.cur_t is None:
                continue
            world_x = robot_x * math.cos(self.cur_t) - robot_y * math.sin(self.cur_t)
            world_y = robot_x * math.sin(self.cur_t) + robot_y * math.cos(self.cur_t)
           
            #representing map's coordinates relative to the robot's position
            pixel_x = int(((world_x + self.cur_x) / self._M_PER_PIXEL) + self._WIDTH / 2)
            pixel_y = int(((world_y + self.cur_y) / self._M_PER_PIXEL) + self._WIDTH / 2)

            self._map[pixel_x, pixel_y] = 255

        cv2.imshow('map',self._map)
        cv2.waitKey(10)

    def _odom_callback(self, msg):
        pose = msg.pose.pose

        self.cur_x = pose.position.x
        self.cur_y = pose.position.y

        o = pose.orientation

        roll, pitchc, self.cur_t = euler_from_quaternion(o)

        self.get_logger().info(f"at ({self.cur_x},{self.cur_y},{self.cur_t})")


def main(args=None):
    rclpy.init(args=args)
    node = CollectLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
