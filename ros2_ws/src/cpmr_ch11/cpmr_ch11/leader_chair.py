from enum import Enum
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from cpmr_ch11 import drive

class FSM_STATES(Enum):
    AT_START = 'AT STart',
    PERFORMING_TASK = 'Performing Task',
    TASK_DONE = 'Task Done'

class FSM(Node):

    def __init__(self):
        super().__init__('FSM')
        self.get_logger().info(f'{self.get_name()} created')

        self.declare_parameter('chair_name', "chair_0")
        chair_name = self.get_parameter('chair_name').get_parameter_value().string_value
        self.declare_parameter('heading0_tol', 0.05)
        self.declare_parameter('range_tol', 0.05)
        self.declare_parameter('theta_min', -np.pi / 2)
        self.declare_parameter('theta_max', np.pi / 2)
        self.declare_parameter('speed_min', 0.04)
        self.declare_parameter('speed_max', 0.75)
        self.declare_parameter('theta_gain', 1.0)
        self.declare_parameter('speed_gain', 0.5)

        self.create_subscription(Odometry, f"/{chair_name}/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, f"/{chair_name}/cmd_vel", 1)
        self.create_service(SetBool, f"/{chair_name}/startup", self._startup_callback)
        self._last_x = 0.0
        self._last_y = 0.0
        self._last_id = 0

        # the blackboard
        self._cur_x = 0.0
        self._cur_y = 0.0
        self._cur_theta = 0.0
        self._cur_state = FSM_STATES.AT_START
        self._start_time = self.get_clock().now().nanoseconds * 1e-9
        self._points = [[10, 0], [10, 10], [15, 10], [15, 0]]
        self._point = 0
        self._run = False

    def _startup_callback(self, request, resp):
        self.get_logger().info(f'Got a request {request}')
        if request.data:
            self.get_logger().info(f'fsm starting')
            self._run = True
            resp.success = True
            resp.message = "Architecture running"
        else:
            self.get_logger().info(f'fsm suspended')
            self._publisher.publish(Twist())
            self._run = False
            resp.success = True
            resp.message = "Architecture suspended"
        return resp
           
    def _drive_to_goal(self, goal_x, goal_y,
                       range_tol = 0.15):
        heading0_tol = self.get_parameter("heading0_tol").get_parameter_value().double_value
        range_tol = self.get_parameter("range_tol").get_parameter_value().double_value
        theta_range = (self.get_parameter("theta_min").get_parameter_value().double_value, self.get_parameter("theta_max").get_parameter_value().double_value)
        speed_range = (self.get_parameter("speed_min").get_parameter_value().double_value, self.get_parameter("speed_max").get_parameter_value().double_value)
        theta_gain = self.get_parameter("theta_gain").get_parameter_value().double_value
        speed_gain = self.get_parameter("speed_gain").get_parameter_value().double_value
        _, reached = drive.drive_to_goal(self._cur_x, self._cur_y, self._cur_theta, self._publisher, goal_x, goal_y, heading0_tol, range_tol,theta_range, speed_range, theta_gain, speed_gain)
        return reached

    def _do_state_at_start(self):
        self.get_logger().info(f'in start state')
        if self._run:
            self.get_logger().info(f'Starting...')
            self._cur_state = FSM_STATES.PERFORMING_TASK

    def _do_state_performing_task(self):
        if not self._run:
            return
        self.get_logger().info(f'heading to task {self._point}')
        if self._drive_to_goal(self._points[self._point][0], self._points[self._point][1]):
            self._point = self._point + 1
            if self._point >= len(self._points):
                self._point = 0

    def _state_machine(self):
        if self._cur_state == FSM_STATES.AT_START:
            self._do_state_at_start()
        elif self._cur_state == FSM_STATES.PERFORMING_TASK:
            self._do_state_performing_task()
        else:
            self.get_logger().info(f'bad state {self._cur_state}')

    def _listener_callback(self, msg):
        pose = msg.pose.pose

        d2 = (pose.position.x - self._last_x) * (pose.position.x - self._last_x) + (pose.position.y - self._last_y) * (pose.position.y - self._last_y)

        roll, pitch, yaw = drive.euler_from_quaternion(pose.orientation)
        self._cur_x = pose.position.x
        self._cur_y = pose.position.y
        self._cur_theta = drive.short_angle(yaw)
        self._state_machine()



def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    try:
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

