import rclpy
from rclpy.node import Node

from mission_control_interfaces.msg import MissionControlToRobot
from servo_interfaces.msg import QuadrupedLegPositions

import math
import time



class QuadMCMotionTest(Node):

    def __init__(self):
        super().__init__('mission_control_motion_test')

        self.subscription_ = self.create_subscription(
            MissionControlToRobot,
            'mission_control_to_robot',
            self.mission_control_callback,
            10)
        self.publisher_ = self.create_publisher(
            QuadrupedLegPositions,
            'leg_positions',
            10)
        
        update_period = 1 / 100  # seconds
        self.timer = self.create_timer(update_period, self.timer_callback)
        self.last_update_time = time.perf_counter()
        
        self.x = 0.0
        self.y = 0.0
        self.z = 8.5
        self.turn_theta = 0.0
        self.tilt_theta = 0.0
        
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 8.5
        self.target_turn_theta = 0.0
        self.target_tilt_theta = 0.0

        self.radius = 2.0  # inches
        self.max_turn_degrees = 15
        self.max_tilt_degrees = 30

        self.get_logger().info('mission control motion test running')
    
    def mission_control_callback(self, mc_msg):
        self.target_x = mc_msg.speed_right
        self.target_y = mc_msg.speed_forward
        self.target_z = 8.5 + 2.5 * mc_msg.height  # inches
        self.target_turn_theta = mc_msg.speed_angular * self.max_turn_degrees * math.pi / 180
        self.target_tilt_theta = mc_msg.tilt * self.max_tilt_degrees * math.pi / 180
        # self.get_logger().info('target < x, y, z, turn, tilt > = < {:5.2f}, {:5.2f}, {:5.2f}, {:5.2f}, {:5.2f} >'.format(
        #     self.target_x, self.target_y, self.target_z, self.target_turn_theta, self.target_tilt_theta))

    def timer_callback(self):
        now = time.perf_counter()
        time_elapsed = now - self.last_update_time

        # EMA of controller input
        self.x += (self.target_x - self.x) * min(1.0, time_elapsed / 0.1)
        self.y += (self.target_y - self.y) * min(1.0, time_elapsed / 0.1)
        self.z += (self.target_z - self.z) * min(1.0, time_elapsed / 0.1)
        self.turn_theta += (self.target_turn_theta - self.turn_theta) * min(1.0, time_elapsed / 0.1)
        self.tilt_theta += (self.target_tilt_theta - self.tilt_theta) * min(1.0, time_elapsed / 0.2)

        # self.get_logger().info('< x, y, z, turn, tilt > = < {:5.2f}, {:5.2f}, {:5.2f}, {:5.2f}, {:5.2f} >'.format(
        #     self.x, self.y, self.z, self.turn_theta, self.tilt_theta))

        amp = self.radius

        msg = QuadrupedLegPositions()

        x = 7 + amp * -self.x
        y = 7 + amp * -self.y
        msg.leg_front_right.x = x * math.cos(self.turn_theta) - y * math.sin(self.turn_theta)
        msg.leg_front_right.y = x * math.sin(self.turn_theta) + y * math.cos(self.turn_theta)
        msg.leg_front_right.z = -self.z
        tilt_x = msg.leg_front_right.y
        tilt_y = msg.leg_front_right.z
        msg.leg_front_right.y = tilt_x * math.cos(-self.tilt_theta) - tilt_y * math.sin(-self.tilt_theta)
        msg.leg_front_right.z = tilt_x * math.sin(-self.tilt_theta) + tilt_y * math.cos(-self.tilt_theta)

        x = -7 + amp * -self.x
        y = 7 + amp * -self.y
        msg.leg_front_left.x = x * math.cos(self.turn_theta) - y * math.sin(self.turn_theta)
        msg.leg_front_left.y = x * math.sin(self.turn_theta) + y * math.cos(self.turn_theta)
        msg.leg_front_left.z = -self.z
        tilt_x = msg.leg_front_left.y
        tilt_y = msg.leg_front_left.z
        msg.leg_front_left.y = tilt_x * math.cos(-self.tilt_theta) - tilt_y * math.sin(-self.tilt_theta)
        msg.leg_front_left.z = tilt_x * math.sin(-self.tilt_theta) + tilt_y * math.cos(-self.tilt_theta)

        x = -7 + amp * -self.x
        y = -7 + amp * -self.y
        msg.leg_rear_left.x = x * math.cos(self.turn_theta) - y * math.sin(self.turn_theta)
        msg.leg_rear_left.y = x * math.sin(self.turn_theta) + y * math.cos(self.turn_theta)
        msg.leg_rear_left.z = -self.z
        tilt_x = msg.leg_rear_left.y
        tilt_y = msg.leg_rear_left.z
        msg.leg_rear_left.y = tilt_x * math.cos(-self.tilt_theta) - tilt_y * math.sin(-self.tilt_theta)
        msg.leg_rear_left.z = tilt_x * math.sin(-self.tilt_theta) + tilt_y * math.cos(-self.tilt_theta)

        x = 7 + amp * -self.x
        y = -7 + amp * -self.y
        msg.leg_rear_right.x = x * math.cos(self.turn_theta) - y * math.sin(self.turn_theta)
        msg.leg_rear_right.y = x * math.sin(self.turn_theta) + y * math.cos(self.turn_theta)
        msg.leg_rear_right.z = -self.z
        tilt_x = msg.leg_rear_right.y
        tilt_y = msg.leg_rear_right.z
        msg.leg_rear_right.y = tilt_x * math.cos(-self.tilt_theta) - tilt_y * math.sin(-self.tilt_theta)
        msg.leg_rear_right.z = tilt_x * math.sin(-self.tilt_theta) + tilt_y * math.cos(-self.tilt_theta)

        self.publisher_.publish(msg)

        self.last_update_time = now


def main(args=None):
    rclpy.init(args=args)

    quad_mc_motion_test = QuadMCMotionTest()

    rclpy.spin(quad_mc_motion_test)

    quad_mc_motion_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
