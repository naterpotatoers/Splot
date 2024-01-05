import rclpy
from rclpy.node import Node

from servo_interfaces.msg import QuadrupedLegPositions

import math
import time



class QuadMotionTest(Node):

    def __init__(self):
        super().__init__('quadruped_motion_test')

        self.publisher_ = self.create_publisher(
            QuadrupedLegPositions,
            'leg_positions',
            10)
        update_period = 1 / 100  # seconds
        self.timer = self.create_timer(update_period, self.timer_callback)

        self.get_logger().info('motion test running')
    
    def timer_callback(self):
        now = time.perf_counter()

        theta = (now * 2 * math.pi * 0.8) % (2 * math.pi)  # radians
        amp = 1.0  # inches
        z = -7.5 + 1.5 * math.cos(now * 2 * math.pi / 3)  # inches

        msg = QuadrupedLegPositions()

        msg.leg_front_right.x = 7 + amp * math.cos(theta)
        msg.leg_front_right.y = 8 + amp * math.sin(theta)
        msg.leg_front_right.z = z

        msg.leg_front_left.x = -7 + amp * math.cos(theta)
        msg.leg_front_left.y = 8 + amp * math.sin(theta)
        msg.leg_front_left.z = z

        msg.leg_rear_left.x = -7 + amp * math.cos(theta)
        msg.leg_rear_left.y = -8 + amp * math.sin(theta)
        msg.leg_rear_left.z = z

        msg.leg_rear_right.x = 7 + amp * math.cos(theta)
        msg.leg_rear_right.y = -8 + amp * math.sin(theta)
        msg.leg_rear_right.z = z

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    quad_motion_test = QuadMotionTest()

    rclpy.spin(quad_motion_test)

    quad_motion_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
