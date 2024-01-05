import rclpy
from rclpy.node import Node

from servo_interfaces.msg import LegJointAngles, QuadrupedJointAngles

from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/
import time

# Constants
nbPCAServo = 12

# Parameters
MIN_IMP = [500] * nbPCAServo
MAX_IMP = [2500] * nbPCAServo
ACTUATION_RANGE = [274, 276.5, 273.25, 276.5, 278, 275, 274.75, 277, 273, 275.75, 273.75, 277.5]
SERVO_ZERO = [133, 138.5, 140, 136, 133.5, 144, 138.5, 139, 142, 143, 142, 131]


RAD2DEG = 180.0 / 3.141592


class PCA9685Driver(Node):

    def __init__(self):
        super().__init__('pca9685_driver')

        # Initialize servo joint angle subscriber
        self.subscription = self.create_subscription(
                QuadrupedJointAngles,
                'joint_angles',
                self.listener_callback,
                10)

        # Initialize PCA9685 I2C
        self.pca = ServoKit(channels=16)
        for i in range(nbPCAServo):
            self.pca.servo[i].set_pulse_width_range(MIN_IMP[i], MAX_IMP[i])
            self.pca.servo[i].actuation_range = ACTUATION_RANGE[i]

        self.get_logger().info('PCA9685 running')
        self._last_angle_log_time = 0

    def listener_callback(self, msg):
        now = time.perf_counter()
        # if now - self._last_angle_log_time >= 1:
        #     self._last_angle_log_time = now
        #     # Log received angles
        #     self.get_logger().info('Servo angles received:')
        #     self.get_logger().info('\tFront right: < %.2f, %.2f, %.2f >' % (
        #         msg.leg_front_right.joint1_angle,
        #         msg.leg_front_right.joint2_angle,
        #         msg.leg_front_right.joint3_angle,
        #     ))
        #     self.get_logger().info('\tFront left: < %.2f, %.2f, %.2f >' % (
        #         msg.leg_front_left.joint1_angle,
        #         msg.leg_front_left.joint2_angle,
        #         msg.leg_front_left.joint3_angle,
        #     ))
        #     self.get_logger().info('\tRear left: < %.2f, %.2f, %.2f >' % (
        #         msg.leg_rear_left.joint1_angle,
        #         msg.leg_rear_left.joint2_angle,
        #         msg.leg_rear_left.joint3_angle,
        #     ))
        #     self.get_logger().info('\tRear right: < %.2f, %.2f, %.2f >' % (
        #         msg.leg_rear_right.joint1_angle,
        #         msg.leg_rear_right.joint2_angle,
        #         msg.leg_rear_right.joint3_angle,
        #     ))

        positions_reachable = True
        positions_reachable = positions_reachable and msg.leg_front_right.position_reachable
        positions_reachable = positions_reachable and msg.leg_front_left.position_reachable
        positions_reachable = positions_reachable and msg.leg_rear_left.position_reachable
        positions_reachable = positions_reachable and msg.leg_rear_right.position_reachable

        if not positions_reachable:
            self.get_logger().info('UNREACHABLE POSITION')

        angles_valid = True
        if msg.leg_front_right.position_reachable:
            angles_valid = angles_valid and self.get_servo_angle_valid(0, -msg.leg_front_right.joint1_angle)
            angles_valid = angles_valid and self.get_servo_angle_valid(1, msg.leg_front_right.joint2_angle)
            angles_valid = angles_valid and self.get_servo_angle_valid(2, -msg.leg_front_right.joint3_angle)
        if msg.leg_front_left.position_reachable:
            angles_valid = angles_valid and self.get_servo_angle_valid(3, msg.leg_front_left.joint1_angle)
            angles_valid = angles_valid and self.get_servo_angle_valid(4, -msg.leg_front_left.joint2_angle)
            angles_valid = angles_valid and self.get_servo_angle_valid(5, msg.leg_front_left.joint3_angle)
        if msg.leg_rear_left.position_reachable:
            angles_valid = angles_valid and self.get_servo_angle_valid(6, -msg.leg_rear_left.joint1_angle)
            angles_valid = angles_valid and self.get_servo_angle_valid(7, -msg.leg_rear_left.joint2_angle)
            angles_valid = angles_valid and self.get_servo_angle_valid(8, msg.leg_rear_left.joint3_angle)
        if msg.leg_rear_right.position_reachable:
            angles_valid = angles_valid and self.get_servo_angle_valid(9, msg.leg_rear_right.joint1_angle)
            angles_valid = angles_valid and self.get_servo_angle_valid(10, msg.leg_rear_right.joint2_angle)
            angles_valid = angles_valid and self.get_servo_angle_valid(11, -msg.leg_rear_right.joint3_angle)

        if not angles_valid:
            self.get_logger().info('INVALID ANGLE')

        if positions_reachable and angles_valid:
            # Send angles to PCA9685
            self.set_servo_angle(0, -msg.leg_front_right.joint1_angle)
            self.set_servo_angle(1, msg.leg_front_right.joint2_angle)
            self.set_servo_angle(2, -msg.leg_front_right.joint3_angle)
            self.set_servo_angle(3, msg.leg_front_left.joint1_angle)
            self.set_servo_angle(4, -msg.leg_front_left.joint2_angle)
            self.set_servo_angle(5, msg.leg_front_left.joint3_angle)
            self.set_servo_angle(6, -msg.leg_rear_left.joint1_angle)
            self.set_servo_angle(7, -msg.leg_rear_left.joint2_angle)
            self.set_servo_angle(8, msg.leg_rear_left.joint3_angle)
            self.set_servo_angle(9, msg.leg_rear_right.joint1_angle)
            self.set_servo_angle(10, msg.leg_rear_right.joint2_angle)
            self.set_servo_angle(11, -msg.leg_rear_right.joint3_angle)

    def get_servo_angle_valid(self, servo_i, angle_rad):
        angle_deg = SERVO_ZERO[servo_i] + RAD2DEG * angle_rad
        return 0 <= angle_deg and angle_deg <= 270

    def set_servo_angle(self, servo_i, angle_rad):
        self.pca.servo[servo_i].angle = max(0, min(270, SERVO_ZERO[servo_i] + RAD2DEG * angle_rad))
        # self.get_logger().info('servo[%d].angle = %.2f' % (servo_i, SERVO_ZERO[servo_i] + RAD2DEG * angle_rad))


def main(args=None):
    rclpy.init(args=args)

    servo_driver = PCA9685Driver()

    rclpy.spin(servo_driver)

    servo_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
