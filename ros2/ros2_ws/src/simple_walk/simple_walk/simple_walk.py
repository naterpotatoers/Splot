import rclpy
from rclpy.node import Node

from mission_control_interfaces.msg import MissionControlToRobot
from servo_interfaces.msg import QuadrupedLegPositions

import math
import time


def get_step_position(t, stride_size, raised_height, ratio_raised):
    t = t % (1.0 + ratio_raised)
    if t <= 1.0:
        return (
            stride_size * (0.5 - t),
            0.0,
        )
    else:
        t -= 1
        return (
            stride_size * (-0.5 + t / ratio_raised),
            raised_height * math.sin(math.pi / ratio_raised * t),
        )


def get_shift_amount(t, ratio_raised, shift_amount):
    t = t % (1.0 + ratio_raised)
    if t <= 1.0:
        return shift_amount * (2.0 * t - 1)**2
    else:
        return shift_amount


def wrap_direction_error(target_radians, current_radians):
    pi = math.pi
    pi2 = 2.0 * math.pi
    return ((target_radians - current_radians) % pi2 + pi) % pi2 - pi


class QuadSimpleWalk(Node):

    def __init__(self):
        super().__init__('quadruped_simple_walk')

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
        self.last_timer_time = time.perf_counter()

        self.target_direction = 0.0
        self.direction = 0.0
        self.max_speed = 0.2
        self.speed = 0.0

        self.t = 0.0

        self.get_logger().info('simple walk running')

    def mission_control_callback(self, msg):
        self.speed = math.sqrt(msg.speed_forward**2 + msg.speed_right**2)
        if (self.speed > 0.01):
            self.target_direction = math.atan2(
                -msg.speed_right,
                msg.speed_forward)
        self.speed = max(0.0, min(1.0, self.speed))
    
    def timer_callback(self):
        now = time.perf_counter()

        # Find out how much to change direction
        direction_error = self.target_direction - self.direction
        # Wrap direction error to take the shortest path
        direction_error = (direction_error % () + 180) % 360 - 180
        # Slowly change input direction
        dir_speed = (now - self.last_timer_time) * math.pi * 2.0 * 0.5
        self.direction += max(-dir_speed, min(dir_speed, direction_error))

        dir_x, dir_y = (
            math.cos(self.direction + math.pi * 0.5),
            math.sin(self.direction + math.pi * 0.5),
        )

        t = self.t
        z = -10.0  # inches

        stride_size = 4.0
        raised_height = 2.0
        ratio_raised = 0.2
        stance_width_x = 6
        stance_width_y = 8.5
        shift_amplitude = 0.8

        msg = QuadrupedLegPositions()

        shift_x = 0.0
        shift_y = 0.0

        leg_t = t
        (step_xy, step_z) = get_step_position(leg_t, stride_size, raised_height, ratio_raised)
        msg.leg_front_right.x = stance_width_x + dir_x * step_xy
        msg.leg_front_right.y = stance_width_y + dir_y * step_xy
        msg.leg_front_right.z = z + step_z
        shift_amount = get_shift_amount(leg_t, ratio_raised, shift_amplitude)
        shift_x += shift_amount * 2
        shift_y += shift_amount * 3

        leg_t = t - 0.5 * (1 + ratio_raised)
        (step_xy, step_z) = get_step_position(leg_t, stride_size, raised_height, ratio_raised)
        msg.leg_front_left.x = -stance_width_x + dir_x * step_xy
        msg.leg_front_left.y = stance_width_y + dir_y * step_xy
        msg.leg_front_left.z = z + step_z
        shift_amount = get_shift_amount(leg_t, ratio_raised, shift_amplitude)
        shift_x += shift_amount * -2
        shift_y += shift_amount * 3

        leg_t = t - 0.25 * (1 + ratio_raised)
        (step_xy, step_z) = get_step_position(leg_t, stride_size, raised_height, ratio_raised)
        msg.leg_rear_left.x = -stance_width_x + dir_x * step_xy
        msg.leg_rear_left.y = -stance_width_y + dir_y * step_xy
        msg.leg_rear_left.z = z + step_z
        shift_amount = get_shift_amount(leg_t, ratio_raised, shift_amplitude)
        shift_x += shift_amount * -2
        shift_y += shift_amount * -3

        leg_t = t - 0.75 * (1 + ratio_raised)
        (step_xy, step_z) = get_step_position(leg_t, stride_size, raised_height, ratio_raised)
        msg.leg_rear_right.x = stance_width_x + dir_x * step_xy
        msg.leg_rear_right.y = -stance_width_y + dir_y * step_xy
        msg.leg_rear_right.z = z + step_z
        shift_amount = get_shift_amount(leg_t, ratio_raised, shift_amplitude)
        shift_x += shift_amount * 2
        shift_y += shift_amount * -3
        
        msg.leg_front_right.x += shift_x
        msg.leg_front_right.y += shift_y
        msg.leg_front_left.x += shift_x
        msg.leg_front_left.y += shift_y
        msg.leg_rear_left.x += shift_x
        msg.leg_rear_left.y += shift_y
        msg.leg_rear_right.x += shift_x
        msg.leg_rear_right.y += shift_y

        self.publisher_.publish(msg)

        # Advance "time" parameter according to desired speed
        self.t += (now - self.last_timer_time) * self.speed * self.max_speed

        # Decay speed over time when not being updated by mission control
        # to mitigate connection-loss issues
        halt_time = 0.5  # seconds it takes for the robot to halt when not receiving commands
        self.speed = max(0.0, self.speed - self.max_speed * (now - self.last_timer_time) / halt_time)

        self.last_timer_time = now


def main(args=None):
    rclpy.init(args=args)

    quad_simple_walk = QuadSimpleWalk()

    rclpy.spin(quad_simple_walk)

    quad_simple_walk.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
