import rclpy
from rclpy.node import Node
import rclpy.time

from triple_axis_servo_controller_interfaces.msg import (
    TripleAxisServoControllerDriveCommand,
    TripleAxisServoControllerFeedback,
    TripleAxisServoControllerMotorCommand,
    TripleAxisServoControllerStatus,
)

import serial
import serialcomms
import sys
import time


def get_connected_board_id(ser, timeout=None):
    command = serialcomms.makeCommand(
        0,
        False,
        False, False, 0, 0,
        False, False, 0, 0,
        False, False, 0, 0,
    )
    board_id = None
    start_time = time.perf_counter()
    while board_id is None:
        now = time.perf_counter()
        if timeout is not None:
            if now - start_time > timeout:
                return None
        r = serialcomms.sendCommand(ser, command)
        if len(r) > 0:
            try:
                board_id = serialcomms.parseResponse(r)[0]
            except:
                return None
    return board_id


class TripleAxisServoControllerDriver(Node):

    def __init__(self):
        super().__init__('triple_axis_servo_controller')
    
        self.serial_timeout = 0.1
        self.drive_command_timeout = 0.1
        self.command_timeout = 0.1

        self.declare_parameter('serial_port', rclpy.Parameter.Type.STRING)
        self._serial_port = self.get_parameter('serial_port').value
        try:
            self._ser = serial.Serial(self._serial_port, 115200, timeout=self.serial_timeout)
        except serial.SerialException as e:
            self.get_logger().info('ERROR: unable to open serial port {}'.format(self._serial_port))
            sys.exit(-1)

        self._tasc_board_id = get_connected_board_id(self._ser, timeout=3)
        if self._tasc_board_id is None:
            self.get_logger().info('ERROR: unable to communicate with board on serial port {}'.format(self._serial_port))
            sys.exit(-1)

        self._status_publisher = self.create_publisher(
            TripleAxisServoControllerStatus,
            '/leg_motor_status_{}'.format(self._tasc_board_id),
            10)
        self._feedback_publisher = self.create_publisher(
            TripleAxisServoControllerFeedback,
            '/leg_motor_feedback_{}'.format(self._tasc_board_id),
            10)
        self._command_subscriber = self.create_subscription(
            TripleAxisServoControllerMotorCommand,
            '/leg_motor_command_{}'.format(self._tasc_board_id),
            self._command_listener_callback,
            10)
        self._drive_command_subscriber = self.create_subscription(
            TripleAxisServoControllerDriveCommand,
            '/leg_motor_drive_command_{}'.format(self._tasc_board_id),
            self._drive_command_listener_callback,
            10)

        self._drives_enable = False
        self._drv1_ready = False
        self._drv2_ready = False
        self._drv3_ready = False
        self._drv1_faulted = False
        self._drv2_faulted = False
        self._drv3_faulted = False
        self._drv1_in1 = 0
        self._drv1_in2 = 0
        self._drv2_in1 = 0
        self._drv2_in2 = 0
        self._drv3_in1 = 0
        self._drv3_in2 = 0

        self._pot1 = 0
        self._current1 = 0
        self._pot2 = 0
        self._current2 = 0
        self._pot3 = 0
        self._current3 = 0
        self._servo_voltage = 0

        self._last_serial_response_time = self.get_clock().now()
        self._last_drive_command_time = self.get_clock().now()
        self._last_command_time = self.get_clock().now()

        serial_timer_period = 0.05
        self._serial_timer = self.create_timer(
            serial_timer_period,
            self._serial_timer_callback)

        self.get_logger().info('triple_axis_servo_controller running on serial port {} with board_id {}'.format(self._serial_port, self._tasc_board_id))

    def __delete__(self):
        try:
            self._ser.close()
            self._ser = None
        except:
            pass

    def _serial_timer_callback(self):
        """Communicates with the board at a regular interval.
        """
        now = self.get_clock().now()
        timediff = (now - self._last_drive_command_time).nanoseconds * 0.000000001
        drive_command_timedout = timediff >= self.drive_command_timeout or timediff < 0
        if drive_command_timedout and timediff <= 0.5:
            self.get_logger().warning(
                'drive command timeout: {} > {}'.format(
                    timediff,
                    self.drive_command_timeout,
                ),
                throttle_duration_sec=0.5,
            )
        timediff = (now - self._last_command_time).nanoseconds * 0.000000001
        command_timedout = timediff >= self.command_timeout or timediff < 0
        if command_timedout and timediff <= 0.5:
            self.get_logger().warning(
                'command timeout: {} > {}'.format(
                    timediff,
                    self.command_timeout,
                ),
                throttle_duration_sec=0.5,
            )

        drives_enable = self._drives_enable and not drive_command_timedout and not command_timedout
        if not drives_enable:
            self._drv1_in1 = 255
            self._drv1_in2 = 255
            self._drv2_in1 = 255
            self._drv2_in2 = 255
            self._drv3_in1 = 255
            self._drv3_in2 = 255

        try:
            command = serialcomms.makeCommand(
                self._tasc_board_id,
                drives_enable,
                True, self._drv1_faulted, self._drv1_in1, self._drv1_in2,
                True, self._drv2_faulted, self._drv2_in1, self._drv2_in2,
                True, self._drv3_faulted, self._drv3_in1, self._drv3_in2,
            )
        except:
            return

        if self._drv1_faulted:
            self.get_logger().warning('DRV1 FAULTED', throttle_duration_sec=0.2)
        if self._drv2_faulted:
            self.get_logger().warning('DRV2 FAULTED', throttle_duration_sec=0.2)
        if self._drv3_faulted:
            self.get_logger().warning('DRV3 FAULTED', throttle_duration_sec=0.2)

        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        _ = self._ser.write(command)
        serial_timestamp = self.get_clock().now()
        r = self._ser.readline()
        if len(r) > 0:
            try:
                (
                    board_id,
                    drv1_enabled, self._drv1_ready, self._drv1_faulted,
                    drv2_enabled, self._drv2_ready, self._drv2_faulted,
                    drv3_enabled, self._drv3_ready, self._drv3_faulted,
                    self._pot1, self._current1,
                    self._pot2, self._current2,
                    self._pot3, self._current3,
                    self._servo_voltage,
                ) = serialcomms.parseResponse(r)
                self._last_serial_response_time = serial_timestamp
            except Exception as e:
                raise e

        status_msg = TripleAxisServoControllerStatus()
        status_msg.stamp = serial_timestamp.to_msg()
        status_msg.board_id = self._tasc_board_id
        timediff_nanos = (serial_timestamp - self._last_serial_response_time).nanoseconds
        serial_timedout = timediff_nanos >= self.serial_timeout * 1000000000
        drives_powered = self._servo_voltage >= 4.5
        status_msg.comms_ok = not serial_timedout
        if not status_msg.comms_ok:
            self._drv1_ready = False
            self._drv2_ready = False
            self._drv3_ready = False
        status_msg.drive1_ready = self._drv1_ready and not self._drv1_faulted and drives_powered
        status_msg.drive2_ready = self._drv2_ready and not self._drv2_faulted and drives_powered
        status_msg.drive3_ready = self._drv3_ready and not self._drv3_faulted and drives_powered

        feedback_msg = TripleAxisServoControllerFeedback()
        feedback_msg.stamp = serial_timestamp.to_msg()
        feedback_msg.board_id = self._tasc_board_id
        feedback_msg.servo1_pot = self._pot1
        feedback_msg.servo1_current = self._current1
        feedback_msg.servo2_pot = self._pot2
        feedback_msg.servo2_current = self._current2
        feedback_msg.servo3_pot = self._pot3
        feedback_msg.servo3_current = self._current3
        feedback_msg.servo_voltage = self._servo_voltage

        self._status_publisher.publish(status_msg)
        self._feedback_publisher.publish(feedback_msg)

    def _drive_command_listener_callback(self, msg):
        if msg.board_id == self._tasc_board_id:
            (
                stamp,
                self._drives_enable,
            ) = (
                msg.stamp,
                msg.drives_enable,
            )
            stamp = self.get_clock().now().to_msg()
            self._last_drive_command_time = rclpy.time.Time.from_msg(stamp)

    def _command_listener_callback(self, msg):
        if msg.board_id == self._tasc_board_id:
            (
                stamp,
                self._drv1_in1,
                self._drv1_in2,
                self._drv2_in1,
                self._drv2_in2,
                self._drv3_in1,
                self._drv3_in2,
            ) = (
                msg.stamp,
                msg.servo1_in1,
                msg.servo1_in2,
                msg.servo2_in1,
                msg.servo2_in2,
                msg.servo3_in1,
                msg.servo3_in2,
            )
            stamp = self.get_clock().now().to_msg()
            self._last_command_time = rclpy.time.Time.from_msg(stamp)

def main(args=None):
    rclpy.init(args=args)

    triple_axis_servo_controller_driver = TripleAxisServoControllerDriver()

    rclpy.spin(triple_axis_servo_controller_driver)

    triple_axis_servo_controller_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
