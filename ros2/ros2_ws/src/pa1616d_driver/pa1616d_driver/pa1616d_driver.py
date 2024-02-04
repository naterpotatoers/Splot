import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix

import serial
import adafruit_gps
import time

# Constants

# Parameters


class PA1616DDriver(Node):

    def __init__(self):
        super().__init__('pa1616d_driver')

        # Initialize GPS publisher
        self._publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)

        # Initialize PA1616D serial
        self._ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
        self._gps = adafruit_gps.GPS(self._ser)

        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self._gps.send_command(b'PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0')

        self.get_logger().info('PA1616D running')
        self._last_update_time = 0
        self._gps_status = NavSatFix().status.STATUS_NO_FIX
        self._fix_timeout = 1

        timer_period = 0.1
        self._timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_frame'
        msg.longitude = float('nan')
        msg.latitude = float('nan')
        msg.altitude = float('nan')
        x_variance_scale = 3.0  # "accuracy of 3 meters"
        y_variance_scale = 3.0  # "accuracy of 3 meters"
        z_variance_scale = 3.0  # "accuracy of 3 meters"
        hdop = float('inf')
        vdop = float('inf')
        if self._gps.update():
            self._last_update_time = time.perf_counter()
            if self._gps.fix_quality_3d == 1:
                self._gps_status = msg.status.STATUS_NO_FIX
                x_variance_scale = float('inf')
                y_variance_scale = float('inf')
                z_variance_scale = float('inf')
            elif self._gps.fix_quality_3d == 2:
                self._gps_status = msg.status.STATUS_FIX
                msg.latitude = self._gps.latitude
                msg.longitude = self._gps.longitude
                msg.altitude = float('nan')
                z_variance_scale = float('inf')
            elif self._gps.fix_quality_3d == 3:
                self._gps_status = msg.status.STATUS_FIX
                msg.latitude = self._gps.latitude
                msg.longitude = self._gps.longitude
                msg.altitude = self._gps.altitude_m
            msg.status.service = msg.status.SERVICE_GPS | msg.status.SERVICE_GLONASS
            if self._gps.hdop is not None:
                hdop = self._gps.hdop
            if self._gps.vdop is not None:
                vdop = self._gps.vdop
            msg.position_covariance[0] = x_variance_scale * hdop
            msg.position_covariance[1] = 0
            msg.position_covariance[2] = 0
            msg.position_covariance[3] = 0
            msg.position_covariance[4] = y_variance_scale * hdop
            msg.position_covariance[5] = 0
            msg.position_covariance[6] = 0
            msg.position_covariance[7] = 0
            msg.position_covariance[8] = z_variance_scale * vdop
            msg.position_covariance_type = msg.COVARIANCE_TYPE_APPROXIMATED
            msg.status.status = self._gps_status
            self._publisher.publish(msg)
        else:
            if time.perf_counter() - self._last_update_time > self._fix_timeout:
                self._gps_status = msg.status.STATUS_NO_FIX
            msg.status.status = self._gps_status
            self._publisher.publish(msg)

    def publish_final_message(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_frame'
        x_variance_scale = 3.0  # "accuracy of 3 meters"
        y_variance_scale = 3.0  # "accuracy of 3 meters"
        z_variance_scale = 3.0  # "accuracy of 3 meters"
        hdop = float('inf')
        vdop = float('inf')
        msg.longitude = float('nan')
        msg.latitude = float('nan')
        msg.altitude = float('nan')
        x_variance_scale = float('inf')
        y_variance_scale = float('inf')
        z_variance_scale = float('inf')
        msg.status.service = msg.status.SERVICE_GPS | msg.status.SERVICE_GLONASS
        msg.position_covariance[0] = x_variance_scale * hdop
        msg.position_covariance[1] = 0
        msg.position_covariance[2] = 0
        msg.position_covariance[3] = 0
        msg.position_covariance[4] = y_variance_scale * hdop
        msg.position_covariance[5] = 0
        msg.position_covariance[6] = 0
        msg.position_covariance[7] = 0
        msg.position_covariance[8] = z_variance_scale * vdop
        msg.position_covariance_type = msg.COVARIANCE_TYPE_APPROXIMATED
        msg.status.status = msg.status.STATUS_NO_FIX
        self._publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    gps_interface = PA1616DDriver()

    rclpy.spin(gps_interface)

    gps_interface.publish_final_message()
    gps_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
