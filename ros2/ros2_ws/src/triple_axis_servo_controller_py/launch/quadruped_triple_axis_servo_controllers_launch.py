from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
      Node(
         package='triple_axis_servo_controller_py',
         executable='triple_axis_servo_controller',
         name='triple_axis_servo_controller_ttyACM0',
         parameters=[{
            'serial_port': '/dev/ttyACM0',
            'servo1_potentiometer_scale': 6.0,
            'servo2_potentiometer_scale': 6.0,
            'servo3_potentiometer_scale': 6.0,
            'servo1_angle_offset': 0.0,
            'servo2_angle_offset': 0.0,
            'servo3_angle_offset': 0.0,
         }]
      ),
      Node(
         package='triple_axis_servo_controller_py',
         executable='triple_axis_servo_controller',
         name='triple_axis_servo_controller_ttyACM1',
         parameters=[{
            'serial_port': '/dev/ttyACM1',
            'servo1_potentiometer_scale': 6.0,
            'servo2_potentiometer_scale': 6.0,
            'servo3_potentiometer_scale': 6.0,
            'servo1_angle_offset': 0.0,
            'servo2_angle_offset': 0.0,
            'servo3_angle_offset': 0.0,
         }]
      ),
      Node(
         package='triple_axis_servo_controller_py',
         executable='triple_axis_servo_controller',
         name='triple_axis_servo_controller_ttyACM2',
         parameters=[{
            'serial_port': '/dev/ttyACM2',
            'servo1_potentiometer_scale': 6.0,
            'servo2_potentiometer_scale': 6.0,
            'servo3_potentiometer_scale': 6.0,
            'servo1_angle_offset': 0.0,
            'servo2_angle_offset': 0.0,
            'servo3_angle_offset': 0.0,
         }]
      ),
      Node(
         package='triple_axis_servo_controller_py',
         executable='triple_axis_servo_controller',
         name='triple_axis_servo_controller_ttyACM3',
         parameters=[{
            'serial_port': '/dev/ttyACM3',
            'servo1_potentiometer_scale': 6.0,
            'servo2_potentiometer_scale': 6.0,
            'servo3_potentiometer_scale': 6.0,
            'servo1_angle_offset': 0.0,
            'servo2_angle_offset': 0.0,
            'servo3_angle_offset': 0.0,
         }]
      ),
   ])
