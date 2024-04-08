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
         }]
      ),
      Node(
         package='triple_axis_servo_controller_py',
         executable='triple_axis_servo_controller',
         name='triple_axis_servo_controller_ttyACM1',
         parameters=[{
            'serial_port': '/dev/ttyACM1',
         }]
      ),
      Node(
         package='triple_axis_servo_controller_py',
         executable='triple_axis_servo_controller',
         name='triple_axis_servo_controller_ttyACM2',
         parameters=[{
            'serial_port': '/dev/ttyACM2',
         }]
      ),
      Node(
         package='triple_axis_servo_controller_py',
         executable='triple_axis_servo_controller',
         name='triple_axis_servo_controller_ttyACM3',
         parameters=[{
            'serial_port': '/dev/ttyACM3',
         }]
      ),
   ])
