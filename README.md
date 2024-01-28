# Splot

A quadruped robot built for AMD's Pervasive AI Contest 2024.


## Installing
* Compute platform: Raspberry Pi 3 Model B+
* Operating system: Ubuntu Server 22.04.3 LTS (64-bit)
* Python: 3.10.12
* ROS: ROS2 Iron Irwini (follow the [installation instructions](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html) and choose `ros-iron-ros-base`)

Run `git clone --recurse-submodules [git repo url]` to clone this repository with submodules (i.e. `joystick_drivers`).

For gamepad support, follow the installation instructions on the [joystick_drivers wiki](https://wiki.ros.org/joystick_drivers). Additionally, `apt install ros-iron-diagnostic-updater libx11-dev libxext-dev`.

Install python dependencies using `python3 -m pip install -r Splot/requirements.txt`. Note that legacy code using the PCA9685 boards also requires `adafruit-circuitpython-servokit`.


## Building
* `cd` into `Splot/ros2/ros2_ws` and run `colcon build --symlink-install` to build the Splot ROS2 packages.
* `cd` into `Splot/ros2/ros2_joy_ws` and run `colcon build --symlink-install` again to build the `joystick_drivers` package.


## Running
* Source the ROS workspaces using `source Splot/ros2/ros2_ws/install/local_setup.bash` and `source Splot/ros2/ros2_joy_ws/install/local_setup.bash` (these need to be run every time you login/reboot)
* `cd` into `Splot/ros2/launch` and run `ros2 launch mission_control_joy.yaml` to run the gamepad input demo, or choose a different launch file


## Dependencies
* [joystick_drivers](https://wiki.ros.org/joystick_drivers) - ROS2 nodes for receiving joystick/gamepad input
* [robot_localization](https://github.com/cra-ros-pkg/robot_localization) - ROS2 sensor fusion package implementing extended kalman filtering and unscented kalman filtering
* [adafruit-circuitpython-bno08x](https://docs.circuitpython.org/projects/bno08x/en/latest/) - Python package for interfacing with BNO08x family 9-axis absolute orientation IMUs
* [adafruit gps](https://docs.circuitpython.org/projects/gps/en/latest/) - Python package for interfacing with Adafruit's PA1616D "Ultimate GPS Breakout"
* [pyserial](https://pypi.org/project/pyserial/) - Python package for serial communication


## Other Notes
By default, Ubuntu does not allow access to the Raspberry Pi's built-in RX/TX serial pins, which we use to communicate with the GPS. We followed [these instructions](https://raspberrypi.stackexchange.com/a/116860) to enable hardware serial access.

See `requirements.txt` for Python package dependencies.
