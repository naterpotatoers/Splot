# Splot

A quadruped robot built for AMD's Pervasive AI Contest 2024.


## Installing
* Compute platform: Raspberry Pi 3 Model B+
* Operating system: Ubuntu Server 22.04.3 LTS (64-bit)
* Python: 3.10.12
* ROS: ROS2 Iron Irwini (`apt install ros-dev-tools ros-iron-ros-base`)

Run `git clone --recurse-submodules [git repo url]` to clone this repository with submodules (i.e. `joystick_drivers`).

Follow the installation instructions on the [joystick_drivers wiki](https://wiki.ros.org/joystick_drivers).

Install python dependencies using `python3 -m pip install -r Splot/requirements.txt`. Note that legacy code using the PCA9685 boards also requires `adafruit-circuitpython-servokit`.


## Building
* `cd` into `Splot/ros2/ros2_ws` and run `colcon build --symlink-install` to build the Splot ROS2 packages.
* `cd` into `Splot/ros2/ros2_joy_ws` and run `colcon build --symlink-install` again to build the `joystick_drivers` package.


## Running
* Source the ROS workspaces using `source Splot/ros2/ros2_ws/install/local_setup.bash` and `source Splot/ros2/ros2_joy_ws/install/local_setup.bash` (these need to be run every time you login/reboot)
* `cd` into `Splot/ros2/launch` and run `ros2 launch mission_control_joy.yaml` to run the gamepad input demo, or choose a different launch file


## Dependencies
* [joystick_drivers](https://wiki.ros.org/joystick_drivers) - ROS2 nodes for receiving joystick/gamepad input

See `requirements.txt` for Python package dependencies.
