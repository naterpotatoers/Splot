# Splot

A quadruped robot built for AMD's Pervasive AI Contest 2024.


## Installing
* Compute platform: Raspberry Pi 4 8Gb Model
* Operating system: Ubuntu Server 22.04.3 LTS (64-bit)
* Python: 3.10.12
* ROS: ROS2 Iron Irwini (follow the [installation instructions](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html) and choose `ros-iron-ros-base`)

Run `git clone --recurse-submodules [git repo url]` to clone this repository with submodules (i.e. `joystick_drivers`).

For gamepad support, follow the installation instructions on the [joystick_drivers wiki](https://wiki.ros.org/joystick_drivers). Additionally, `apt install ros-iron-diagnostic-updater libx11-dev libxext-dev`.

Install python dependencies using `python3 -m pip install -r Splot/requirements.txt`. Note that legacy code using the PCA9685 boards also requires `adafruit-circuitpython-servokit`.


## Building
* Run `sudo rosdep init` to initialize rosdep if you have not done so already.
* Run `rosdep update` to update rosdep index
* For each of `Splot/ros2/ros2_joy_ws`, `Splot/ros2/ros2_robot_localization_ws`, and `Splot/ros2/ros2_ws`:
* `cd` into the workspace directory (i.e. `Splot/ros2/ros2_robot_localization_ws`)
* Run `rosdep install --from-paths src -y --ignore-src` to install dependencies
* Run `colcon build --symlink-install` to build the packages


## Running
* Source the ROS workspaces using `source Splot/ros2/ros2_ws/install/local_setup.bash`, `source Splot/ros2/ros2_robot_localization_ws/install/local_setup.bash`, and `source Splot/ros2/ros2_joy_ws/install/local_setup.bash` (these need to be run every time you login/reboot)
* `cd` into `Splot/ros2/launch` and run `ros2 launch mission_control_joy.yaml` to run the gamepad input demo, or choose a different launch file


## Dependencies
* [joystick_drivers](https://wiki.ros.org/joystick_drivers) - ROS2 nodes for receiving joystick/gamepad input
* [robot_localization](https://github.com/cra-ros-pkg/robot_localization) - ROS2 sensor fusion package implementing extended kalman filtering and unscented kalman filtering
* [adafruit-circuitpython-bno08x](https://docs.circuitpython.org/projects/bno08x/en/latest/) - Python package for interfacing with BNO08x family 9-axis absolute orientation IMUs
* [adafruit circuitpython-gps](https://docs.circuitpython.org/projects/gps/en/latest/) - Python package for interfacing with Adafruit's PA1616D "Ultimate GPS Breakout"
* [pyserial](https://pypi.org/project/pyserial/) - Python package for serial communication


## Other Notes
By default, Ubuntu does not allow access to the Raspberry Pi's built-in RX/TX serial pins, which we use to communicate with the GPS. We followed [these instructions](https://raspberrypi.stackexchange.com/a/116860) to enable hardware serial access.

# Scout
Scout uses MAVLINK to communicate with the drone. The MAVLINK code is written in C++ and is attached to the python script that is used for the search algorithm. The C++ code is located in the MAVLINK/examples folder, but first needs to be copied and pasted into the MAVLINK/examples folder. Then run the python script in scout/demo_missions/scout_search.py and the C++ code will be executed. This script is expected to be connected to a simulation such as jmavsim, gazebo, or a real drone. (IP must be changed in the C++ script to connect to the real drone)

## PX4 Simulation
To run the PX4 simulation, follow the instructions in the [PX4 User Guide](https://docs.px4.io/master/en/simulation/ros_interface.html). The `px4` package is included as a submodule in this repository.

## TODO List: 
- Add more python functionalities for more drone control and autonomy.
- Add IP change in python so we can easily connect to the real drone or the simulation.
- Add offboard autonomous control when needed.
- Add Camera functionality on drone.
- Add CNN for object/person detection.
- Add autonomous search and rescue algorithm.
