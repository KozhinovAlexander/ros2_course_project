# My ROS2 Course project
My ROS2 course repository based on [ROS2 For Beginners (ROS Foxy, Humble - 2022)](https://www.udemy.com/course/ros2-for-beginners/)

## Developemnt Environment used

- Operating System: [Ubuntu 22.04 LTS](https://releases.ubuntu.com/22.04/)
- ROS2 distribution: [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
- IDE: [Visual Studio Code v1.74.0](https://code.visualstudio.com/updates/v1_74)


# Getting Started

- Install ROS2 Humble according to installation steps [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

- Clone this repository:
```shell
git clone https://github.com/Nukersson/ros2_course_project.git
```

- Putt following lines to your `~/.bashrc` file:
```shell
source /opt/ros/humble/setup.bash
source <ros2_course_project_directory>/install/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```

# Building Separate Packages

There are following packages defined:

- turtlesim_controller

All mentioned packages are C++ packages and can be build with ROS2's colcon build system:

```shell
cd <ros2_course_project_dir>
colcon build --packages-select <package_name>
```

for `turtlesim_controller` package it could be done as follows:

```shell
colcon build --packages-select turtlesim_controller
```

# Starting Separate Packages

Each package contains different executables which can be started as follwos:

```shell
ros2 run <package_name> <executable_name>
```

for `turtlesim_controller` package and `turtlesim_controller` it could be done as follows:

```shell
ros2 run turtlesim_controller turtlesim_controller
```

# Starting Whole Project with Launch System

Launchin the project can be done with following command:

```shell
ros2 launch catch_them_all_bringup catch_them_all_launch.py
```
