ros_canopen
===========

Forked from [ros_canopen](https://github.com/ros-industrial/ros_canopen). Adapted to allow multiple motors per node, as the Roboteq SBL2360 controllers used for HEROX support this.

Canopen implementation for ROS.

[![Build Status](https://travis-ci.com/ros-industrial/ros_canopen.svg?branch=melodic-devel)](https://travis-ci.com/ros-industrial/ros_canopen)
[![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0)
([![License: BSD 3-Clause](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause) for `can_msgs` and `socketcan_bridge`)

The current develop branch is `melodic-devel`, it targets ROS `melodic`. Needs C++14 compiler.
The released version gets synced over to the distro branch for each release.

# Installation

Clone this repository into a catkin workspace:

```bash
ros@ros-usb:~/catkin_ws/src$ git clone git@gitlab.com:kea-robotics/industry_projects/21_amr/ros_canopen.git
```

Then, install system dependencies using `rosdep`:

```bash
ros@ros-usb:~/catkin_ws/src$ rosdep install --from-paths ros_canopen --ignore-src -r -y
```
