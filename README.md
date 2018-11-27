# Turtlebot_RRT_SLAM
[![Build Status](https://travis-ci.org/CP-TSAI/Turtlebot_RRT_SLAM.svg?branch=master)](https://travis-ci.org/CP-TSAI/Turtlebot_RRT_SLAM)
[![Coverage Status](https://coveralls.io/repos/github/CP-TSAI/Turtlebot_RRT_SLAM/badge.svg?branch=master)](https://coveralls.io/github/CP-TSAI/Turtlebot_RRT_SLAM?branch=master)
---

## Overview
In recent years, SLAM (Simultaneous Localization and Mapping) applications have been developed in mobile robots, autonomous cars, and drones. Given an unknown environment, the robot needs to know what the environment looks like, where it is in the environment, and how it can move from the current position to the target position without colliding with the obstacles. Next, it is crucial for the robot to be capable of moving in a dynamic environment, such as some moving or unpredictable obstacles not shown in the map.
In this project, a Turtlebot is utilized in an unknown environment simulated by Gazebo. The Turtlebot will perform SLAM for robot mapping and localization to build a 2D map, and then it will navigate using RRT(Rapidly-Exploring Random Tree) path planning algorithm.

## License
- BSD 3-clause license
 ```
Copyright <2018> <Chien-Te Lee> <Chin-Po Tsai>

Redistribution and use in source and binary forms, with or without modification, are permitted provided that 
the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
 following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the 
following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or 
promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY 
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
DAMAGE.
```

## Dependencies
- Ubuntu 16.04
- CMake
- ROS kinetic
- TurtleBot

## Standard install via command-line
- ccreate catkin workspace
- clone Turtlebot_RRT_SLAM package into catkin workspace
- build catkin workspace
```
$ source /opt/ros/kinetic/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/CP-TSAI/Turtlebot_RRT_SLAM.git
$ cd ~/catkin_ws
$ catkin_make
```
## How to run simulation
- use launch file to run simulation
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ roslaunch Turtlebot_RRT_SLAM Turtlebot_RRT_SLAM.launch
```

## How to compile and run gtest
$ cd ~/catkin_ws
$ catkin_make run_tests

## Known issues and bugs
- gazebo may shut down when doing gmapping
- map created using SLAM may sometimes be inaccurate

## Developers
- Chin-Po Tsai
- Chien-Te Lee

## Pair Programming and SIP Process
The product backlog googlesheet is at: [Product Backlog](https://drive.google.com/open?id=1GGu_NdKpPYwJQIi1h2X-HnGCWf6llKr8mGp2iHy4rJw)
The sprint planning notes and review is at googledoc is at: [sprint planning and review](https://drive.google.com/open?id=1NWolqhI0ZdGPkRvtMio8TpJGyvjn2_onNdll_8KjDRM)



