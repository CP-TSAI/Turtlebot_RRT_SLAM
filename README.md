# Turtlebot_RRT_SLAM
[![Build Status](https://travis-ci.org/CP-TSAI/Turtlebot_RRT_SLAM.svg?branch=master)](https://travis-ci.org/CP-TSAI/Turtlebot_RRT_SLAM)
[![Coverage Status](https://coveralls.io/repos/github/CP-TSAI/Turtlebot_RRT_SLAM/badge.svg?branch=master)](https://coveralls.io/github/CP-TSAI/Turtlebot_RRT_SLAM?branch=master)
---

## Overview
In recent years, SLAM (Simultaneous Localization and Mapping) applications have been developed in mobile robots, autonomous cars, and drones. Given an unknown environment, the robot needs to know what the environment looks like, where it is in the environment, and how it can move from the current position to the target position without colliding with the obstacles. Next, it is crucial for the robot to be capable of moving in a dynamic environment, such as some moving or unpredictable obstacles not shown in the map.  
In this project, a Turtlebot is exploring in an unknown environment simulated by Gazebo. We first use SLAM to build a 2D map for Turtlebot to run AMCL localization. Next, the Turtlebot will navigate in Gazebo environment using RRT(Rapidly-Exploring Random Tree) path planning algorithm. Moreover, we spawn a coke can as an unexpected obstacle for Turtlebot to avoid. Finally, we use a subscriber node to output wheel velocity of the Turtlebot.   
The demo of this project contains 3 parts: (1) launching RRT planner plugin, (2) spawn coke can for unexpected obstacle, and (3) use a node (twist_to_wheel) to output the velocity. Launching RRT plugin using AMCL and self-implemented RRT plugin for global path planning. Next, spawning a coke can show RRT replanning for unexpected obastacle. Finally use a subscriber node to check wheel speed of Turtlebot.



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
- Gazebo
- RViz

## Standard install via command-line
- create catkin workspace
- clone Turtlebot_RRT_SLAM package into catkin workspace src file
- catkin_make catkin workspace
```
$ source /opt/ros/kinetic/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/CP-TSAI/Turtlebot_RRT_SLAM.git
$ cd ~/catkin_ws
$ catkin_make
```

## How to run demo simulation using launch file 
- use launch file to run rrt-plugin simulation
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ roslaunch Turtlebot_RRT_SLAM all.launch
```

- spawn coke can (unexpected obstacle) in Gazebo
```
$ roslaunch Turtlebot_RRT_SLAM all.launch is_obs:=enable
```

- print our wheel velocity of Turtlebot (differential drive)
```
$ rosrun Turtlebot_RRT_SLAM ttw
```



## Demo Result

- The initial scenario

<img src="https://github.com/CP-TSAI/Turtlebot_RRT_SLAM/raw/master/pic/initial.png" width="75%" height="75%"> 


- The RRT planning process

<img src="https://github.com/CP-TSAI/Turtlebot_RRT_SLAM/raw/master/pic/planning.png" width="75%" height="75%"> 


- Spawning an additional obstacle (coke can) that is not in the original SLAM map

<img src="https://github.com/CP-TSAI/Turtlebot_RRT_SLAM/raw/master/pic/spawn_coke.png" width="75%" height="75%"> 


- Avoidancing unexpected obstacle

<img src="https://github.com/CP-TSAI/Turtlebot_RRT_SLAM/raw/master/pic/replan.png" width="75%" height="75%"> 








## How to compile and run gtest
```
$ cd ~/catkin_ws
$ catkin_make run_tests
```

## Known issues and bugs
- map created using SLAM may sometimes be inaccurate
- rrt planner may change path occasionally



### How to see the information?

- How to see the **current planner**?
```
$ rosparam get /move_base/base_global_planner
```

- How to see the output **TWIST velocity** in ROS navigation

```
$ rostopic echo /navigation_velocity_smoother/raw_cmd_vel
```


### How to generate Doxygen

- In your doxygen folder, run 

```
$ doxygen Doxygenfile
```

- Then it generates **.html** and **.latex** file

- [Doxygen output](http://htmlpreview.github.io/?https://github.com/CP-TSAI/Turtlebot_RRT_SLAM/blob/master/doxygen/html/index.html)



## Personnel
- Chin-Po Tsai  
I am a robotics-majored 2nd year student, who wants to become a developer in Acme Robotics. 

- Chien-Te Lee  
A UMD robotics student taking ENPM808X course, hoping to acquire a researcher position in Acme Robotics Company. 

## Pair Programming and SIP Process
The product backlog googlesheet is at: [Product Backlog](https://drive.google.com/open?id=1GGu_NdKpPYwJQIi1h2X-HnGCWf6llKr8mGp2iHy4rJw).  
The sprint planning notes and review is at googledoc: [sprint planning and review](https://drive.google.com/open?id=1NWolqhI0ZdGPkRvtMio8TpJGyvjn2_onNdll_8KjDRM).

## Presentation

- [Power Point file](https://docs.google.com/presentation/d/1yzfVHWqFA2UFmxs-3oZu3o9SHaI5KFhRjaCeco7IOII/edit?usp=sharing)

- Description of this project: [Presentation video](https://youtu.be/ybhsXg99vLc)

- A simple RRT demo: [Turtlebot demo video 1](https://www.youtube.com/watch?v=rupK7FksiIM&feature=youtu.be)

- Spawn coke can and planning demo: [Turtlebot demo video 2](https://www.youtube.com/watch?v=9H27fU-CqfI&feature=youtu.be)

- Show the wheel velocity demo: [Turtlebot demo video 3](https://www.youtube.com/watch?v=8Xl9cwU-H28&feature=youtu.be)
