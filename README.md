# Turtlebot_RRT_SLAM

## Overview

In recent years, SLAM (Simultaneous Localization and Mapping) applications have been developed in mobile robots, autonomous cars, and drones. Given an unknown environment, the robot needs to know what the environment looks like, where it is in the environment, and how it can move from the current position to the target position without colliding with the obstacles. Next, it is crucial for the robot to be capable of moving in a dynamic environment, such as some moving or unpredictable obstacles not shown in the map.
In this project, a Turtlebot is utilized in an unknown environment simulated by Gazebo. The Turtlebot will perform SLAM for robot mapping and localization to build a 2D map, and then it will navigate using RRT(Rapidly-Exploring Random Tree) path planning algorithm.
