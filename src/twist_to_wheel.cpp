#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include "twist_to_wheel.h"


int main(int argc, char** argv){
  ros::init(argc, argv, "twist_listener");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/navigation_velocity_smoother/raw_cmd_vel", 1000, &twistCb);
  ros::spin();
  return 0;
};


