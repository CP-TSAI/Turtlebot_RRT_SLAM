#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>


void twistCb(const geometry_msgs::TwistConstPtr &msg) {
  double transVelocity = msg->linear.x;
  double rotVelocity = msg->angular.z;
  double velDiff = (0.143 * rotVelocity) / 2.0;
  double leftPower = (transVelocity + velDiff) / 0.076;
  double rightPower = (transVelocity - velDiff) / 0.076;
  ROS_INFO_STREAM("\nLeft wheel: " << leftPower << ",  Right wheel: "<< rightPower << "\n");
}
