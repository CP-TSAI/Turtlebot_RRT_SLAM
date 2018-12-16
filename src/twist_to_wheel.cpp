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


int main(int argc, char** argv){
  ros::init(argc, argv, "twist_listener");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/navigation_velocity_smoother/raw_cmd_vel", 1000, &twistCb);
  ros::spin();
  return 0;
};


