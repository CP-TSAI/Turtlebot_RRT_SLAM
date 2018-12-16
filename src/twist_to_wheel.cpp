/**
 *  BSD License
 *  Copyright <2018> <Chien-Te Lee> <Chin-Po Tsai>
 *  
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided that 
 *  the following conditions are met:
 *  
 *  1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *  following disclaimer.
 *  
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the 
 *  following disclaimer in the documentation and/or other materials provided with the distribution.
 *  
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or 
 *  promote products derived from this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
 *  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY 
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
 *  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
 *  DAMAGE.
*/


/** @file twist_to_wheel.cpp
 *  @brief Implementation of a subscriber for twist velocity topics 
 *  @copyright (c) 2018 Chien-Te Lee, Chin-Po Tsai 
 *  @author Chien-Te Lee, Chin-Po Tsai
 *  @date   12/12/2018
 *
 *  Implementation of a subscriber for twist velocity topics 
*/

#include <twist_to_wheel.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>


/**
*   @brief This function subscribes the velocity topics, and transform it to a motor velocity command in left/right wheel
*   @param void
*   @return int
*/
int main(int argc, char** argv) {
  ros::init(argc, argv, "twist_listener");
  ros::NodeHandle node;
  ros::Subscriber sub =
  node.subscribe("/navigation_velocity_smoother/raw_cmd_vel", 1000, &twistCb);
  ros::spin();
  return 0;
}


