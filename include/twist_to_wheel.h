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


/** @file twist_to_wheel.h
 *  @brief Definition of twist_to_wheel call back function
 *  @copyright (c) 2018 Chien-Te Lee, Chin-Po Tsai 
 *  @author Chien-Te Lee, Chin-Po Tsai
 *  @date   12/12/2018
 *
 *  This file is the definition for subscriber
*/


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#ifndef INCLUDE_TWIST_TO_WHEEL_H_
#define INCLUDE_TWIST_TO_WHEEL_H_

/**
*   @brief tranform the msg to the velocity of both wheel
*   @param msg
*   @return void
*/
void twistCb(const geometry_msgs::TwistConstPtr &msg) {
  double transVelocity = msg->linear.x;
  double rotVelocity = msg->angular.z;
  double velDiff = (0.143 * rotVelocity) / 2.0;
  double leftPower = (transVelocity + velDiff) / 0.076;
  double rightPower = (transVelocity - velDiff) / 0.076;
  ROS_INFO_STREAM("\nLeft wheel: " << leftPower
                  << ",  Right wheel: "<< rightPower << "\n");
}
#endif  // INCLUDE_TWIST_TO_WHEEL_H_"
