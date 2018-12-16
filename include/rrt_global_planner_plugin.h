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


/** @file rrt_global_planner_plugin.h
 *  @brief Definition of rrt global planner
 *  @copyright (c) 2018 Chien-Te Lee, Chin-Po Tsai 
 *  @author Chien-Te Lee, Chin-Po Tsai
 *  @date   12/12/2018
 *
 *  This file is the definition for rrt
*/


#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <set>
#include <string>
#include <vector>
#include <utility>
#include <random>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <boost/foreach.hpp>

#ifndef INCLUDE_RRT_GLOBAL_PLANNER_PLUGIN_H_
#define INCLUDE_RRT_GLOBAL_PLANNER_PLUGIN_H_


/**
 *  @brief definition of a point in RRT
*/
class vertex {
 private:
    ///< the coordinate of the vertex
    std::pair<float, float> position;

    ///< the parent index of the vertex
    int parentIdx = 0;

    ///< the index of the vertex
    int idx = 0;

 public:
    ///< set the coordinate of the vertex
    void setPosition(float x, float y);

    ///< set the parent index of the vertex
    void setParentIdx(int idx);

    ///< set the index of the vertex
    void setIdx(int i);

    ///< get the coordinate of the vertex
    std::pair<float, float> getPosition();

    ///< get the parent index of the vertex
    int getParentIdx();

    ///< get the index of the vertex
    int getIdx();
};


namespace rrt_planner {
class rrtPlannerROS : public nav_core::BaseGlobalPlanner {
 public:
    ///< constructor
    rrtPlannerROS();
    rrtPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    ///< the node handler
    ros::NodeHandle ROSNodeHandle;

    /**
    *   @brief This function initialize the planner, and use the information provided by pgm/yaml map file
    *   @param name, costmap_ros
    *   @return none
    */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
    *   @brief This function works as the main function for calling planning algorithms
    *   @param start, goal, plan
    *   @return bool
    */
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

    ///< matrix information
    int width;
    int height;
    int mapSize;
    float resolution;

    ///< the obstacle map
    bool* occupiedGridMap;

    ///< the origin coordinate
    float originX;
    float originY;

    ///< the parameter of rrt algorithm
    double step_size_;
    double min_dist_from_robot_;
    bool initialized_;

    ///< the map information
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;

    /**
    *   @brief This function does coordinate shift
    *   @param (x, y)
    *   @return void
    */
    void getCorrdinate(float& x, float& y);

    /**
    *   @brief This function does coordinate transform whth a resolution to get the cell index
    *   @param (x, y)
    *   @return int
    */
    int convertToCellIndex(float x, float y);

    /**
    *   @brief This function does coordinate transform
    *   @param (index, x, y)
    *   @return void
    */
    void convertToCoordinate(int index, float& x, float& y);

    /**
    *   @brief check if the point is inside map 
    *   @param (x, y)
    *   @return bool
    */
    bool isCellInsideMap(float x, float y);

    /**
    *   @brief the core algorithm of rrt
    *   @param startCell, goalCell
    *   @return std::vector<int>
    */
    std::vector<int> rrtPlanner(int startCell, int goalCell);

    /**
    *   @brief This function indicates if a start/goal point is valid
    *   @param startCell/goalCell, the index of the occupiedGridMap
    *   @return if the cell is free
    */
    bool isStartAndGoalCellsValid(int startCell, int goalCell);


    /**
    *   @brief This function indicates if a grid is occupied
    *   @param (i, j), the coordinate of the occupiedGridMap
    *   @return if the cell is free
    */
    bool isFree(int i, int j);

    /**
    *   @brief This function indicates if a grid is occupied
    *   @param CellID, the index of the occupiedGridMap
    *   @return if the cell is free
    */
    bool isFree(int CellID);

    /**
    *   @brief get a random point in the map
    *   @param void
    *   @return pair<int, int>
    */
    std::pair<int, int> GetRandomPoint();

    /**
    *   @brief get the array index by the matrix coordinate 
    *   @param (i, j)
    *   @return the index of the array
    */
    int getCellIndex(int i, int j);


    /**
    *   @brief get the row of the matrix coordinate by array index
    *   @param index
    *   @return row of the matrix
    */
    int getCellRowID(int index);

    /**
    *   @brief get the col of the matrix coordinate by array index
    *   @param index
    *   @return col of the matrix
    */
    int getCellColID(int index);
};
};  // namespace rrt_planner
#endif  // INCLUDE_RRT_GLOBAL_PLANNER_PLUGIN_H_"
