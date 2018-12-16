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


/** @file rrt_global_planner_plugin.cpp
 *  @brief Implementation of rrt global planner
 *  @copyright (c) 2018 Chien-Te Lee, Chin-Po Tsai 
 *  @author Chien-Te Lee, Chin-Po Tsai
 *  @date   12/12/2018
 *
 *  This file is applied as a plugin planner for ROS navigation stack.
 *  A RRT planner is implented under the global planner frame work 
*/

#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <rrt_global_planner_plugin.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <map>


///< register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::rrtPlannerROS, nav_core::BaseGlobalPlanner)


///< class vertex mehtods
void vertex::setPosition(float x, float y) {
    position = std::make_pair(x, y);
}

///< class vertex mehtods
void vertex::setParentIdx(int idx) {
    parentIdx = idx;
}

///< class vertex mehtods
void vertex::setIdx(int i) {
    idx = i;
}

///< class vertex mehtods
std::pair<float, float> vertex::getPosition() {
    return position;
}

///< class vertex mehtods
int vertex::getParentIdx() {
    return parentIdx;
}

///< class vertex mehtods
int vertex::getIdx() {
    return idx;
}



///< class rrt_planner methods
namespace rrt_planner {
    ///< Constructors
    rrtPlannerROS::rrtPlannerROS() {}
    rrtPlannerROS::rrtPlannerROS(std::string name,
        costmap_2d::Costmap2DROS* costmap_ros) {
        initialize(name, costmap_ros);
    }


    /**
    *   @brief This function initialize the planner, and use the information provided by pgm/yaml map file
    *   @param name, costmap_ros
    *   @return none
    */
    void rrtPlannerROS::initialize(std::string name,
        costmap_2d::Costmap2DROS* costmap_ros) {
        if (!initialized_) {
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            ros::NodeHandle private_nh("~/" + name);

            ///< read the information from the map
            originX = costmap_->getOriginX();
            originY = costmap_->getOriginY();
            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            resolution = costmap_->getResolution();
            mapSize = width * height;

            ///< build the occupiedGridMap with the given map
            occupiedGridMap = new bool[mapSize];
            for (unsigned int iy = 0; iy < height; iy++) {
                for (unsigned int ix = 0; ix < width; ix++) {
                    unsigned int cost =
                    static_cast<int>(costmap_->getCost(ix, iy));
                    if (cost == 0)
                        occupiedGridMap[iy * width + ix] = true;
                    else
                        occupiedGridMap[iy * width + ix] = false;
                }
            }
            ROS_INFO("rrt planner initialized successfully");
            initialized_ = true;
        } else {
            ROS_WARN("This planner is initialized...");
        }
    }


    /**
    *   @brief This function works as the main function for calling planning algorithms
    *   @param start, goal, plan
    *   @return bool
    */
    bool rrtPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan) {
        if (!initialized_) {
            ROS_ERROR("The planner is not initialized yet");
            return false;
        }

        ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f",
            start.pose.position.x, start.pose.position.y,
            goal.pose.position.x, goal.pose.position.y);

        plan.clear();

        if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
            ROS_ERROR("frame error, only %s frame, not in the %s frame.",
                       costmap_ros_->getGlobalFrameID().c_str(),
                       goal.header.frame_id.c_str());
            return false;
        }

        ///< convert the start and goal positions
        float startX = start.pose.position.x;
        float startY = start.pose.position.y;

        float goalX = goal.pose.position.x;
        float goalY = goal.pose.position.y;

        ///< change the coordinate of the position.
        getCorrdinate(startX, startY);
        getCorrdinate(goalX, goalY);

        ///< get the cell number of start/goal
        int startCell;
        int goalCell;
        if (isCellInsideMap(startX, startY) && isCellInsideMap(goalX, goalY)) {
            startCell = convertToCellIndex(startX, startY);
            goalCell = convertToCellIndex(goalX, goalY);
        } else {
            ROS_WARN("the start or goal is out of the map");
            return false;
        }

        ///< call the rrt global planner
        if (isStartAndGoalCellsValid(startCell, goalCell)) {
            std::vector<int> bestPath;
            bestPath.clear();

            ///< start planning
            bestPath = rrtPlanner(startCell, goalCell);

            ///< if the global planner find a path
            if (bestPath.size() > 0) {
                ///< convert the path
                for (int i = 0; i < bestPath.size(); i++) {
                    float x = 0.0;
                    float y = 0.0;

                    int index = bestPath[i];

                    convertToCoordinate(index, x, y);

                    ///< convert the point to a "pose"
                    geometry_msgs::PoseStamped pose = goal;

                    pose.pose.position.x = x;
                    pose.pose.position.y = y;
                    pose.pose.position.z = 0.0;

                    pose.pose.orientation.x = 0.0;
                    pose.pose.orientation.y = 0.0;
                    pose.pose.orientation.z = 0.0;
                    pose.pose.orientation.w = 1.0;

                    plan.push_back(pose);
                }
                return true;
            } else {
                ROS_WARN("failed to find a path, choose aother goal");
                return false;
            }
        } else {
            ROS_WARN("Not valid start or goal");
            return false;
        }
    }


    /**
    *   @brief This function does coordinate shift
    *   @param (x, y)
    *   @return void
    */
    void rrtPlannerROS::getCorrdinate(float& x, float& y) {
        x = x - originX;
        y = y - originY;
    }


    /**
    *   @brief This function does coordinate transform whth a resolution to get the cell index
    *   @param (x, y)
    *   @return int
    */
    int rrtPlannerROS::convertToCellIndex(float x, float y) {
        int cellIndex;
        float newX = x / resolution;
        float newY = y / resolution;
        cellIndex = getCellIndex(newY, newX);
        return cellIndex;
    }


    /**
    *   @brief This function does coordinate transform
    *   @param (index, x, y)
    *   @return void
    */
    void rrtPlannerROS::convertToCoordinate(int index, float& x, float& y) {
        x = getCellColID(index) * resolution;
        y = getCellRowID(index) * resolution;

        x = x + originX;
        y = y + originY;
    }

    /**
    *   @brief check if the point is inside map 
    *   @param (x, y)
    *   @return bool
    */
    bool rrtPlannerROS::isCellInsideMap(float x, float y) {
        bool valid = true;
        if (x > (width * resolution) || y > (height * resolution))
            valid = false;
        return valid;
    }

    /**
    *   @brief get a random point in the map
    *   @param void
    *   @return pair<int, int>
    */
    std::pair<int, int> rrtPlannerROS::GetRandomPoint() {
        thread_local unsigned int seed = time(NULL);
        int x = rand_r(&seed) % height;
        int y = rand_r(&seed) % width;
        return std::make_pair(x, y);
    }


    /**
    *   @brief the core algorithm of rrt
    *   @param startCell, goalCell
    *   @return std::vector<int>
    */ 
    std::vector<int> rrtPlannerROS::rrtPlanner(int startCell, int goalCell) {
        int sampleNumber = 100000;
        int stepSize = 2;
        double prob_to_choose_dest = 0.1;


        vertex src, dest;
        src.setParentIdx(0);
        src.setIdx(1);
        src.setPosition(getCellRowID(startCell), getCellColID(startCell));
        dest.setPosition(getCellRowID(goalCell), getCellColID(goalCell));
        int index = 2;

        std::vector<vertex> rrt_vector = {src};
        std::map<int, vertex> rrt_dict = {{src.getIdx(), src}};

        ///< start sampling
        for (int i = 0; i < sampleNumber; i++) {
            /**
            *   randomly choose a point in the map, 
            *   with a small probab	ility to choose the destination point
            */ 
            thread_local unsigned int seed = time(NULL);
            double prob = static_cast<double>(rand_r(&seed) / (RAND_MAX));
            std::pair<int, int> q_rand;
            if (prob < prob_to_choose_dest)
                q_rand = dest.getPosition();
            else
                q_rand = GetRandomPoint();


            /**
            *   get the nearest vertex to the random point
            *   by iterating all the points in the search tree
            */
            double min_dist = 10000;
            int minIdx = 0;
            for (auto v : rrt_vector) {
                int diff_x = q_rand.first - v.getPosition().first;
                int diff_y = q_rand.second - v.getPosition().second;
                double dist = sqrt(diff_x * diff_x + diff_y * diff_y);
                if (dist < min_dist) {
                    min_dist = dist;
                    minIdx = v.getIdx();
                }
            }
            vertex q_near = rrt_dict[minIdx];


            /**
            *   build a new vertex along the direction: q_near->q_rand
            *   q_new is then added to the search tree. 
            */
            int diff_x = q_rand.first - q_near.getPosition().first;
            int diff_y = q_rand.second - q_near.getPosition().second;
            int diff = sqrt(diff_x * diff_x + diff_y * diff_y);
            vertex q_new;
            if (diff < stepSize) {  // go straight to q_rand
                q_new.setPosition(q_rand.first, q_rand.second);
            } else {  // go a small step in q_rand direction
                int q_new_x = q_near.getPosition().first +
                              stepSize * diff_x / diff;
                int q_new_y = q_near.getPosition().second +
                              stepSize * diff_y / diff;
                q_new.setPosition(q_new_x, q_new_y);
            }

            ///< exclude some ERRORS by checking if q_new is free
            if (!isFree(q_new.getPosition().first, q_new.getPosition().second))
                continue;

            ///< if q_new is valid, then save it to the search tree
            q_new.setParentIdx(q_near.getIdx());
            q_new.setIdx(index); index++;
            rrt_vector.push_back(q_new);
            rrt_dict[q_new.getIdx()] = q_new;

            ///< check if the goal the goal found
            if (q_new.getPosition() == dest.getPosition()) {
                ROS_INFO("found the goal!!");
                break;
            }
        }


        ///< trace back the search tree
        std::stack<vertex> path;

        ///< use the destination as the initial point
        vertex v_tmp = rrt_vector.back();

        ///< sequentially put the path in a stack, the top is the start point
        while (1) {
            path.push(v_tmp);
            int parent_idx = v_tmp.getParentIdx();
            if (parent_idx == 0) break;
            v_tmp = rrt_dict[parent_idx];
        }

        ///< trace the path and put them into bestPath vector
        std::vector<int> bestPath;
        bestPath.clear();
        while (!path.empty()) {
            vertex v = path.top(); path.pop();
            int array_position_index =
            getCellIndex(v.getPosition().first, v.getPosition().second);
            bestPath.push_back(array_position_index);
        }
        return bestPath;
    }


    /**
	*   @brief This function indicates if a start/goal point is valid
	*   @param startCell/goalCell, the index of the occupiedGridMap
	*   @return if the cell is free
	*/
    bool rrtPlannerROS::isStartAndGoalCellsValid(int startCell, int goalCell) {
        return isFree(startCell) && isFree(goalCell);
    }

    /**
	*   @brief This function indicates if a grid is occupied
	*   @param (i, j), the coordinate of the occupiedGridMap
	*   @return if the cell is free
	*/
    bool  rrtPlannerROS::isFree(int i, int j) {
        int CellID = getCellIndex(i, j);
        return occupiedGridMap[CellID];
    }

    /**
	*   @brief This function indicates if a grid is occupied
	*   @param CellID, the index of the occupiedGridMap
	*   @return if the cell is free
	*/
    bool rrtPlannerROS::isFree(int CellID) {
        return occupiedGridMap[CellID];
    }


    /**
	*   @brief get the array index by the matrix coordinate 
	*   @param (i, j)
	*   @return the index of the array
	*/
    int rrtPlannerROS::getCellIndex(int i, int j) {
        return (i * width) + j;
    }

    /**
	*   @brief get the row of the matrix coordinate by array index
	*   @param index
	*   @return row of the matrix
	*/
    int rrtPlannerROS::getCellRowID(int index) {
        return index / width;
    }

    /**
	*   @brief get the col of the matrix coordinate by array index
	*   @param index
	*   @return col of the matrix
	*/
    int rrtPlannerROS::getCellColID(int index) {
        return index % width;
    }

};  // namespace rrt_planner



