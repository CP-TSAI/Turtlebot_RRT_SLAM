#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <set>
#include <vector>
#include <utility>


#ifndef RRT_GLOBAL_PLANNER_PLUGIN_H
#define RRT_GLOBAL_PLANNER_PLUGIN_H


class vertex {
 private:
    std::pair<float, float> position;
    int parentIdx = 0;
    int idx = 0;
 public:
    void setPosition(float x, float y);
    void setParentIdx(int idx);
    void setIdx(int i);
    std::pair<float, float> getPosition();
    int getParentIdx();
    int getIdx();
};


namespace rrt_planner {
class rrtPlannerROS : public nav_core::BaseGlobalPlanner {
 public:
    ///< constructor
    rrtPlannerROS();
    rrtPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    ///< inherited function from ROS navigation stack
    ros::NodeHandle ROSNodeHandle;
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

    ///< matrix information
    int width;
    int height;
    int mapSize;
    bool* occupiedGridMap;
    float originX;
    float originY;
    float resolution;
    double step_size_;
    double min_dist_from_robot_;
    bool initialized_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;

    ///< coordinate transformations
    void getCorrdinate(float& x, float& y);
    int convertToCellIndex(float x, float y);
    void convertToCoordinate(int index, float& x, float& y);
    bool isCellInsideMap(float x, float y);

    ///< planner
    std::vector<int> rrtPlanner(int startCell, int goalCell);
    bool isStartAndGoalCellsValid(int startCell, int goalCell);
    bool isFree(int i, int j); 
    bool isFree(int CellID);
    std::pair<int, int> GetRandomPoint();

    ///< matrix notation transformation
    int getCellIndex(int i,int j);
    int getCellRowID(int index);
    int getCellColID(int index);
};
};  // namespace rrt_planner
#endif
