

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
