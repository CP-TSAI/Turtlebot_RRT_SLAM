#include <vector>
#include <map>
#include "rrt_global_planner_plugin.h"
#include <pluginlib/class_list_macros.h>


///< register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::rrtPlannerROS, nav_core::BaseGlobalPlanner)


///< class vertex mehtods
void vertex::setPosition(float x, float y) {
    position = std::make_pair(x, y);
}

void vertex::setParentIdx(int idx) {
    parentIdx = idx;
}


void vertex::setIdx(int i) {
    idx = i;
}

std::pair<float, float> vertex::getPosition() {
    return position;
}

int vertex::getParentIdx() {
    return parentIdx;
}

int vertex::getIdx() {
    return idx;
}