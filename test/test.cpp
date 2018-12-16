
#include <ros/ros.h>
#include <gtest/gtest.h>
#include "rrt_global_planner_plugin.h"


// ========== test vertex ==============//

///< test setPosition, getPosition
TEST(TESTVertex, testFunc1) {
  vertex vtx;

  vtx.setPosition(10.4, 15.7);
  std::pair<float, float> p = std::make_pair(10.4, 15.7);
  EXPECT_EQ(p, vtx.getPosition());
}

///< test setParentIdx, getParentIdx
TEST(TESTVertex, testFunc2) {
  vertex vtx;

  vtx.setParentIdx(5);
  EXPECT_EQ(5, vtx.getParentIdx());

}

///< test setIdx, getIdx
TEST(TESTVertex, testFunc3) {
  vertex vtx;

  vtx.setIdx(2);
  EXPECT_EQ(2, vtx.getIdx());
}


// ========== test planner 1 ==============//

TEST(TESTRRTplanner, testFunc1) {
  int i = 3;
  int j = 4;

  rrt_planner::rrtPlannerROS planner;
  planner.width = 10;

  EXPECT_EQ(34, planner.getCellIndex(i, j));
}

TEST(TESTRRTplanner, testFunc2) {
  rrt_planner::rrtPlannerROS planner;
  planner.width = 10;

  EXPECT_EQ(11, planner.getCellRowID(116));
}

TEST(TESTRRTplanner, testFunc3) {
  rrt_planner::rrtPlannerROS planner;
  planner.width = 10;

  EXPECT_EQ(6, planner.getCellColID(116));
}

// ========== test planner 2 ==============//

TEST(TESTRRTplanner, testFunc4) {
  float x = 12.5;
  float y = 7.3;

  rrt_planner::rrtPlannerROS planner;
  planner.originX = 2.5;
  planner.originY = 2.8;

  planner.getCorrdinate(x, y);

  EXPECT_EQ(10.0, x);
  EXPECT_EQ(4.5, y);
}

TEST(TESTRRTplanner, testFunc5) {
  float x = 50.0;
  float y = 65.0;

  rrt_planner::rrtPlannerROS planner;
  planner.width = 4;
  planner.resolution = 10.0;

  EXPECT_EQ(29, planner.convertToCellIndex(x, y));
}


TEST(TESTRRTplanner, testFunc6) {
  int index = 125;
  float x = 0.0;
  float y = 0.0;

  rrt_planner::rrtPlannerROS planner;
  planner.originX = 2.5;
  planner.originY = 3.0;
  planner.width = 10;
  planner.resolution = 5.0;

  planner.convertToCoordinate(index, x, y);

  EXPECT_EQ(27.5, x);
  EXPECT_EQ(63.0, y);
}


TEST(TESTRRTplanner, testFunc7) {
  float x = 40.0;
  float y = 50.0;

  rrt_planner::rrtPlannerROS planner;
  planner.height = 20;
  planner.width = 10;
  planner.resolution = 5.0;

  EXPECT_TRUE(planner.isCellInsideMap(x, y));
}


// ========== test planner 3 ==============//

TEST(TESTRRTplanner, testFunc8) {
  rrt_planner::rrtPlannerROS planner;
  planner.height = 200;
  planner.width = 150;

  std::pair<float, float> p = planner.GetRandomPoint();

  EXPECT_LT(p.first, 200.0);
  EXPECT_GE(p.first, 0.0);
  EXPECT_LT(p.second, 150.0);
  EXPECT_GE(p.second, 0.0);
}


TEST(TESTRRTplanner, testFunc9) {
  rrt_planner::rrtPlannerROS planner;
  planner.occupiedGridMap = new bool[100];

  for(int i=0; i<100; i++){
    if(i<50){
      planner.occupiedGridMap[i] = true;
    } else {
      planner.occupiedGridMap[i] = false;
    }
  }

  EXPECT_TRUE(planner.isFree(10));
  EXPECT_FALSE(planner.isFree(56));
}


TEST(TESTRRTplanner, testFunc10) {
  rrt_planner::rrtPlannerROS planner;
  planner.width = 10;
  planner.occupiedGridMap = new bool[100];

  for(int i=0; i<100; i++){
    if(i<50){
      planner.occupiedGridMap[i] = true;
    } else {
      planner.occupiedGridMap[i] = false;
    }
  }

  EXPECT_TRUE(planner.isFree(1, 0));
  EXPECT_FALSE(planner.isFree(5, 6));
}


TEST(TESTRRTplanner, testFunc11) {
  rrt_planner::rrtPlannerROS planner;
  planner.width = 10;
  planner.occupiedGridMap = new bool[100];

  for(int i=0; i<100; i++){
    if(i<50){
      planner.occupiedGridMap[i] = true;
    } else {
      planner.occupiedGridMap[i] = false;
    }
  }

  EXPECT_TRUE(planner.isStartAndGoalCellsValid(15, 25));
  EXPECT_FALSE(planner.isStartAndGoalCellsValid(15, 65));
}


TEST(TESTRRTplanner, testFunc12) {
  rrt_planner::rrtPlannerROS planner;
  planner.height = 10;
  planner.width = 10;
  planner.occupiedGridMap = new bool[100];

  for(int i=0; i<100; i++){
    if(i<50){
      planner.occupiedGridMap[i] = true;
    } else {
      planner.occupiedGridMap[i] = false;
    }
  }

  std::vector<int> path = planner.rrtPlanner(1, 50);

  EXPECT_GT(10, path.size());
}



// ========== run all tests ==============//

int main(int argc, char **argv) {
  ros::init(argc, argv, "plugin_rostest");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


