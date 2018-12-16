
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



// ========== run all tests ==============//

int main(int argc, char **argv) {
  ros::init(argc, argv, "plugin_rostest");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


