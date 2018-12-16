/**
 * BSD 3 clauses Liscense
 *
 * Copyright <2018> <Chien-Te Lee> <Chin-Po Tsai>
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions 
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in 
 * the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from 
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/** @file test.cpp
 *  @brief Implementation of unit test for rrt_planner plugin.
 *  @copyright (c) 2018 Chien-Te Lee, Chin-Po Tsai
 *  @author Chien-Te Lee, Chin-Po Tsai
 *  @date   12/16/2018
 *
 *  This program implemnts unit test for methods of class vertex and rrtPlannerROS, and subscriber callback function of twist_to_wheel.
 *  
 */

#include <rrt_global_planner_plugin.h>
#include <twist_to_wheel.h>
#include <ros/ros.h>
#include <gtest/gtest.h>



///< ========== test vertex methods ==============//

/**
 *  @brief This is a testcase for setPosition() and getPosition()
 *  @param TESTVertex is the name of the test suite
 *  @param testFunc1 is the name of the testcase
 *  @return none
 */
TEST(TESTVertex, testFunc1) {
    vertex vtx;

    vtx.setPosition(10.4, 15.7);
    std::pair<float, float> p = std::make_pair(10.4, 15.7);
    EXPECT_EQ(p, vtx.getPosition());
}

/**
 *  @brief This is a testcase for setParentIdx() and getParentIdx()
 *  @param TESTVertex is the name of the test suite
 *  @param testFunc2 is the name of the testcase
 *  @return none
 */
TEST(TESTVertex, testFunc2) {
    vertex vtx;

    vtx.setParentIdx(5);
    EXPECT_EQ(5, vtx.getParentIdx());
}

/**
 *  @brief This is a testcase for setIdx() and getIdx()
 *  @param TESTVertex is the name of the test suite
 *  @param testFunc3 is the name of the testcase
 *  @return none
 */
TEST(TESTVertex, testFunc3) {
    vertex vtx;

    vtx.setIdx(2);
    EXPECT_EQ(2, vtx.getIdx());
}


// ========== test rrt_planner methods 1 ==============//

/**
 *  @brief This is a testcase for getCellIndex()
 *  @param TESTRRTplanner is the name of the test suite
 *  @param testFunc1 is the name of the testcase
 *  @return none
 */
TEST(TESTRRTplanner, testFunc1) {
    int i = 3;
    int j = 4;

    rrt_planner::rrtPlannerROS planner;
    planner.width = 10;

    EXPECT_EQ(34, planner.getCellIndex(i, j));
}

/**
 *  @brief This is a testcase for getCellRowID()
 *  @param TESTRRTplanner is the name of the test suite
 *  @param testFunc2 is the name of the testcase
 *  @return none
 */
TEST(TESTRRTplanner, testFunc2) {
    rrt_planner::rrtPlannerROS planner;
    planner.width = 10;

    EXPECT_EQ(11, planner.getCellRowID(116));
}

/**
 *  @brief This is a testcase for getCellColID()
 *  @param TESTRRTplanner is the name of the test suite
 *  @param testFunc3 is the name of the testcase
 *  @return none
 */
TEST(TESTRRTplanner, testFunc3) {
    rrt_planner::rrtPlannerROS planner;
    planner.width = 10;

    EXPECT_EQ(6, planner.getCellColID(116));
}


// ========== test rrt_planner methods 2 ==============//

/**
 *  @brief This is a testcase for getCorrdinate()
 *  @param TESTRRTplanner is the name of the test suite
 *  @param testFunc4 is the name of the testcase
 *  @return none
 */
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

/**
 *  @brief This is a testcase for convertToCellIndex()
 *  @param TESTRRTplanner is the name of the test suite
 *  @param testFunc5 is the name of the testcase
 *  @return none
 */
TEST(TESTRRTplanner, testFunc5) {
    float x = 50.0;
    float y = 65.0;

    rrt_planner::rrtPlannerROS planner;
    planner.width = 4;
    planner.resolution = 10.0;

    EXPECT_EQ(29, planner.convertToCellIndex(x, y));
}

/**
 *  @brief This is a testcase for convertToCoordinate()
 *  @param TESTRRTplanner is the name of the test suite
 *  @param testFunc6 is the name of the testcase
 *  @return none
 */
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

/**
 *  @brief This is a testcase for isCellInsideMap()
 *  @param TESTRRTplanner is the name of the test suite
 *  @param testFunc7 is the name of the testcase
 *  @return none
 */
TEST(TESTRRTplanner, testFunc7) {
    float x = 40.0;
    float y = 50.0;

    rrt_planner::rrtPlannerROS planner;
    planner.height = 20;
    planner.width = 10;
    planner.resolution = 5.0;

    EXPECT_TRUE(planner.isCellInsideMap(x, y));
}


// ========== test rrt_planner methods 3 ==============//

/**
 *  @brief This is a testcase for GetRandomPoint()
 *  @param TESTRRTplanner is the name of the test suite
 *  @param testFunc8 is the name of the testcase
 *  @return none
 */
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

/**
 *  @brief This is a testcase for isFree()
 *  @param TESTRRTplanner is the name of the test suite
 *  @param testFunc9 is the name of the testcase
 *  @return none
 */
TEST(TESTRRTplanner, testFunc9) {
    rrt_planner::rrtPlannerROS planner;
    planner.occupiedGridMap = new bool[100];

    for (int i = 0; i < 100; i++) {
        if (i < 50) {
            planner.occupiedGridMap[i] = true;
        } else {
            planner.occupiedGridMap[i] = false;
        }
    }

    EXPECT_TRUE(planner.isFree(10));
    EXPECT_FALSE(planner.isFree(56));

    delete planner.occupiedGridMap;
}

/**
 *  @brief This is a testcase for isFree()
 *  @param TESTRRTplanner is the name of the test suite
 *  @param testFunc10 is the name of the testcase
 *  @return none
 */
TEST(TESTRRTplanner, testFunc10) {
    rrt_planner::rrtPlannerROS planner;
    planner.width = 10;
    planner.occupiedGridMap = new bool[100];

    for (int i = 0; i < 100; i++) {
        if (i < 50) {
            planner.occupiedGridMap[i] = true;
        } else {
            planner.occupiedGridMap[i] = false;
        }
    }

    EXPECT_TRUE(planner.isFree(1, 0));
    EXPECT_FALSE(planner.isFree(5, 6));

    delete planner.occupiedGridMap;
}

/**
 *  @brief This is a testcase for isStartAndGoalCellsValid()
 *  @param TESTRRTplanner is the name of the test suite
 *  @param testFunc11 is the name of the testcase
 *  @return none
 */
TEST(TESTRRTplanner, testFunc11) {
    rrt_planner::rrtPlannerROS planner;
    planner.width = 10;
    planner.occupiedGridMap = new bool[100];

    for (int i = 0; i < 100; i++) {
        if (i < 50) {
            planner.occupiedGridMap[i] = true;
        } else {
            planner.occupiedGridMap[i] = false;
        }
    }

    EXPECT_TRUE(planner.isStartAndGoalCellsValid(15, 25));
    EXPECT_FALSE(planner.isStartAndGoalCellsValid(15, 65));

    delete planner.occupiedGridMap;
}

/**
 *  @brief This is a testcase for rrtPlanner()
 *  @param TESTRRTplanner is the name of the test suite
 *  @param testFunc12 is the name of the testcase
 *  @return none
 */
TEST(TESTRRTplanner, testFunc12) {
    rrt_planner::rrtPlannerROS planner;
    planner.height = 10;
    planner.width = 10;
    planner.occupiedGridMap = new bool[100];

    for (int i = 0; i < 100; i++) {
        if (i < 50) {
            planner.occupiedGridMap[i] = true;
        } else {
            planner.occupiedGridMap[i] = false;
        }
    }

    std::vector<int> path = planner.rrtPlanner(1, 50);

    EXPECT_GT(10, path.size());

    delete planner.occupiedGridMap;
}


// ========== test twist to wheel callback function ===============//

/**
 *  @brief This is a testcase for subscriber callback funciton
 *  @param TESTSubscriberCallback is the name of the test suite
 *  @param testTwistCb is the name of the testcase
 *  @return none
 */
TEST(TESTSubscriberCallback, testTwistCb) {
    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;


    ros::NodeHandle nh;
    ros::Publisher pub =
    nh.advertise<geometry_msgs::Twist>
    ("/navigation_velocity_smoother/raw_cmd_vel", 5);

    ///< run publisher and check publisher and subscriber
    ros::WallDuration(5.0).sleep();
    ros::spinOnce();
    EXPECT_EQ(0, pub.getNumSubscribers());

    ///< run subscriber and check publisher and subscriber
    ros::Subscriber sub =
    nh.subscribe("/navigation_velocity_smoother/raw_cmd_vel", 5, &twistCb);

    for (int i = 0; i < 10; i++) {
        pub.publish(msg);
        ros::WallDuration(0.1).sleep();
        ros::spinOnce();
    }
    EXPECT_EQ(1, sub.getNumPublishers());
    EXPECT_EQ(1, pub.getNumSubscribers());
}




// ========== run all tests ==============//

/**
 *  @brief main fucntion to run all testcases
 *  @param argc is the number of input argument
 *  @param argv are the input arguments
 *  @return 0 if all the tests are successful
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "plugin_rostest");
    ros::NodeHandle nh;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


