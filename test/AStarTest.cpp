#include <ros/ros.h>
#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <queue>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "AStar.hpp"


TEST( AStarTest, backTrackTest) {
  AStar obj(25,40,1);
  std::vector<std::vector<std::vector<double>>> backList;
  backList.push_back({{1,2}, {0,0}, {1.0}, {0.1}});
  backList.push_back({{3,4},{1,2},{2.0},{0.2}});
  backList.push_back({{5,6},{3,4},{3.0},{0.3}});
  std::vector<std::vector<std::vector<double>>> rosInputs = obj.backTrack(backList);
  EXPECT_EQ(rosInputs.at(0).at(0).at(0), 1.0);
  EXPECT_EQ(rosInputs.at(0).at(1).at(0), 0.1);   
}  

TEST( AStarTest, motionModelTest) {
  AStar obj(25,40,1);
  std::vector<std::vector<double>> actions = obj.motionModel();

  EXPECT_NEAR(actions.at(0).at(0), 5, 0.001);
  EXPECT_NEAR(actions.at(0).at(1), -1, 0.001);
  EXPECT_NEAR(actions.at(0).at(2), 5.09902, 0.001);
  EXPECT_NEAR(actions.at(0).at(3), -0.281028, 0.001);
  EXPECT_NEAR(actions.at(0).at(4), 4.97419, 0.001);
  EXPECT_NEAR(actions.at(0).at(5), -0.281028, 0.001);
}

TEST( AStarTest, aStarTest) {
  AStar obj(25,40,1);
  std::vector<double> stp{3,3};
  std::vector<double> etp{7,7};
  std::vector<std::vector<std::vector<double>>> dif = obj.aStar(stp,etp);
  EXPECT_NEAR(dif.at(0).at(0).at(0), 0, 0.0001);
  EXPECT_NEAR(dif.at(0).at(1).at(0), 0, 0.0001);
  EXPECT_NEAR(dif.at(1).at(0).at(0), 4.97419, 0.0001);
  EXPECT_NEAR(dif.at(1).at(1).at(0), 0.281028, 0.0001);
}

void velCallback(geometry_msgs::Twist vel) {
}

TEST( AStarTest, rosTurtleTest) {
  ros::NodeHandle n;
  ros::Publisher testPub = n.advertise < geometry_msgs::Twist > ("/cmd_vel", 1000);    
  ros::Subscriber testSub = n.subscribe("cmd_vel", 1000, velCallback);
  EXPECT_EQ(testPub.getNumSubscribers(), 1);
  EXPECT_EQ(testSub.getNumPublishers(), 1);

  }












