#include <ros/ros.h>
#include <gtest/gtest.h>
#include <iostream>
//#include <opencv2/opencv.hpp>
#include "NodeParam.hpp"



TEST( NodeParamTest, GettersTest){
   NodeParam node;
   node.setCurrent({10,20});
   std::vector<double> curr = {10,20};
   EXPECT_EQ(node.getCurrent(), curr);
   node.setParent({20,25});
   std::vector<double> par = {20,25}; 
   EXPECT_EQ(node.getParent(), par );
   node.setGCost(25.0);
   EXPECT_EQ(node.getGCost(), 25.0);
   node.setHCost(10.0);
   EXPECT_EQ(node.getHCost(), 10.0);
   node.setTotalCost(35.0);
   EXPECT_EQ(node.getTotalCost(), 35.0);
   node.settheta(5.0);
   EXPECT_EQ(node.getTheta(), 5.0);
   node.setDtheta(2.0);
   EXPECT_EQ(node.getDTheta(), 2.0);
   node.setDx(0.5);
   EXPECT_EQ(node.getDx(), 0.5);
   node.setDy(0.5);
   EXPECT_EQ(node.getDy(), 0.5);
}  
















