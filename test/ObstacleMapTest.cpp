
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "ObstacleMap.hpp"



TEST( ObstacleMapTest, GetRadiusTest){
   ObstacleMap obsMap(1110, 1010);
   obsMap.setRadius(10.0);
   EXPECT_EQ(obsMap.getRadius(), 10.0);
}

TEST( ObstacleMapTest, CreateRectanglesTest){
   ObstacleMap obsMap(1110, 1010);
   obsMap.setRadius(0.0);
   bool req = obsMap.createRectangles(917,1069);
   EXPECT_EQ(req, true);
   bool req1 = obsMap.createRectangles(1109,294);
   EXPECT_EQ(req1, true);
   bool req2 = obsMap.createRectangles(0,0);
   EXPECT_EQ(req2,false);
   bool req3 = obsMap.createRectangles(1111,296);
   EXPECT_EQ(typeid(req3), typeid(bool));
}

TEST( ObstacleMapTest, CreateTablesTest){
   ObstacleMap obsMap(1110, 1010);
   obsMap.setRadius(0.0);
   bool req = obsMap.createTables(309,909);
   EXPECT_EQ(req, true);
   bool req1 = obsMap.createTables(0,0);
   EXPECT_EQ(req1, false);
   bool req2 = obsMap.createTables(1111,296);
   EXPECT_EQ(typeid(req2), typeid(bool));
}

TEST( ObstacleMapTest, CreateCirclesTest){
   ObstacleMap obsMap(1110, 1010);
   obsMap.setRadius(0.0);
   bool req = obsMap.createCircles(0,0);
   EXPECT_EQ(req, false);
   bool req1 = obsMap.createCircles(390, 965);
   EXPECT_EQ(req1,true);
   bool req2 = obsMap.createCircles(10,10);
   EXPECT_EQ(typeid(req2), typeid(bool));
}


TEST( ObstacleMapTest, DrawBoundaryTest){
   ObstacleMap obsMap(1110, 1010);
   obsMap.setRadius(0.0);
   bool req = obsMap.drawBoundary(0,0);
   EXPECT_EQ(req, true);
   bool req1 = obsMap.drawBoundary(555,250);
   EXPECT_EQ(req1,false);
   bool req2 = obsMap.drawBoundary(10,10);
   EXPECT_EQ(typeid(req2), typeid(bool));
}


TEST( ObstacleMapTest, CreateMapTest){
   ObstacleMap obsMap(1110, 1010);
   obsMap.setRadius(0.0);
   cv::Mat req = obsMap.createMap();
   EXPECT_EQ(req.at<double>(0,1), 0);
   cv::Mat req1 = obsMap.createMap();
   EXPECT_EQ(req1.at<double>(20,20),1);
   cv::Mat req2 = obsMap.createMap();
   EXPECT_EQ(typeid(req2), typeid(cv::Mat));
}















