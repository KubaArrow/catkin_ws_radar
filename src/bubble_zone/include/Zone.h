#ifndef _Zone_
#define _Zone_

#include "ros/ros.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/ColorRGBA.h>
#include <string>
#include <vector>

class Zone {
private:
  std::vector<geometry_msgs::Point32> vectorPoints;
  std::vector<geometry_msgs::Point32> actualVectorPoints;
  bool work;
  float amplificationFactorLinear;
  float amplificationFactorAngular;
  void extensionZoneUpdate(const nav_msgs::Odometry &rosMsg);
  void halfExtensionZoneUpdate(const nav_msgs::Odometry &rosMsg);
  void adaptiveRectangleUpdate(const nav_msgs::Odometry &rosMsg);

public:
  int id;
  std::string typeZone;
  std_msgs::ColorRGBA markerColor;
  std::string checkMsg;
  Zone(int identificator, bool onOff, std::string type, float afLinear,
       float afAngular, std::string msg, float alpha, unsigned char red,
       unsigned char green, unsigned char blue,
       std::vector<geometry_msgs::Point32> Points);
  void dynamicZone();
  geometry_msgs::Polygon drawZone();
  void updateZones(const nav_msgs::Odometry &rosMsg);
  bool checkPoint(const geometry_msgs::Point32 &p);
  void resetZone();
};

#endif