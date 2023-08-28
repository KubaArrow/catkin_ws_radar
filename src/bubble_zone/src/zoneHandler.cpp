#include "RadarScan.h"
#include "Zone.cpp"
#include "pugixml.cpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <algorithm>
#include <fstream>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <string>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
using namespace pugi;

int idMarker = 0;
ros::Publisher pubMarker;
ros::Publisher statusInfoPub;
std::vector<ros::Publisher> zonesPolygon;
ros::Publisher zonesStatusPub;
std::vector<bool> zonesCheck;
visualization_msgs::MarkerArray markerArray;
std::vector<Zone> vectorZones;
std::string main_tf = "base_link";
std::string xml_path = "Radary.xml";
ros::NodeHandle *hndl;

void drawZones();
void createZones();
void updateZones(const nav_msgs::Odometry rosMsg);
void pointCheck(const geometry_msgs::Point32 rosMsg);
void changeXML(std::string newPath);
void sendZonesTable();

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "zonesEngine");
  ros::NodeHandle hanlde;
  hndl = &hanlde;
  std::string pathTMP;
  if (ros::param::get("/bubble_zone/path_xml", pathTMP))
    xml_path = pathTMP;
  else
    return -2;

  if (ros::param::get("/bubble_zone/main_tf", pathTMP))
    main_tf = pathTMP;
  else
    return -3;
  pubMarker = hanlde.advertise<visualization_msgs::MarkerArray>(
      "visualization_marker", 1);
  statusInfoPub = hanlde.advertise<std_msgs::String>("status", 10);
  createZones();
  zonesStatusPub = hanlde.advertise<std_msgs::String>("zonesStatus", 10);

  drawZones();
  std::cout << "\n\nSUBSCRIBER STARTS...\n\n";
  ros::Subscriber pointSub = hanlde.subscribe("/radarsPoint", 1000, pointCheck);
  ros::Subscriber odomSub = hanlde.subscribe("/odom", 1000, updateZones);
  ros::spin();

  return 0;
}

void drawZones() {
  geometry_msgs::PolygonStamped poli;
  for (Zone &zone : vectorZones) {
    poli.polygon = zone.drawZone();
    poli.header.frame_id = main_tf;
    poli.header.stamp = ros::Time::now();
    zonesPolygon[zone.id].publish(poli);
  }
}

void createZones() {
  vectorZones.clear();
  std::cout << "CREATING ZONES...\n";
  xml_document doc;

  int id_count = 0;
  doc.load_file(xml_path.c_str());

  xml_node xml_Zones = doc.child("Zones");
  for (pugi::xml_node xml_zone = xml_Zones.child("Zone"); xml_zone;
       xml_zone = xml_zone.next_sibling("Zone")) {
    std::string typ, msg;
    unsigned char r, g, b;
    float a, afLinear, afAngular;
    bool work;

    work = xml_zone.attribute("work").as_bool();
    afLinear = xml_zone.attribute("amplificationFactorLinear").as_float();
    afAngular = xml_zone.attribute("amplificationFactorAngular").as_float();
    typ = xml_zone.attribute("zoneType").as_string();
    a = xml_zone.attribute("colorAlpha").as_float();
    r = xml_zone.attribute("colorRed").as_float();
    b = xml_zone.attribute("colorBlue").as_float();
    g = xml_zone.attribute("colorGreen").as_float();
    msg = xml_zone.attribute("msg").as_string();

    std::vector<geometry_msgs::Point32> vectorPoints;
    for (pugi::xml_node xml_points = xml_zone.child("Point"); xml_points;
         xml_points = xml_points.next_sibling("Point")) {

      geometry_msgs::Point32 tmp;
      tmp.x = xml_points.attribute("x").as_float();

      tmp.y = xml_points.attribute("y").as_float();

      tmp.z = 0;
      vectorPoints.push_back(tmp);
    }

    ros::param::set("/zone_" + std::to_string(id_count) + "_work", work);
    ros::param::set("/zone_" + std::to_string(id_count) +
                        "_amplificationFactorLinear",
                    afLinear);
    ros::param::set("/zone_" + std::to_string(id_count) +
                        "_amplificationFactorAngular",
                    afAngular);
    ros::param::set("/zone_" + std::to_string(id_count) + "_zoneType", typ);
    Zone m = Zone(id_count, work, typ, afLinear, afAngular, msg, a, r, g, b,
                  vectorPoints);

    vectorZones.push_back(m);
    id_count++;
  }
  zonesCheck.reserve(vectorZones.size());
  zonesPolygon.clear();
  for (Zone i : vectorZones) {
    std::string name = "zonesPolygon_" + std::to_string(i.id);
    zonesPolygon.push_back(
        hndl->advertise<geometry_msgs::PolygonStamped>(name, 1));
  }
  std::cout << "ZONES CREATED\n";
}

void pointCheck(const geometry_msgs::Point32 rosMsg) {
  idMarker++;
  bool color = true;
  if (std::abs(rosMsg.x) > 0.05 && std::abs(rosMsg.y) > 0.05) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CYLINDER;

    marker.ns = "basic_shapes";
    marker.id = idMarker;
    for (Zone i : vectorZones) {

      if (i.checkPoint(rosMsg)) {
        if (color) {
          marker.color = i.markerColor;
          color = false;
        }
        if (i.checkMsg != "") {
          std_msgs::String msg;
          msg.data = i.checkMsg;
          statusInfoPub.publish(msg);
          zonesCheck[i.id] = true;
        }
      }
    }
    if (color) {
      marker.color.a = 1;
      marker.color.r = 255;
      marker.color.b = 255;
      marker.color.g = 255;
    }
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = rosMsg.x;
    marker.pose.position.y = rosMsg.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.05;
    marker.color.a = 1;
    marker.lifetime = ros::Duration(0.2);
    markerArray.markers.push_back(marker);

  } else {
    std::string pathTMP;
    if (ros::param::get("path_xml", pathTMP)) {
      changeXML(pathTMP);
    }
    sendZonesTable();
    pubMarker.publish(markerArray);
    markerArray.markers.clear();
    drawZones();
    idMarker = 0;
  }
}

void updateZones(const nav_msgs::Odometry rosMsg) {
  for (Zone &i : vectorZones) {
    if (i.typeZone != "static")
      i.updateZones(rosMsg);
  }
}

void changeXML(std::string newPath) {
  if (xml_path != newPath) {
    xml_path = newPath;
    createZones();
  }
}

void sendZonesTable() {
  std_msgs::String msg;
  for (auto i : zonesCheck)
    if (i)
      msg.data.push_back('1');
    else
      msg.data.push_back('0');

  zonesStatusPub.publish(msg);
  zonesCheck.clear();
  zonesCheck.reserve(vectorZones.size());
  for (auto zone : vectorZones)
    zonesCheck.push_back(false);
}