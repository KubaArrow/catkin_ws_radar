#include "RadarScan.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>

ros::Publisher pub;
tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped transformStamped;
std::string number;
void readPointCloud(const ti_mmwave_rospkg::RadarScan rosMsg);

int main(int argc, char *argv[]) {

  number = argv[1];

  ros::init(argc, argv, "converterTF" + number);
  ros::NodeHandle hanlde;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Subscriber sub =
      hanlde.subscribe("/ti_mmwave/radar_scan_" + number, 1000, readPointCloud);
  pub = hanlde.advertise<geometry_msgs::Point32>("radarsPoint", 1000);

  ros::spin();

  return 0;
}

void readPointCloud(const ti_mmwave_rospkg::RadarScan rosMsg) {

  geometry_msgs::Point32 convertPoint;
  if (rosMsg.point_id != 0) {

    transformStamped.transform.translation.x;
    transformStamped.transform.rotation.z;
    tf2::Quaternion quat(transformStamped.transform.rotation.x,
                         transformStamped.transform.rotation.y,
                         transformStamped.transform.rotation.z,
                         transformStamped.transform.rotation.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    convertPoint.x = rosMsg.x * cos(yaw) - rosMsg.y * sin(yaw) +
                     transformStamped.transform.translation.x;
    convertPoint.y = rosMsg.x * sin(yaw) + rosMsg.y * cos(yaw) +
                     transformStamped.transform.translation.y;
    convertPoint.z = rosMsg.z + transformStamped.transform.translation.z;

    pub.publish(convertPoint);
  } else {
    pub.publish(convertPoint);
    std::string main_tf;
    ros::param::get("/bubble_zone/main_tf", main_tf);
    try {
      transformStamped = tfBuffer.lookupTransform("ti_mmwave_" + number,
                                                  main_tf, ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
    }
  }
}