#include "Zone.h"

Zone::Zone(int identificator, bool onOff, std::string type, float afLinear,
           float afAngular, std::string msg, float alpha, unsigned char red,
           unsigned char green, unsigned char blue,
           std::vector<geometry_msgs::Point32> Points)
    : id(identificator), work(onOff), typeZone(type), vectorPoints(Points),
      actualVectorPoints(Points), checkMsg(msg),
      amplificationFactorLinear(afLinear),
      amplificationFactorAngular(afAngular) {
  markerColor.a = alpha;
  markerColor.r = red;
  markerColor.b = blue;
  markerColor.g = green;
}

geometry_msgs::Polygon Zone::drawZone() {
  bool paramGetWork;
  float paramGetLinear, paramGetAngular;
  std::string paramGetTyp;
  if (ros::param::get("/zone_" + std::to_string(id) + "_work", paramGetWork))
    work = paramGetWork;
  if (ros::param::get("/zone_" + std::to_string(id) +
                          "_amplificationFactorLinear",
                      paramGetLinear))
    amplificationFactorLinear = paramGetLinear;
  if (ros::param::get("/zone_" + std::to_string(id) +
                          "_amplificationFactorAngular",
                      paramGetAngular))
    amplificationFactorAngular = paramGetAngular;
  if (ros::param::get("/zone_" + std::to_string(id) + "_zoneType", paramGetTyp))
    typeZone = paramGetTyp;

  geometry_msgs::Polygon poly;

  if (work) {
    if (typeZone == "static")
      poly.points = vectorPoints;
    else {
      poly.points = actualVectorPoints;
    }
  }

  return poly;
}

bool Zone::checkPoint(const geometry_msgs::Point32 &p) {

  int n = actualVectorPoints.size();
  if (n < 3 || !work)
    return false;
  int counter = 0;
  int i;
  double xinters;
  geometry_msgs::Point32 p1, p2;

  p1 = actualVectorPoints[0];
  for (i = 1; i <= n; i++) {
    p2 = actualVectorPoints[i % n];
    if (p.y > std::min(p1.y, p2.y)) {
      if (p.y <= std::max(p1.y, p2.y)) {
        if (p.x <= std::max(p1.x, p2.x)) {
          if (p1.y != p2.y) {
            xinters = (p.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
            if (p1.x == p2.x || p.x <= xinters)
              counter++;
          }
        }
      }
    }
    p1 = p2;
  }

  if (counter % 2 == 0)
    return false;
  else
    return true;
}

void Zone::resetZone() { actualVectorPoints = vectorPoints; }

void Zone::updateZones(const nav_msgs::Odometry &rosMsg) {
  if (work) {
    if (typeZone == "extension_zone")
      extensionZoneUpdate(rosMsg);
    else if (typeZone == "half_extension_zone")
      halfExtensionZoneUpdate(rosMsg);
    else if (typeZone == "adaptive_rectangle")
      adaptiveRectangleUpdate(rosMsg);
  }
}
void Zone::extensionZoneUpdate(const nav_msgs::Odometry &rosMsg) {

  actualVectorPoints.clear();
  for (const auto &point : vectorPoints) {
    geometry_msgs::Point32 actualPoint;

    actualPoint.x = point.x + (point.x * amplificationFactorLinear *
                               std::abs(rosMsg.twist.twist.linear.x));
    actualPoint.y = point.y + (point.y * amplificationFactorLinear *
                               std::abs(rosMsg.twist.twist.linear.x));
    actualPoint.z = point.z;

    actualVectorPoints.push_back(actualPoint);
  }
}

void Zone::halfExtensionZoneUpdate(

    const nav_msgs::Odometry &rosMsg) {

  actualVectorPoints.clear();
  geometry_msgs::Point32 average;
  for (const auto &point : vectorPoints) {
    average.x += point.x / 3;
  }
  for (auto &point : vectorPoints) {
    if (rosMsg.twist.twist.linear.x > 0.005) {
      if (point.x > average.x) {
        geometry_msgs::Point32 actualPoint;

        actualPoint.x = point.x + (point.x * amplificationFactorLinear *
                                   std::abs(rosMsg.twist.twist.linear.x));
        actualPoint.y = point.y + (point.y * amplificationFactorLinear *
                                   std::abs(rosMsg.twist.twist.linear.x));
        actualPoint.z = point.z;
        actualVectorPoints.push_back(actualPoint);
      } else {
        actualVectorPoints.push_back(point);
      }
    } else if (rosMsg.twist.twist.linear.x < -0.005) {
      if (point.x < average.x) {
        geometry_msgs::Point32 actualPoint;

        actualPoint.x = point.x + (point.x * amplificationFactorLinear *
                                   std::abs(rosMsg.twist.twist.linear.x));
        actualPoint.y = point.y + (point.y * amplificationFactorLinear *
                                   std::abs(rosMsg.twist.twist.linear.x));
        actualPoint.z = point.z;
        actualVectorPoints.push_back(actualPoint);
      } else {
        actualVectorPoints.push_back(point);
      }
    } else
      actualVectorPoints = vectorPoints;
  }
}

void Zone::adaptiveRectangleUpdate(

    const nav_msgs::Odometry &rosMsg) {
  actualVectorPoints.clear();

  if (std::abs(rosMsg.twist.twist.angular.z) > 0.09) {
    geometry_msgs::Point32 specifyingPoint;
    for (const auto &point : vectorPoints) {
      specifyingPoint.x += point.x / vectorPoints.size();
    }
    for (const auto &point : vectorPoints)
      if (point.x > specifyingPoint.x) {
        specifyingPoint = point;
        break;
      }
    for (const auto &point : vectorPoints)
      if (point.x > specifyingPoint.x) {
        specifyingPoint = point;
        break;
      }

    float maxX =
        specifyingPoint.x + (specifyingPoint.x * amplificationFactorLinear *
                             std::abs(rosMsg.twist.twist.linear.x));

    float width = std::abs(specifyingPoint.y) * 2;
    float maxY =
        (rosMsg.twist.twist.angular.z - width) * amplificationFactorAngular;

    actualVectorPoints.push_back(vectorPoints[3]);
    for (int i = 1; i < 4; i++) {
      geometry_msgs::Point32 p;
      p.x = (maxX / 3) * i;
      p.y = specifyingPoint.y + ((maxY / 14) * i * i);
      actualVectorPoints.push_back(p);
    }
    for (int i = 3; i >= 0; i--) {
      geometry_msgs::Point32 p;
      p.x = actualVectorPoints[i].x;
      p.y = actualVectorPoints[i].y + width;
      actualVectorPoints.push_back(p);
    }

  } else {
    halfExtensionZoneUpdate(rosMsg);
  }
}
