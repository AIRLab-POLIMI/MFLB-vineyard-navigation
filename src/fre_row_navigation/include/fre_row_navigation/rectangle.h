#pragma once

#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>

class Rectangle {
public:
  float x1, y1, x2, y2;
  unsigned int getNumPointsInsideRectangle(
      const pcl::PointCloud<pcl::PointXYZ>& points);
  static Rectangle grow_y(float x1, float x2, float y1, float growthStep,
                          float y2_max,
                          const pcl::PointCloud<pcl::PointXYZ>& points,
                          unsigned int maxPoints);

  static Rectangle grow_x(float x1, float y1, float y2, float growthStep,
                          float x2_max,
                          const pcl::PointCloud<pcl::PointXYZ>& points,
                          unsigned int maxPoints);

  visualization_msgs::Marker createMarker(int id, std::string frame_id);
};