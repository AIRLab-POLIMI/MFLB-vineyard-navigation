#include <fre_row_navigation/rectangle.h>

unsigned int Rectangle::getNumPointsInsideRectangle(
    const pcl::PointCloud<pcl::PointXYZ>& points) {
  unsigned int counter = 0;

  float xmin = std::min(x1, x2);
  float xmax = std::max(x1, x2);
  float ymin = std::min(y1, y2);
  float ymax = std::max(y1, y2);

  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = points.begin();
       it != points.end(); ++it) {
    if (it->x >= xmin && it->x <= xmax && it->y >= ymin && it->y <= ymax) {
      ++counter;
    }
  }
  return counter;
}

Rectangle Rectangle::grow_y(float x1, float x2, float y1, float growthStep,
                            float y2_max,
                            const pcl::PointCloud<pcl::PointXYZ>& points,
                            unsigned int maxPoints) {
	Rectangle rect;
	rect.x1 = x1;
	rect.x2 = x2;
	rect.y1 = y1;
	rect.y2 = y1;
	while ((growthStep > 0 ? rect.y2 <= y2_max : rect.y2 >= y2_max) &&
	 rect.getNumPointsInsideRectangle(points) < maxPoints) {
	rect.y2 += growthStep;
	}
	return rect;
}

Rectangle Rectangle::grow_x(float x1, float y1, float y2, float growthStep,
                            float x2_max,
                            const pcl::PointCloud<pcl::PointXYZ>& points,
                            unsigned int maxPoints) {
  Rectangle rect;
  rect.x1 = x1;
  rect.x2 = x1;
  rect.y1 = y1;
  rect.y2 = y2;
  while ((growthStep > 0 ? rect.x2 <= x2_max : rect.x2 >= x2_max) &&
         rect.getNumPointsInsideRectangle(points) < maxPoints) {
    rect.x2 += growthStep;
  }
  return rect;
}

visualization_msgs::Marker Rectangle::createMarker(int id,
                                                   std::string frame_id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "rect";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = (x1 + x2) / 2;
  marker.pose.position.y = (y1 + y2) / 2;
  marker.scale.x = x2 - x1;
  marker.scale.y = y2 - y1;
  marker.scale.z = 0.01;
  marker.color.g = 1;
  marker.color.a = 1;
  marker.lifetime=ros::Duration(3.5);
  return marker;
}
