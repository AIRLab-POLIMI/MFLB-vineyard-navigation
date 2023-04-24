#include <fre_row_navigation/cone.h>

double Cone::width() { return std::abs(endAngle - startAngle); }

double Cone::middle() { return (endAngle + startAngle) / 2; }

tf::Point Cone::getMiddleLinePoint() {
  tf::Point point;
  point.setX(std::cos(middle()) * ray_length);
  point.setY(std::sin(middle()) * ray_length);
  point.setZ(0.0);
  return point;
}

Cone Cone::findCone(const sensor_msgs::LaserScan& scan, double center_angle,
                    double ray_length, double obs_angle,
                    double min_obs_density) {
  Cone cone;
  cone.frame_id = scan.header.frame_id;	
  cone.ray_length = ray_length;

  int start_index = (center_angle - scan.angle_min) / scan.angle_increment;	// angle_min : starting angle of the laserscan

  const size_t obstacle_window_size =
      std::max(static_cast<size_t>(1), static_cast<size_t>(std::abs(std::ceil(obs_angle / scan.angle_increment))));
  bool obstacle_window[obstacle_window_size];

  // set end angle
  size_t obstacle_window_index = 0;
  std::fill(obstacle_window, obstacle_window + obstacle_window_size, false);
  for (int i = start_index; i < scan.ranges.size(); ++i) {
    double distance = scan.ranges[i];
    double angle = scan.angle_min + i * scan.angle_increment;
    // Set cone border here if this is the last ray
    if (i == scan.ranges.size() - 1) {
      cone.endAngle = angle;
      break;
    }

    bool hit =!(distance < 0.1 || std::isnan(distance)) && (distance < ray_length);

    obstacle_window[obstacle_window_index] = hit;
    double points_in_cone = std::count(obstacle_window, obstacle_window + obstacle_window_size, true);
    // Set cone border here if obstacle window has a high enough density
    if (points_in_cone / obstacle_window_size > min_obs_density) {
      cone.endAngle = angle + obs_angle / 2.;
      break;
    }
    obstacle_window_index = (obstacle_window_index + 1) % obstacle_window_size;
  }

  // set start angle
  obstacle_window_index = 0;
  std::fill(obstacle_window, obstacle_window + obstacle_window_size, false);
  for (int i = start_index; i >= 0; --i) {
    double distance = scan.ranges[i];
    double angle = scan.angle_min + i * scan.angle_increment;
    // Set cone border here if this is the last ray
    if (i == 0) {
      cone.startAngle = angle;
      break;
    }

    bool hit =!(distance < 0.1 || std::isnan(distance)) && distance < ray_length;

    obstacle_window[obstacle_window_index] = hit;
    double points_in_cone = std::count(obstacle_window, obstacle_window + obstacle_window_size, true);

    // Set cone border here if obstacle window has a high enough density
    if (points_in_cone / obstacle_window_size > min_obs_density) {
      cone.startAngle = angle - obs_angle / 2.;
      break;
    }
    obstacle_window_index = (obstacle_window_index + 1) % obstacle_window_size;
  }

  return cone;
}

visualization_msgs::Marker Cone::createMarker(int id) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
	marker.ns = "cone";
	marker.id = id;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.05;
	marker.color.r = 1;
	marker.color.a = 1;
	marker.lifetime=ros::Duration(3.5);

	// Lateral line of the cone
	geometry_msgs::Point p1;
	p1.x = std::cos(startAngle) * ray_length;
	p1.y = std::sin(startAngle) * ray_length;
	marker.points.push_back(p1);

	geometry_msgs::Point p2;
	marker.points.push_back(p2);

	// Central line of the cone
	geometry_msgs::Point p4;
	p4.x = std::cos(middle()) * ray_length;
	p4.y = std::sin(middle()) * ray_length;
	marker.points.push_back(p4);

	marker.points.push_back(p2);

	// Lateral line of the cone
	geometry_msgs::Point p3;
	p3.x = std::cos(endAngle) * ray_length;
	p3.y = std::sin(endAngle) * ray_length;
	marker.points.push_back(p3);

	return marker;
}
