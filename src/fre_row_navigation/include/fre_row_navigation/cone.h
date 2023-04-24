#pragma once

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

class Cone {
public:
  double startAngle;
  double endAngle;
  double ray_length;
  std::string frame_id;

  double width();
  double middle();
  tf::Point getMiddleLinePoint();
  visualization_msgs::Marker createMarker(int id);

  /**
   * @brief calculates a cone with at center_angle with length ray_length
   *
   * @param scan the laser scan in which the open cone is searched in
   * @param center_angle the start angle from which the boarder spreads out
   * @param ray_length the length of the boarder to even consider any obstacles
   * @param obs_angle the min angle which has points closer than ray_length
   * to consider as an obstacle (rad)
   * @param min_obs_density the min density in min_obs_angle to consider it as
   *an obstacle
   **/
  static Cone findCone(const sensor_msgs::LaserScan& scan, double center_angle,
                       double ray_length, double obs_angle = 0.0,
                       double min_obs_density = 0.5);
};