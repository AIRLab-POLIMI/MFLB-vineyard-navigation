#ifndef PC_FILTERS
#define PC_FILTERS

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <array>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <algorithm>
#include "std_msgs/String.h"

class Filters {

	public:

		Filters();
		sensor_msgs::PointCloud2 radius_filtering(const sensor_msgs::PointCloud2& point_cloud, double radius);
		sensor_msgs::PointCloud2 zone_filtering(int side_to_remove, double lateral_dist, double points_height, double points_height_start, const sensor_msgs::PointCloud2& input_cloud);
		sensor_msgs::PointCloud2 downsampling_filtering(double downsampling_cube, const sensor_msgs::PointCloud2& input_cloud);
		sensor_msgs::PointCloud2 outliers_filtering(const sensor_msgs::PointCloud2& point_cloud, int meanK, double stddevMulThresh);



	private:

		ros::NodeHandle n;

		std::string sensor_topic_name;


};
#endif 
