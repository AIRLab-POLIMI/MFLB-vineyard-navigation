#include <cmath>
#include "pc_filters/filters.hpp"
#include "ros/ros.h"
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
#include <algorithm>
#include "std_msgs/String.h"
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>



		Filters::Filters(){
	
		}

		// Filters the PointCloud that are out of a specified radius
		sensor_msgs::PointCloud2 Filters::radius_filtering(const sensor_msgs::PointCloud2& point_cloud, double radius){
			pcl::PCLPointCloud2 pcl_pc2;
			pcl::PCLPointCloud2 pcl_pc2_out;
			pcl::PointCloud<pcl::PointXYZ>  projected;
			pcl_conversions::toPCL (point_cloud,pcl_pc2);
			pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

			 // build the condition
			pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
			range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, radius)));
			range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, -radius)));
			range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, radius))); 
			range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, -radius))); 

			// GT means greather than, LT means less than

			// build the filter
			pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
			condrem.setCondition (range_cond);
			condrem.setInputCloud (temp_cloud);
			condrem.setKeepOrganized (true);

			// apply the filter
			condrem.filter (*temp_cloud);

			// conversion to pointcloud2 and then to ROS
			pcl::toPCLPointCloud2(*temp_cloud, pcl_pc2_out);


			sensor_msgs::PointCloud2 output;
			pcl_conversions::fromPCL(pcl_pc2_out, output);
			output.header = point_cloud.header;
			output.header.stamp = ros::Time::now();

			return output;
		}


		// Filters the PointCloud in some zones (removes on Z and on Y)
		sensor_msgs::PointCloud2 Filters::zone_filtering(int side_to_remove, double lateral_dist, double points_height, double points_height_start, const sensor_msgs::PointCloud2& input_cloud){
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);

			pcl::fromROSMsg(input_cloud, *input);

			// Create the filtering object for height filtering
			pcl::PassThrough<pcl::PointXYZ> pass1;
			pass1.setInputCloud(input);
			pass1.setFilterFieldName ("z");
			pass1.setFilterLimits(points_height_start, points_height);			// points into this range will be mantained 
			// NB: THE CUT IS MADE CONSIDERING AS (0, 0, 0) THE SENSOR, SO TO KEEP ALSO WHAT IS UNDER THE SENSOR YOU HAVE TO PUT A NEGATIVE VALUE IN points_height_start
			pass1.filter(*cloud_filtered1);

			// Create the filtering object for side filtering
			pcl::PassThrough<pcl::PointXYZ> pass2;
			pass2.setInputCloud (cloud_filtered1);
			pass2.setFilterFieldName ("y");
			if(side_to_remove==1) pass2.setFilterLimits (-120.0, lateral_dist);			// remove points on robot right side
			else if(side_to_remove==-1) pass2.setFilterLimits (-lateral_dist, 120);		// remove points on robot left side
			pass2.filter(*cloud_filtered2);

			sensor_msgs::PointCloud2 filtered;
			pcl::toROSMsg(*cloud_filtered2, filtered);
		
			return filtered;

		}
		
		
		//Downsamples the PointCloud
		sensor_msgs::PointCloud2 Filters::downsampling_filtering(double downsampling_cube, const sensor_msgs::PointCloud2& input_cloud){
		
			pcl::PCLPointCloud2::Ptr pcl_pc2 (new pcl::PCLPointCloud2 ());
			pcl_conversions::toPCL (input_cloud, *pcl_pc2);
			
			pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
			
			// Create the filtering object
			pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
			sor.setInputCloud(pcl_pc2);
			sor.setLeafSize (downsampling_cube, downsampling_cube, downsampling_cube);	// uses cube of side downsampling_cube to downsampling filtering
			sor.filter (*cloud_filtered);
			
			sensor_msgs::PointCloud2 filtered;
			pcl_conversions::fromPCL(*cloud_filtered, filtered);
			filtered.header=input_cloud.header;
			filtered.header.stamp=ros::Time::now();
			
			return filtered;
			
			
			/*VoxelGrid creates cubes of dimension downsampling_cube meters and it reduces all the points in them to a single point*/
		}
		
		// Filters the outliers points w.r.t. mean distances from neighbors
		sensor_msgs::PointCloud2 Filters::outliers_filtering(const sensor_msgs::PointCloud2& point_cloud, int meanK, double stddevMulThresh){
		
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg(point_cloud, *input);
			
			// Create the filtering object
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			sor.setInputCloud(input);
			sor.setMeanK(meanK);						// more points I look, more points I can remove
			sor.setStddevMulThresh (stddevMulThresh);			// bigger is the multiplier, more are the points that respect the condition
			sor.filter (*cloud_filtered);
			
			sensor_msgs::PointCloud2 filtered;
			pcl::toROSMsg(*cloud_filtered, filtered);

			
			return filtered;
		}
		


