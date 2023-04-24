#include <cmath>
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
#include "messages/filtering_instr.h"
#include <dynamic_reconfigure/server.h>
#include <navigation/FilteringParamConfig.h>


#include "pc_filters/filters.hpp"

class Filter_node{

	private:
		ros::NodeHandle nh;
		ros::Subscriber filter_instr;
		ros::Subscriber pc_sub;
		ros::Publisher pc_pub;
		Filters filter;

		dynamic_reconfigure::Server<navigation::FilteringParamConfig> server;
		dynamic_reconfigure::Server<navigation::FilteringParamConfig>::CallbackType f;

		std::string sensor_topic_name;
		sensor_msgs::PointCloud2 pc_input, pc_output;
		double radius, points_height, points_height_start, lateral_dist, downsampling_cube, meanK, stddevMulThresh;
		bool radius_filtering, zone_filtering, downsampling_filtering, outliers_filtering;
		int side_to_remove;
		
		ros::WallTime time;
		std::ofstream obj1;
		std::string storage;


	public:
		Filter_node(){

			nh.getParam("/filter_node/sensor_topic_name", sensor_topic_name);
			nh.getParam("/filter_node/storage", storage);


			filter_instr=nh.subscribe("filtering_instr", 50, &Filter_node::filter_instr_callback, this);
			pc_sub=nh.subscribe(sensor_topic_name+"/points", 50, &Filter_node::pc_sub_callback, this);		
			pc_pub=nh.advertise<sensor_msgs::PointCloud2>(sensor_topic_name+"/points_filtered", 1);

			f = boost::bind(&Filter_node::dynamicParamCb, this, _1, _2);
			server.setCallback(f);
			
			// By default only the radius filtering is applied
			radius_filtering=true;
			zone_filtering=false;
			downsampling_filtering=true;
			outliers_filtering=false;
			
			obj1.open(storage+"/cpu_times/file_filter_node.csv", std::ofstream::trunc);

		}

		// Callback for the dynamic reconfigure
		void dynamicParamCb(const navigation::FilteringParamConfig& config, uint32_t level) {
			this->radius=config.r;
		  	this->points_height = config.points_height;
		  	this->points_height_start=config.points_height_start;
			this->lateral_dist= config.lateral_dist;
			this->downsampling_cube=config.downsampling_cube;
			this->stddevMulThresh=config.stddevMulThresh;
			this->meanK=config.meanK;
			this->outliers_filtering=config.outliers_filtering;
		}

		// Callback for the filtering instruction (which filters need to be applied)
		void filter_instr_callback(const messages::filtering_instr::ConstPtr& msg){
			radius_filtering=msg->radius_filtering;
			zone_filtering=msg->zone_filtering;
			downsampling_filtering=msg->downsampling_filtering;
			side_to_remove=msg->side_to_remove;	
		}

		// Applies the requested filters to the PointCloud
		void pc_sub_callback(const sensor_msgs::PointCloud2& msg){
			time=ros::WallTime::now();
			if(radius_filtering==true) {
				pc_output=filter.radius_filtering(msg, radius);
				pc_input=pc_output;
			}
			if(zone_filtering==true){
				pc_output=filter.zone_filtering(side_to_remove, lateral_dist, points_height, points_height_start, pc_input);		
				pc_input=pc_output;
			}
			if(downsampling_filtering==true){
				pc_output=filter.downsampling_filtering(downsampling_cube, pc_input);
				pc_input=pc_output;
			}
			if(outliers_filtering==true){
				pc_output=filter.outliers_filtering(pc_input, meanK, stddevMulThresh);
			}

			pc_pub.publish(pc_output);
			obj1<<(ros::WallTime::now()-time).toSec()<<"\n";
		}

};

int main(int argc, char **argv){ 
	ros::init(argc, argv, "filter_node");
	ros::NodeHandle n;

	Filter_node filter_node;
	
	ros::spin();
	return 0;
}
