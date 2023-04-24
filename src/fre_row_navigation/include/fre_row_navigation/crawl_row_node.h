#pragma once

#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <fre_row_navigation/CrawlRowAction.h>
#include <fre_row_navigation/CrawlRowParamConfig.h>
#include <fre_row_navigation/cone.h>
#include <fre_row_navigation/pid.h>
#include <fre_row_navigation/rectangle.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_listener.h>
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "messages/vineyard_info.h"
#include "messages/controller.h"
#include "std_msgs/Float32.h"
class CrawlRowAction {
	public:
		CrawlRowAction();

		void goalCb();
		void preemptCb();

		void lidarCb(const sensor_msgs::LaserScan::ConstPtr& msg);
		void cloudCb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
		void odomCb(const nav_msgs::Odometry::ConstPtr& msg);

		/**
		* @brief call called when updating dynamic params
		**/
		void dynamicParamCb(const fre_row_navigation::CrawlRowParamConfig& config,uint32_t level);
		void vineyard_infoCb(const messages::vineyard_info::ConstPtr& msg);
		
		void sub_controller_callback(const messages::controller::ConstPtr& msg);

	protected:
		void processLidar(const sensor_msgs::LaserScan& scan);

		ros::NodeHandle nh;
		ros::NodeHandle ph;
		actionlib::SimpleActionServer<fre_row_navigation::CrawlRowAction> as;

		ros::Subscriber lidar_front_sub;
		ros::Subscriber cloud_front_sub;
		ros::Subscriber odom_sub;
		ros::Subscriber vineyard_info_sub;
		ros::Subscriber controller_sub;
		ros::Publisher viz_pub;
		ros::Publisher array_viz_pub;
		ros::Publisher drive_pub;
		ros::Publisher p_gain_pub;
		ros::Publisher driveAngle_pub;

		tf::TransformBroadcaster odom_broadcaster;

		std::string odom_topic;
		std::string lidar_sensor;

		dynamic_reconfigure::Server<fre_row_navigation::CrawlRowParamConfig> dyn_param_server;

		tf2_ros::Buffer tf_buffer;
		tf2_ros::TransformListener tf_listener;

		visualization_msgs::MarkerArray targets;

		pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudFront;
		fre_row_navigation::CrawlRowGoal goal;
		geometry_msgs::Point last_pos;
		bool got_initial_pos;

		int end_line_frame_count;
		float speed;
		float start_travelled_distance;
		float travelled_distance;
		double sensor_min_range;
		float max_reachable_speed;
		float min_row_length;
		double start_end_row;
		double end_line_meters_threshold;
		bool finishing_the_row;
		int done_rows;
		int todo_rows;
		double distance_obstacle;
		bool slow_down_obstacle;


		float speed_low;
		float speed_high;
		float p_gain;
		float i_gain;
		float d_gain;

		double count;

		int work;
		ros::WallTime time;
		std::ofstream obj1, obj2, obj3, obj4, debug, pid_output;
		ros::Time start_turning_time, end_turning_time;
		
		std::string storage;
		
		bool start_work;

		/**
		* @brief the pid controller for the steering angle
		**/

		PID pidController;
		ros::Time start_time;

		// Params
		double ray_length;
		double corridor_width;
		double max_lateral_rect_dist;
		double row_dim;
		bool use_speed_control;
		int end_line_frame_count_threshold;

		double driveAngle;
		double oldDriveAngle;

		/**
		* @brief if true, use three cones and select the best one
		**/
		bool multi_cone_enabled;

		/**
		* @brief minimal cone width in degrees
		*
		* If the cone width is smaller than this, then that cone is discarded
		**/
		double min_cone_width;

		/**
		* @brief maximal cone width in degrees
		*
		* If the best cones width is larger than this, drive straight
		**/
		double max_cone_width;

		/**
		* @brief the angle to consider when judging scan points as an obstacle (deg)
		**/
		double obstacle_angle;

		/**
		* @brief min density of the points in obstacle_angle to consider it as an
		*obstacle
		**/
		double min_obs_density;
};
