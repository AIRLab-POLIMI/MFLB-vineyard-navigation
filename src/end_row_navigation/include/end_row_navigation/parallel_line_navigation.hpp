
#include <algorithm>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <array>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <end_row_navigation/ParallelLineNavigationAction.h>
#include <fre_row_navigation/pid.h>
#include <fre_row_navigation/rectangle.h>
#include <end_row_navigation/EndRowParamConfig.h>
#include "messages/vineyard_info.h"
#include <pcl_assembler/client.hpp>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "messages/controller.h"
#include <bits/stdc++.h>
#include <fstream>

class ParallelLineNav{

	
	private:
		ros::NodeHandle n;
		ros::NodeHandle nh;
		ros::Subscriber pointcloud2_sub;
		ros::Subscriber clusters_sub;
		ros::Subscriber cloud_front_sub;
		ros::Subscriber odom_sub;
		ros::Subscriber vineyard_info_sub;
		ros::Subscriber controller_sub;
		ros::Subscriber barycenter_sub;
		ros::Publisher pub_speeds;
		ros::Publisher viz_pub;
		PID pidController; 

		actionlib::SimpleActionServer<end_row_navigation::ParallelLineNavigationAction> as;
		end_row_navigation::ParallelLineNavigationGoal goal;

		dynamic_reconfigure::Server<end_row_navigation::EndRowParamConfig> server;
		dynamic_reconfigure::Server<end_row_navigation::EndRowParamConfig>::CallbackType f;

		PclAssemblerClient pcl_assembler_client;

		bool got_initial_pos;
		int marker_id;
		double segm_ang_coeff;
		float max_reachable_speed;
		float min_travel_length;
		double speed;
		float start_travelled_distance;
		float travelled_distance;
		std::string lidar_sensor;
		std::string lidar_topic_name;
		std::string odom_topic;
		std::string pcl_assembler_server_name;
		geometry_msgs::Point last_pos;
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudFront;
		float speed_low;
		float speed_high;
		float p_gain;
		float i_gain;
		float d_gain;
		bool use_speed_control;
		bool start;
		double driveAngle;
		double oldDriveAngle;
		double correction_angle;
		double row_dim;
		double corridor_width;
		int corridors_to_skip;
		int input_corridors_to_skip;
		double correction_speed;
		std::vector<geometry_msgs::Point> points;
		std::vector<geometry_msgs::Point> points_odom; // same as points, but in odom frame
		std::vector<geometry_msgs::Point> old_points_odom;
		double middle_dist;
		int action_flag;
		int check_angle_err;
		double start_angle, actual_angle;
		double angle_err;
		geometry_msgs::Twist stopCommand;
		std::vector<geometry_msgs::Point> clusters;
		std::vector<geometry_msgs::Point> clusters_odom;
		int aligned_with_segm;
		int start_align_last_segm;
		bool clusters_in_sensor_frame;
		double xm, ym, zm;			// coordinates of the last segment's middle point
		double xm_odom, ym_odom, zm_odom;	// coordinates of the last segment's middle point in odom frame
		bool first_calc_middle_point;
		bool use_barycenter_as_point;
		std::vector<geometry_msgs::Point> to_send_clusters;
		geometry_msgs::Point last_pole;
		bool first_save_last_pole;
		double last_segm_ang_coeff;
		bool reaching_middle_point;
		double done_angle;
		double middle_point;
		int count_driveCommandEmerg;
		bool use_manual_corridors_to_skip;
		
		tf2_ros::Buffer tf_buffer;
		tf2_ros::TransformListener tf_listener;
		
		ros::WallTime time;
		std::ofstream obj1, obj2, debug;
		
		std::vector<geometry_msgs::Point> poles_odom;		// stores the identified poles over the path
		std::vector<geometry_msgs::Point> poles_odom_last_version;
		std::vector<geometry_msgs::Point> candidate_poles_odom; // stores the new ipotetical poles to check
		std::vector<geometry_msgs::Point> candidate_poles_sensor;
		
		
		std::string storage;
		
		int poles_count;
		double pole_radius;
		
		bool start_work;

	public:
		ParallelLineNav(std::string name);
		void goalCb();
		void preemptCb();
		void processClusters(std::vector<geometry_msgs::Point> scan);
		std::vector<geometry_msgs::Point> find_two_points(std::vector<geometry_msgs::Point> scan);
		void pointcloud_sub(const sensor_msgs::PointCloud2& msg);
		void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
		void cloudCb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
		void dynamicParamCb(const end_row_navigation::EndRowParamConfig& config, uint32_t level);
		void vineyard_infoCb(const messages::vineyard_info::ConstPtr& msg);
		void new_corridor_entrance(geometry_msgs::Point p1, geometry_msgs::Point p2);
		void clustersCb(const sensor_msgs::PointCloud2& msg);
		void barycentersCb(const sensor_msgs::PointCloud2& msg);
		int align_last_segm(double segm_ang_coeff);
		void draw_middle_point();
		void draw_cluster_points();
		void draw_cluster_segm();
		void convert_in_odom_frame(std::vector<geometry_msgs::Point> array, int who);
		void convert_in_sensor_frame(std::vector<geometry_msgs::Point> array, int who);
		void calc_middle_point(geometry_msgs::Point p1, geometry_msgs::Point p2);
		void count_poles();
		void sub_controller_callback(const messages::controller::ConstPtr& msg);
		void store_points(const sensor_msgs::PointCloud2& msg);
		bool check_last_two_poles();
		bool same_pole(geometry_msgs::Point candidate, geometry_msgs::Point pole);
		bool last_poles(geometry_msgs::Point p1, geometry_msgs::Point p2);
};
