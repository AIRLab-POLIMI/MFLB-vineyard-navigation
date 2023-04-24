#include <string>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_assembler/ClusteringEuclideanParamConfig.h>
#include <pcl_assembler/PclAssemblerAction.h>
#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "least_square.hpp"
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <fstream>


#include "GRANSAC.hpp"
#include "LineModel.hpp"
#include <cmath>



class PclEuclideanFilter{

	private:
	
	ros::NodeHandle nh;
	ros::Publisher pub_clusters;
	ros::Publisher pub_filtered;
	ros::Publisher pub_poles_points;
	ros::Publisher viz_pub;
	ros::Publisher model_pub;
	ros::Publisher array_viz_pub;
	ros::Publisher poles_array_viz_pub;
	ros::Publisher row_models_viz_pub;
	ros::Subscriber pc_sub;
	std::string topic;
	ros::Time start_time;
	bool debug;
	int pc_count;
	std::string lidar_pc_name;
	std::string lidar_name;
	int i, k;
	int K;
	double radius;
	int min_near_points;
	int how_much;
	bool point_neighborhood_check;
	int iterations;
	double threshold;
	visualization_msgs::MarkerArray clusters_points;
	visualization_msgs::MarkerArray poles_array_markers;
	visualization_msgs::MarkerArray models_array_markers;
	visualization_msgs::MarkerArray point_array_markers;
	visualization_msgs::Marker marker;
	visualization_msgs::Marker point_marker;
	sensor_msgs::PointCloud2 input_pointcloud;

	dynamic_reconfigure::Server<pcl_assembler::ClusteringEuclideanParamConfig> server;
	dynamic_reconfigure::Server<pcl_assembler::ClusteringEuclideanParamConfig>::CallbackType f;

	actionlib::SimpleActionServer<pcl_assembler::PclAssemblerAction> as;
	pcl_assembler::PclAssemblerGoal goal;
	pcl_assembler::PclAssemblerResult result;
	
	std::vector<double> X;
	std::vector<double> Y;
	bool use_row_model, use_RANSAC_model;
	int model_id, point_id, before_projection_point_id;
	
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
	
	ros::WallTime time;
	std::ofstream obj1, obj2, obj3;
	
	Least_square least_square;
	
	std::vector<double> principal_component_vector;
	std::string storage;


	///segment scene into clusters with given distance tolerance using euclidean clustering
	double segradius;




	public:

	PclEuclideanFilter(std::string name);

	void find_clusters(const sensor_msgs::PointCloud2& msg);

	void pcl_dyn_rec_cb(pcl_assembler::ClusteringEuclideanParamConfig &config, uint32_t level);

	void goalCb();
	void preemptCb();
	void pointcloud_sub(const sensor_msgs::PointCloud2& msg);
	
	void draw_regular_model(double m, double q, pcl::PointXYZ point);
	void draw_parallel_model(double k, pcl::PointXYZ point);
	void draw(tf::Point target_point1, tf::Point target_point2, tf::Point target_point3);
	void draw_point(pcl::PointXYZ target_point);
	std::vector<double> calculate_RANSAC_model();
	double calculate_MSE(std::vector<double> line);
};
