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
#include <pcl_assembler/ClusteringParamConfig.h>
#include <pcl_assembler/PclAssemblerAction.h>
#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//Oldo node used for DoN based clustering. No more used.

class PclNormalFilter{

	private:
	
	ros::NodeHandle nh;
	ros::Publisher pub_clusters;
	ros::Publisher pub_filtered;
	ros::Publisher viz_pub;
	ros::Publisher array_viz_pub;
	ros::Subscriber pc_sub;
	std::string topic;
	ros::Time start_time;
	bool debug;
	int pc_count;
	std::string lidar_pc_name;
	std::string lidar_name;
	int i, last_i;
	int how_much;
	visualization_msgs::MarkerArray clusters_points;

	sensor_msgs::PointCloud2 input_pointcloud;

	dynamic_reconfigure::Server<pcl_assembler::ClusteringParamConfig> server;
	dynamic_reconfigure::Server<pcl_assembler::ClusteringParamConfig>::CallbackType f;

	actionlib::SimpleActionServer<pcl_assembler::PclAssemblerAction> as;
	pcl_assembler::PclAssemblerGoal goal;
	pcl_assembler::PclAssemblerResult result;
	
	///The smallest scale to use in the DoN filter.
	double scale1;

	///The largest scale to use in the DoN filter.
	double scale2;


	///segment scene into clusters with given distance tolerance using euclidean clustering
	double segradius;

	///The minimum DoN magnitude to threshold by
	double threshold;


	public:

	PclNormalFilter(std::string name);

	void find_clusters(const sensor_msgs::PointCloud2& msg);

	void pcl_dyn_rec_cb(pcl_assembler::ClusteringParamConfig &config, uint32_t level);

	void goalCb();
	void preemptCb();
	void pointcloud_sub(const sensor_msgs::PointCloud2& msg);

};
