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
#include <pcl_assembler/pcl_normal_filter.hpp>

	
	// Method to find clusters on an input point cloud
	void PclNormalFilter::find_clusters(const sensor_msgs::PointCloud2& msg){
		visualization_msgs::Marker cluster_point;
		// Delete all previous markers
		i=7000;
		int marker_array_size=0;
		while(marker_array_size<clusters_points.markers.size()){
			clusters_points.markers[marker_array_size].header.frame_id = "os_sensor";
			clusters_points.markers[marker_array_size].ns = "each_cluster_points";
			clusters_points.markers[marker_array_size].id = i;
			i++;
			clusters_points.markers[marker_array_size].type = visualization_msgs::Marker::POINTS;
			clusters_points.markers[marker_array_size].action = visualization_msgs::Marker::DELETEALL;
			clusters_points.markers[marker_array_size].color.a = 0.0;
			viz_pub.publish(cluster_point);
			marker_array_size++;
		}
	
		ROS_INFO("PclAssemblerServer find_clusters()");

		sensor_msgs::PointCloud2 output_pointcloud;
		start_time = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		sensor_msgs::PointCloud2::Ptr filtered(new sensor_msgs::PointCloud2); //message that will be published
	
		pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(msg, *outputCloud);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*outputCloud, *cloud, indices);

		// Create a search tree, use KDTreee for non-organized data.
		pcl::search::Search<pcl::PointXYZ>::Ptr tree;
		
		if (cloud->isOrganized ()) {
			tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
		} else {
			tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
		}
		// Set the input pointcloud for the search tree
		tree->setInputCloud(cloud);
		
		
		// Compute normals using both small and large scales at each point
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
		ne.setInputCloud(cloud);
		ne.setSearchMethod(tree);
		
		/**
		* NOTE: setting viewpoint is very important, so that we can ensure
		* normals are all pointed in the same direction!
		*/
		ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
		// calculate normals with the small scale
		pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);
		ne.setRadiusSearch (scale1);
		ne.compute (*normals_small_scale);

		// calculate normals with the large scale
		pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

		ne.setRadiusSearch (scale2);
		ne.compute (*normals_large_scale);
		
		// Create output cloud for DoN results
		pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
		pcl::copyPointCloud (*cloud, *doncloud);
		ROS_INFO("DIMENSIONE CLOUD: %d", (int)cloud->size());
		
		// Create DoN operator
		pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
		don.setInputCloud (cloud);
		don.setNormalScaleLarge (normals_large_scale);
		don.setNormalScaleSmall (normals_small_scale);
		// Compute DoN
		don.computeFeature (*doncloud);

/////////////////////////////////// Filters the terrain's points lookin at the normal vector

		// Build the condition for filtering
		pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond ( new pcl::ConditionOr<pcl::PointNormal> ());
		range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
				                   new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
				                 );
		// Build the filter
		pcl::ConditionalRemoval<pcl::PointNormal> condrem;
		condrem.setCondition (range_cond);
		condrem.setInputCloud (doncloud);
		pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

		// Apply filter
		condrem.filter (*doncloud_filtered);
		doncloud = doncloud_filtered;
		pcl::search::KdTree<pcl::PointNormal>::Ptr segtree (new pcl::search::KdTree<pcl::PointNormal>);
		segtree->setInputCloud (doncloud);
/////////////////////////////////////////////////////////////////////////////////////////////

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;

		ec.setClusterTolerance(segradius);
		ec.setMinClusterSize (50);
		ec.setMaxClusterSize (100000);
		ec.setSearchMethod (segtree);
		ec.setInputCloud (doncloud);
		ec.extract (cluster_indices);				// it creates an array in which each element is the set of points indexes that create the cluster

		
		int j = 0;
		float barycenter_x=0;
		float barycenter_y=0;
		float barycenter_z=0;
		int count=0;
		int num_clusters=0;
		pcl::PointCloud<pcl::PointXYZ>::Ptr clusters_msg (new pcl::PointCloud<pcl::PointXYZ>);


		int color=0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end(); ++it, j++)
		{
		
			// Color the point in the cluster
			cluster_point.points.resize(0);
			cluster_point.header.frame_id = "os_sensor";
			cluster_point.ns = "each_cluster_points";
			cluster_point.id = i;
			i++;
			cluster_point.type = visualization_msgs::Marker::POINTS;
			cluster_point.action = visualization_msgs::Marker::ADD;
			cluster_point.scale.x = 0.02;
			cluster_point.scale.y = 0.02;
			cluster_point.scale.z = 0.02;
			cluster_point.color.a = 0.5;
			
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			{
				// pit will be the doncloud pointcloud index of the point that composes a cluster (doncloud is the original that has been calculated again)
				// (*doncloud)[*pit]) is a cluster's point, I can access its x and y and make calculations for the middle point
				if((*doncloud)[*pit].z>-0.6){				
					barycenter_x+=(*doncloud)[*pit].x;
					barycenter_y+=(*doncloud)[*pit].y;
					barycenter_z+=(*doncloud)[*pit].z;
					count++;
				
				
				// Color the point in the cluster
				if (color==0) cluster_point.color.g = 1;	
				else if (color==1) cluster_point.color.b=1;
				else if (color==2) {cluster_point.color.r=0.5; cluster_point.color.g=0.5;}
				else if (color==3){cluster_point.color.b=0.5; cluster_point.color.g=0.5;}
				else if (color == 4) cluster_point.color.r=1;
				geometry_msgs::Point target;
				target.x=((*doncloud)[*pit].x);
				target.y=((*doncloud)[*pit].y);
				target.z=((*doncloud)[*pit].z);
				if (target.x!=0 && target.y!=0 && target.z!=0) cluster_point.points.push_back(target);

				}
			}
			
			// Color the point in the cluster
			clusters_points.markers.push_back(cluster_point);
			array_viz_pub.publish(clusters_points);
			
			color++;
			
			if(barycenter_x!=0 && barycenter_y!=0 && barycenter_z!=0 && count!=0)	{
				// calculate middle point and return it in a pointcloud message that will be put in the action (not published on topic!)
				barycenter_x=barycenter_x/count;
				barycenter_y=barycenter_y/count;
				barycenter_z=barycenter_z/count;
				how_much++;
				ROS_INFO("%d : X: %f Y: %f Z: %f  color: %d", how_much, barycenter_x, barycenter_y, barycenter_z, color-1);
				clusters_msg->points.push_back (pcl::PointXYZ(barycenter_x, barycenter_y, barycenter_z));
				num_clusters++;
				barycenter_x=0;
				barycenter_y=0;
				barycenter_z=0;
				count=0;
			}
		}
		last_i=i;

		clusters_msg->width = clusters_msg->size();
		clusters_msg->height = 1;
		clusters_msg->is_dense = true;
		pcl::toROSMsg(*clusters_msg, *filtered);
		filtered->header = msg.header;
		output_pointcloud=*filtered;
		result.clusters=output_pointcloud; 
		pub_clusters.publish(filtered);			// clusters points that I want to publish

		
		pcl::toROSMsg(*doncloud, *filtered);
		pub_filtered.publish(filtered);			// pointcloud filtered: I want to publish also this one
		
		how_much=0;
	}



	PclNormalFilter::PclNormalFilter(std::string name): as (nh, name, false){
		ros::NodeHandle n;
		ros::param::get("~scale1", scale1);
		ros::param::get("~scale2", scale2);
		ros::param::get("~segradius", segradius);
		ros::param::get("~threshold", threshold);
		ros::param::get("~debug", debug);
		ros::param::get("~lidar_pc_name", lidar_pc_name);


		pub_filtered=n.advertise<sensor_msgs::PointCloud2>("clusters_points_filtered",100);
		pub_clusters=n.advertise<sensor_msgs::PointCloud2>("clusters_points",100);
		viz_pub = n.advertise<visualization_msgs::Marker>("viz", 50);
		array_viz_pub = n.advertise<visualization_msgs::MarkerArray>("viz_array", 50);

		pc_sub=n.subscribe(lidar_pc_name, 1, &PclNormalFilter::pointcloud_sub, this);

		f = boost::bind(&PclNormalFilter::pcl_dyn_rec_cb, this, _1, _2);
		server.setCallback(f);

		as.registerGoalCallback(std::bind(&PclNormalFilter::goalCb, this));
		as.registerPreemptCallback(std::bind(&PclNormalFilter::preemptCb, this));
		as.start();

		pc_count=0;
		i=7000;
		last_i=7000;
		how_much=0;
	}


	//Dynamic Reconfigure method
	void PclNormalFilter::pcl_dyn_rec_cb(pcl_assembler::ClusteringParamConfig &config, uint32_t level) {
		
		this->scale1=config.scale_1;
		this->scale2=config.scale_2;
		this->segradius=config.seg_radius;
		this->threshold=config.threshold_;
		
	}


	// Method invoked when a new goal is received
	void PclNormalFilter::goalCb(){
		goal = *as.acceptNewGoal();
		ROS_INFO("SERVER PclNormalFilter: Got new goal!");
		pc_count=1;
		
		if (goal.start==0) as.setSucceeded();	// goal.start=0 means I have to stop calculating the pcl
		
	}


	// Callback for PointCloud2 messages
	void PclNormalFilter::pointcloud_sub(const sensor_msgs::PointCloud2& msg){
		if (as.isActive()) {				// if the goal has been cancelGoal() then as.isActive()==false

			// pcl_assembler needs 0.06 sec to process a pointcloud, but I receive them every 0.02 sec. I have to filter them
			if(pc_count==5){
				pc_count=0;
				//Perform the action
				find_clusters(msg);
			}
			pc_count++;
		}		
	}



	void PclNormalFilter::preemptCb() {
		as.setPreempted();
		ROS_INFO("PclNormalFilter: Preempt!");
	}


int main(int argc, char** argv) {
	ros::init(argc, argv, "pcl_normal_filter");

	PclNormalFilter pcl_normal_filter(ros::this_node::getName());

	ros::spin();

	return 0;
}
