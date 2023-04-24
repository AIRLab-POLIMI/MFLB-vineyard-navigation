#include <string>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <math.h>
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
#include <pcl_assembler/pcl_euclidean_dist.hpp>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <fstream>


#include "pcl_assembler/GRANSAC.hpp"
#include "pcl_assembler/LineModel.hpp"
#include <cmath>


	template <typename T> int sgn(T val) {
		return (T(0) < val) - (val < T(0));
	}
	
	
	//Method to find clusters in a point cloud using euclidean clustering extraction
	void PclEuclideanFilter::find_clusters(const sensor_msgs::PointCloud2& msg){
		time=ros::WallTime::now();
		
		visualization_msgs::Marker cluster_point, poles_markers; 
		
		sensor_msgs::PointCloud2 output_pointcloud;
		start_time = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		sensor_msgs::PointCloud2::Ptr filtered(new sensor_msgs::PointCloud2); //message that will be published
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(msg, *outputCloud);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*outputCloud, *cloud, indices);		
		
		//input point cloud unordered: use KdTree
		pcl::search::KdTree<pcl::PointXYZ>::Ptr segtree (new pcl::search::KdTree<pcl::PointXYZ>);
		segtree->setInputCloud (cloud);

		// Declare and prepare filter
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

		int min_pc_size=100;
		int max_pc_size=100000;
		ec.setClusterTolerance(segradius);
		ec.setMinClusterSize (min_pc_size);
		ec.setMaxClusterSize (max_pc_size);
		
		if(cloud->size()>max_pc_size || cloud->size()<min_pc_size){	
			ROS_INFO("\nPROBLEM! Pointcloud size: %d does not respect min/max size. \nClustering not performed.", (int) cloud->size());
		}
		ec.setSearchMethod(segtree);
		ec.setInputCloud(cloud);
		
		// Apply filter: it creates an array in which each element is the set of points indexes that create the cluster
		ec.extract(cluster_indices);
		
		ROS_INFO("Founded clusters: %d", (int)cluster_indices.size());

		int j = 0;
		float barycenter_x=0;
		float barycenter_y=0;
		float barycenter_z=0;
		int count=0;
		int num_clusters=0;
		pcl::PointCloud<pcl::PointXYZ>::Ptr clusters_msg (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr poles_msg (new pcl::PointCloud<pcl::PointXYZ>);
		std::vector<double> line;
		std::vector<double> projected_point;
		pcl::PointXYZ actual_point;


		int color=0;
		int nome_file=0;
		i=7000;
		point_id=0;
		model_id=0;
		before_projection_point_id=0;
		tf::StampedTransform point_transform;
		geometry_msgs::TransformStamped point_transform_msg;
		
		clusters_points.markers.resize(0);
		poles_array_markers.markers.resize(0);
		
		point_marker.points.resize(0);   
		point_array_markers.markers.resize(0);
		
		models_array_markers.markers.resize(0);
		
		// Iterate over clusters
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end(); ++it, j++)
		{	
		
			X.resize(0);
			Y.resize(0);
		
			// Get point transform in odom frame
			try {
				point_transform_msg = tf_buffer.lookupTransform("odom", lidar_name, ros::Time(0), ros::Duration(3.0));
			} catch (tf2::TransformException& ex) {
				ROS_ERROR_STREAM("Could not lookup odom-sensor transform: " << ex.what());
			return;
			}
			tf::transformStampedMsgToTF(point_transform_msg, point_transform);


		
		
			// Prepare marker to color the points in the cluster
			cluster_point.points.resize(0);
			cluster_point.header.frame_id = "odom";
			cluster_point.ns = "each_cluster_points";
			cluster_point.id = i;
			i++;
			cluster_point.type = visualization_msgs::Marker::POINTS;
			cluster_point.action = visualization_msgs::Marker::ADD;
			cluster_point.scale.x = 0.02;
			cluster_point.scale.y = 0.02;
			cluster_point.scale.z = 0.02;
			cluster_point.color.a = 0.5;
			cluster_point.lifetime=ros::Duration(5.0);
			
			// create a pointcloud that will contain the current cluster
			pcl::PointCloud<pcl::PointXYZ>::Ptr single_cluster (new pcl::PointCloud<pcl::PointXYZ>);

			single_cluster->width = it->indices.size();
			single_cluster->height = 1;
			single_cluster->points.resize (single_cluster->width * single_cluster->height);			
			int position=0;
			
			
			
			// Iterates over points in a cluster
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			{
				// update barycenter data				
				if((*cloud)[*pit].z>-0.6){				
					barycenter_x+=(*cloud)[*pit].x;
					barycenter_y+=(*cloud)[*pit].y;
					barycenter_z+=(*cloud)[*pit].z;
					count++;
					
					// save the point in the cluster's pointcloud
					(*single_cluster).points[position].x = (*cloud)[*pit].x;
					(*single_cluster).points[position].y = (*cloud)[*pit].y;
					(*single_cluster).points[position].z = (*cloud)[*pit].z; 
					
					// store x and y coordinates in vectors to perform Linear Regression or RANSAC
					X.push_back((*cloud)[*pit].x);
					Y.push_back((*cloud)[*pit].y);
					position++;
					
					// Color the points in the cluster
					if (color==0) {cluster_point.color.r=0.0;  cluster_point.color.g = 1.0;	cluster_point.color.b=0.0;} 		// green
					else if (color==1) {cluster_point.color.r=0.0;  cluster_point.color.g = 0.0;	cluster_point.color.b=1.0;} 	// blue
					else if (color==2) {cluster_point.color.r=0.5; cluster_point.color.g=1.0; cluster_point.color.b=1.0;} 		// light blue
					else if (color==3){cluster_point.color.r=1.0; cluster_point.color.b=0.0; cluster_point.color.g=0.7;} 		// orange
					else if (color == 4) {cluster_point.color.r=0.7; cluster_point.color.g=0.7; cluster_point.color.b=0.0;} 	// yellow
					else if(color == 5) {cluster_point.color.r=1.0;  cluster_point.color.g = 0.0;	cluster_point.color.b=1.0;}	// fucsia
					else {cluster_point.color.r=1.0;  cluster_point.color.g = 0.0;	cluster_point.color.b=0.0;}

					
					tf::Point target;
					target.setX((*cloud)[*pit].x);
					target.setY((*cloud)[*pit].y);
					target.setZ((*cloud)[*pit].z);
					tf::Point target_point_in_odom=point_transform*target;
					geometry_msgs::Point trg;
					tf::pointTFToMsg(target_point_in_odom, trg);
					if (trg.x!=0 && trg.y!=0 && trg.z!=0) cluster_point.points.push_back(trg);
				}
				
			}
			
			// Store points in the marker and prepare for next cluster
			clusters_points.markers.push_back(cluster_point);
			color++;
			
			
			// calculate barycenter of the current cluster and return it in a pointcloud message that will be put in the action (not published on topic!)
			if(barycenter_x!=0 && barycenter_y!=0 && barycenter_z!=0 && count!=0)	{
				barycenter_x=barycenter_x/count;
				barycenter_y=barycenter_y/count;
				barycenter_z=barycenter_z/count;
				how_much++;
				clusters_msg->points.push_back (pcl::PointXYZ(barycenter_x, barycenter_y, barycenter_z));
				num_clusters++;
				barycenter_x=0;
				barycenter_y=0;
				barycenter_z=0;
				count=0;
			}
			
			//Search for candidate pole point in the current cluster
			
			// For each cluster, I have to create a KdTree (a cluster is a non-organized pointcloud) and I need to find on the tree the nearest point to the robot.
			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

			kdtree.setInputCloud(single_cluster);

			pcl::PointXYZ searchPoint;	// I need the nearest points to (0, 0, 0)

			searchPoint.x = 0.0;
			searchPoint.y = 0.0;
			searchPoint.z = 0.0;
			
			// K nearest neighbor search. 
			
			if(point_neighborhood_check==false){
				// If point_neighborhood_check is not selected, return only the nearest point: K=1
				K=1;
			}

			std::vector<int> pointIdxKNNSearch(K);
			std::vector<float> pointKNNSquaredDistance(K);
			// pointIdxKNNSearch is associated to pointKNNSquaredDistance and they are both ordered by distance from searchPoint. 
			// pointIdxKNNSearch[0] is the nearest point to searchPoint so the first point that satisfies the condition is also the neareast one. 
			
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;



			if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
			{	
				if(point_neighborhood_check==true){
				
					// I also have to check the neighborhood of each point to control that the selected point has other points all around it.
					for (std::size_t i = 0; i < pointIdxKNNSearch.size (); ++i){
					
						// for each neighbor I have to check if there are points in a radius of 'radius' (parameter) around it
						
						// Neighbors within radius search
						if ( kdtree.radiusSearch ((*single_cluster)[ pointIdxKNNSearch[i] ], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
						{
							if(pointIdxRadiusSearch.size()>min_near_points){
								// good candidate! I can consider it as a pole
								// Then I have to put the selected point in the poles pointcloud that will be returned in the action.
								actual_point=pcl::PointXYZ((*single_cluster)[ pointIdxKNNSearch[i]].x, (*single_cluster)[ pointIdxKNNSearch[i]].y, (*single_cluster)[ pointIdxKNNSearch[i]].z);
								break; 
							}
						}
						
						if(i==(pointIdxKNNSearch.size()-1)){
							// if I am here, I have checked the K nearest points whitout selecting a good candidate: return the nearest point
							actual_point=pcl::PointXYZ((*single_cluster)[ pointIdxKNNSearch[0]].x, (*single_cluster)[ pointIdxKNNSearch[0]].y, (*single_cluster)[ pointIdxKNNSearch[0]].z);
							ROS_INFO("\033[1;31m ERROR! PclEuclideanClustering is not able to find the pole point. Nearest one is used! \033[0m");
						}
					}
				}
				else{
					// I don't need to check the neighborhood so I have a single point in pointIdxKNNSearch which is the nearest one.					
					actual_point=pcl::PointXYZ((*single_cluster)[ pointIdxKNNSearch[0]].x, (*single_cluster)[ pointIdxKNNSearch[0]].y, (*single_cluster)[ pointIdxKNNSearch[0]].z);
				}
			}	
			// If option is selected, I need to project the selected point into the linear regression model of the cluster
			if(use_row_model==true){
				line.resize(0);

				//LINEAR REGRESSION WITH LEAST SQUARE (use_row_model variable contains the value of use_LR_model parameter)
				
				// whatever way the candidate pole point has been calculated, I have to project it on the model
				line=least_square.getRegressionLine(X, Y);	// [0]:m    [1]:q 
				if(isnan(line[1])==false){
					// model is in the form x=my+q
					projected_point=least_square.getPointOnModel(actual_point, line[0], line[1]);
					draw_point(actual_point);	// draw selected point before project it into the model
					actual_point=pcl::PointXYZ(projected_point[0], projected_point[1], 0.0);   
					draw_regular_model(line[0], line[1], actual_point);
				}

				// END LINEAR REGRESSION
			}
			else if(use_RANSAC_model==true){
			
				//RANSAC
				line.resize(0);
				line=calculate_RANSAC_model();
				obj3<<"RANSAC --> m:"<<line[0]<<", q:"<<line[1]<<". ";
				obj3<<"MSE:"<<calculate_MSE(line)<<"\n";
				
				// whatever way the candidate pole point has been calculated, I have to project it on the model
				projected_point=least_square.getPointOnModel(actual_point, line[0], line[1]);		// re-cycled method :)
				draw_point(actual_point);	// draw selected point before project it into the model
				actual_point=pcl::PointXYZ(projected_point[0], projected_point[1], 0.0);   
				draw_regular_model(line[0], line[1], actual_point);
				//END RANSAC
				
				// For data-analysis only: calculate both ransac and linear regression and print their performances on txt file.
				std::vector<double> regression_version_line=least_square.getRegressionLine(X, Y);
				obj3<<"linear regr. --> m:"<<regression_version_line[0]<<", q:"<<regression_version_line[1]<<". ";
				obj3<<"MSE:"<<calculate_MSE(regression_version_line)<<"\n";
				
			}
			
			poles_msg->points.push_back(actual_point);
		}
		array_viz_pub.publish(clusters_points);
		
		
		
		// Publish results
		
		clusters_msg->width = clusters_msg->size();
		clusters_msg->height = 1;
		clusters_msg->is_dense = true;
		pcl::toROSMsg(*clusters_msg, *filtered);
		filtered->header = msg.header;
		output_pointcloud=*filtered;
		pub_clusters.publish(filtered);			// clusters points that I want to publish (middle point of the cluster)
		
		poles_msg->width=poles_msg->size();
		poles_msg->height=1;
		poles_msg->is_dense=true;
		pcl::toROSMsg(*poles_msg, *filtered);
		filtered->header=msg.header;
		result.clusters=*filtered;
		pub_poles_points.publish(filtered);


		// Color the choosen nearest points in the clusters
		
		try {
			point_transform_msg = tf_buffer.lookupTransform("odom", lidar_name, ros::Time(0), ros::Duration(3.0));
		} catch (tf2::TransformException& ex) {
			ROS_ERROR_STREAM("Could not lookup odom-sensor transform: " << ex.what());
		return;
		}
		tf::transformStampedMsgToTF(point_transform_msg, point_transform);
		
		poles_markers.points.resize(0);
		poles_markers.header.frame_id = "odom";
		poles_markers.ns = "poles_points";
		poles_markers.type = visualization_msgs::Marker::POINTS;
		poles_markers.action = visualization_msgs::Marker::ADD;
		poles_markers.scale.x = 0.05;
		poles_markers.scale.y = 0.05;
		poles_markers.scale.z = 0.05;
		poles_markers.color.a = 1.0;
		poles_markers.color.r = 1;
		poles_markers.lifetime=ros::Duration(3.0);
		tf::Point target;
		for(pcl::PointCloud<pcl::PointXYZ>::const_iterator item = ((*poles_msg).points).begin(); item != ((*poles_msg).points).end(); item++){
			poles_markers.id = point_id;
			point_id++;
			target.setX((*item).x);
			target.setY((*item).y);
			target.setZ((*item).z);
			tf::Point target_point_in_odom=point_transform*target;
			geometry_msgs::Point trg;
			tf::pointTFToMsg(target_point_in_odom, trg);
			poles_markers.points.push_back(trg);
		}
		
		
		poles_array_markers.markers.push_back(poles_markers);
		poles_array_viz_pub.publish(poles_array_markers);
		

		
		
		how_much=0;
		
		// For data-analysis only: print on file the needed times to perform clustering.
		if(use_row_model==false && use_RANSAC_model==false) obj1<<(ros::WallTime::now()-time).toSec()<<"\n";	// without model
		else obj2<<(ros::WallTime::now()-time).toSec()<<"\n";	//with model
	}
	
	
	// Calculate MSE to evaluate linear regression and RANSAC models.
	double PclEuclideanFilter::calculate_MSE(std::vector<double> line){
		double MSE=0;
		
		for(int j=0; j<X.size(); j++){
			std::vector<double> proj_point=least_square.getPointOnModel(pcl::PointXYZ(X[j], Y[j], 0.0), line[0], line[1]);
			MSE+=std::pow(proj_point[0] - X[j], 2)+std::pow(proj_point[1] - Y[j], 2);
		}
		MSE=MSE/X.size();
		
		return MSE;
	}
	
	
	// Method to calculate RANSAC model using X and Y coordinate vectors
	std::vector<double> PclEuclideanFilter::calculate_RANSAC_model(){
		
		std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
		
		for(int i=0; i<X.size(); i++){
			std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared<Point2D>(X[i], Y[i]);
			CandPoints.push_back(CandPt);
		}
		
		GRANSAC::RANSAC<Line2DModel, 2> Estimator;
		Estimator.Initialize(threshold, iterations); 			// threshold and iterations set in input by dynamic reconfigure
		Estimator.Estimate(CandPoints);
		
		auto BestLine = Estimator.GetBestModel();
		
		std::vector<double> params=BestLine->getParams();   // [0]:a [1]:b [2]:c		ax+by+c=0
		
		
		std::vector<double> RANSAC_line;
		RANSAC_line.push_back(-(params[1]/params[0]));   		// m   case x=my+q
		RANSAC_line.push_back(-(params[2]/params[0]));		// q   case x=my+q
		
		//RANSAC_line.push_back(-(params[0]/params[1]));   	//m   case y=mx+q
		//RANSAC_line.push_back(-(params[2]/params[1]));	//q   case y=mx+q
		
		return RANSAC_line;
		
	}


	//Method to draw row models with linear regression (case x=my+q)	
	void PclEuclideanFilter::draw_regular_model(double m, double q, pcl::PointXYZ point){

		tf::Point target_point1;
		tf::Point target_point2;
		tf::Point target_point3;
		double mp, qp, x_intersection, y_intersection;
		
		// take input point as starting point
		target_point1.setX(point.x);	
		target_point1.setY(point.y);
		target_point1.setZ(0.0);

		target_point2.setY(5);		// y independent variable. choose 5 as value
		target_point2.setX(5*m+q);	// x = my + q
		target_point2.setZ(0.0);
		
		target_point3.setY(-5);	// y independent variable. choose -5 as value
		target_point3.setX((-5)*m+q);	// x = my + q
		target_point3.setZ(0.0);


//		target_point2.setX(5);		// x independent variable
//		target_point2.setY(5*m+q);	// y = mx + q
//		target_point2.setZ(0.0);

//		target_point3.setX(-5);	// x independent variable
//		target_point3.setY((-5)*m+q);	// y = mx + q
//		target_point3.setZ(0.0);

		draw(target_point1, target_point2, target_point3);
		

		
	}


	// Given 3 points, it draws a line between them (independent from linear regression or RANSAC)
	void PclEuclideanFilter::draw(tf::Point target_point1, tf::Point target_point2, tf::Point target_point3){
	
		// get point transform in odom frame 
		marker.points.resize(0);
		tf::Point target_point_in_odom;
		geometry_msgs::Point target;
		geometry_msgs::TransformStamped point_transform_msg;
		try {
			point_transform_msg = tf_buffer.lookupTransform("odom", lidar_name, ros::Time(0), ros::Duration(3.0));
		} catch (tf2::TransformException& ex) {
			ROS_ERROR_STREAM("Could not lookup odom-sensor transform: " << ex.what());
			return;
		}
		tf::StampedTransform point_transform;
		tf::transformStampedMsgToTF(point_transform_msg, point_transform);
		
	
		// draw the segment between the points
		marker.header.frame_id = "odom";
		marker.ns = "model_line";
		marker.id = model_id;
		model_id++;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = visualization_msgs::Marker::ADD;
		marker.scale.x = 0.05;
		marker.color.g = 1;		
		marker.color.a = 0.5;
		marker.lifetime=ros::Duration(5.0);
		
		target_point_in_odom=point_transform*target_point1;
		tf::pointTFToMsg(target_point_in_odom, target);
		marker.points.push_back(target); 	// selected pole point 
		
		target_point_in_odom=point_transform*target_point2;
		tf::pointTFToMsg(target_point_in_odom, target);
		marker.points.push_back(target);		// second point (farther away)
		
		target_point_in_odom=point_transform*target_point3;
		tf::pointTFToMsg(target_point_in_odom, target);
		marker.points.push_back(target);		// third point (closer)
		
		models_array_markers.markers.push_back(marker);
		row_models_viz_pub.publish(models_array_markers);
	}
	
	
	// A function to draw a single point
	void PclEuclideanFilter::draw_point(pcl::PointXYZ target_point){
		// get point transform in odom frame 
		tf::Point point;
		point.setX(target_point.x);
		point.setY(target_point.y);
		point.setZ(target_point.z);

		tf::Point target_point_in_odom;
		geometry_msgs::Point target;
		geometry_msgs::TransformStamped point_transform_msg;
		try {
			point_transform_msg = tf_buffer.lookupTransform("odom", lidar_name, ros::Time(0), ros::Duration(3.0));
		} catch (tf2::TransformException& ex) {
			ROS_ERROR_STREAM("Could not lookup odom-sensor transform: " << ex.what());
			return;
		}
		tf::StampedTransform point_transform;
		tf::transformStampedMsgToTF(point_transform_msg, point_transform);
		
	
		
		point_marker.header.frame_id = "odom";
		point_marker.ns = "before_projection_points";
		point_marker.id = before_projection_point_id;
		before_projection_point_id++;
		point_marker.type = visualization_msgs::Marker::POINTS;
		point_marker.action = visualization_msgs::Marker::ADD;
		point_marker.scale.x = 0.05;
		point_marker.scale.y = 0.05;
		point_marker.scale.z = 0.05;
		point_marker.color.r = 1;		
		point_marker.color.a = 1.0;
		point_marker.lifetime=ros::Duration(5.0);
		
		target_point_in_odom=point_transform*point;
		tf::pointTFToMsg(target_point_in_odom, target);
		point_marker.points.push_back(target); 	// selected pole point 
		
		
		point_array_markers.markers.push_back(point_marker);
		row_models_viz_pub.publish(point_array_markers);
	}



	PclEuclideanFilter::PclEuclideanFilter(std::string name): as (nh, name, false), tf_listener(tf_buffer){
		ros::NodeHandle n;
		ros::param::get("~segradius", segradius);
		ros::param::get("~debug", debug);
		ros::param::get("~lidar_pc_name", lidar_pc_name);
		ros::param::get("~lidar_name", lidar_name);
		ros::param::get("~storage", storage);

		pub_filtered=n.advertise<sensor_msgs::PointCloud2>("clusters_points_filtered",100);
		pub_clusters=n.advertise<sensor_msgs::PointCloud2>("clusters_points",100);
		viz_pub = n.advertise<visualization_msgs::Marker>("viz", 50);
		array_viz_pub = n.advertise<visualization_msgs::MarkerArray>("viz_array", 50);
		poles_array_viz_pub=n.advertise<visualization_msgs::MarkerArray>("viz_poles_array", 50);
		row_models_viz_pub=n.advertise<visualization_msgs::MarkerArray>("row_models_array", 50);
		pub_poles_points=n.advertise<sensor_msgs::PointCloud2>("founded_poles_points", 50);

		pc_sub=n.subscribe(lidar_pc_name, 1, &PclEuclideanFilter::pointcloud_sub, this);

		f = boost::bind(&PclEuclideanFilter::pcl_dyn_rec_cb, this, _1, _2);
		server.setCallback(f);

		as.registerGoalCallback(std::bind(&PclEuclideanFilter::goalCb, this));
		as.registerPreemptCallback(std::bind(&PclEuclideanFilter::preemptCb, this));
		as.start();
		
		obj1.open(storage+"/cpu_times/file_pcl_no_model.csv", std::ofstream::trunc);
		obj2.open(storage+"/cpu_times/file_pcl_model.csv", std::ofstream::trunc);
		obj3.open(storage+"/rows_models/RANSACvsLINREG.txt", std::ofstream::trunc);

		pc_count=0;
		i=7000;
		k=6000;
		how_much=0;
		model_id=0;
		point_id=0;
		before_projection_point_id=0;

	}

	// Dynamic reconfigure
	void PclEuclideanFilter::pcl_dyn_rec_cb(pcl_assembler::ClusteringEuclideanParamConfig &config, uint32_t level) {
		this->segradius=config.seg_radius;		// segradius is the dimension of the cluster that will be searched
		this->point_neighborhood_check=config.point_neighborhood_check;
		this->radius=config.r;
		this->min_near_points=config.min_near_points;
		this->K=config.K;
		this->use_row_model=config.use_LR_model;
		this->use_RANSAC_model=config.use_RANSAC_model;
		this->threshold=config.threshold;		//RANSAC only
		this->iterations=config.iterations;		//RANSAC only
	}


	// Method invoked when server receives a new goal
	void PclEuclideanFilter::goalCb(){
		goal = *as.acceptNewGoal();
		ROS_INFO("SERVER PclEuclideanFilter: Got new goal!");
		pc_count=1;
		
		if (goal.start==0) as.setSucceeded();	// goal.start=0 means I have to stop calculating the pcl
	}

	//Callback for PointCloud2 messages. It filters them to have output at 2 Hz.
	void PclEuclideanFilter::pointcloud_sub(const sensor_msgs::PointCloud2& msg){
		if (as.isActive()) {				// if the goal has been canceled then as.isActive()==false

			// pcl_assembler needs 0.05 sec to process a point cloud, but I receive them every 0.02 sec. I have to filter them
			if(pc_count==5){
				pc_count=0;
				//Perform the action
				find_clusters(msg);
			}
			pc_count++;
		}		
	}



	void PclEuclideanFilter::preemptCb() {
		as.setPreempted();
		ROS_INFO("PclEuclideanFilter: Preempt!");
	}


int main(int argc, char** argv) {
	ros::init(argc, argv, "pcl_euclidean_dist");

	PclEuclideanFilter pcl_euclidean_filter(ros::this_node::getName());

	ros::spin();

	return 0;
}

