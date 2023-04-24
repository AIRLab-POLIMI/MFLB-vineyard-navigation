#include <end_row_navigation/parallel_line_navigation.hpp>
#include <algorithm>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/String.h"
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
#include <end_row_navigation/EndRowParamConfig.h>
#include "messages/vineyard_info.h"
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "messages/controller.h"
#include <bits/stdc++.h>
#include <fstream>


	template <typename V, typename S>
	V lerp(V a, V b, S t) {
		return a + t * (b - a);
	}
	// lerp returns a value between a and b which depends on t. t can be only between 0 and 1.
	// if t=0, returns a; if t=1, returns b;

	float ramp_up(float begin, float later, float t) {
	  if (t > 1.0) {
	    return later;
	  } else if (t < 0.0) {
	    return begin;
	  } else {
	    return later * t + begin * (1 - t);
	  }
	}

	ParallelLineNav::ParallelLineNav(std::string name):
	n("~"),
	tf_listener(tf_buffer),
	pidController(0.0, 0.0, 0.0),
	as (nh, name, false){

		n.getParam("lidar_sensor", lidar_sensor);
		n.getParam("lidar_topic_name", lidar_topic_name);
		n.getParam("odom_topic", odom_topic);
		n.getParam("pcl_assembler_server_name", pcl_assembler_server_name);
		n.getParam("storage", storage);
		cloud_front_sub = nh.subscribe(lidar_topic_name+"/points_filtered", 1, &ParallelLineNav::cloudCb, this);
		clusters_sub = nh.subscribe("founded_poles_points", 1, &ParallelLineNav::clustersCb, this);
		barycenter_sub =nh.subscribe("clusters_points", 1, &ParallelLineNav::barycentersCb, this);
		pointcloud2_sub=nh.subscribe(lidar_topic_name+"/points_filtered", 1, &ParallelLineNav::pointcloud_sub, this);
		odom_sub = nh.subscribe(odom_topic, 1, &ParallelLineNav::odomCb, this);
		vineyard_info_sub=nh.subscribe("vineyard_info", 1, &ParallelLineNav::vineyard_infoCb, this);
		controller_sub=nh.subscribe("controller", 50, &ParallelLineNav::sub_controller_callback, this);
		pub_speeds = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);			// topic on which the robot is listening for speed commands
		viz_pub = n.advertise<visualization_msgs::Marker>("viz", 50);


		as.registerGoalCallback(std::bind(&ParallelLineNav::goalCb, this));
		as.registerPreemptCallback(std::bind(&ParallelLineNav::preemptCb, this));
		as.start();

		f = boost::bind(&ParallelLineNav::dynamicParamCb, this, _1, _2);
		server.setCallback(f);

		start_work=false;
		marker_id=0;
		travelled_distance=0;
		driveAngle=0;
		oldDriveAngle=0;
		action_flag=0;
		check_angle_err=0;
		aligned_with_segm=0;
		start_align_last_segm=0;
		reaching_middle_point=false;
		middle_point=false;
		poles_count=0;
		done_angle=0;
		count_driveCommandEmerg=0;
		poles_odom.resize(0);
		poles_odom_last_version.resize(0);
		clusters_odom.resize(0);
		obj1.open(storage+"/cpu_times/file_parallel.csv", std::ofstream::trunc);
		obj2.open(storage+"/poles_distance_errors/robot_position_end_row.txt", std::ofstream::trunc);

	}


// Method invoked when a new goal is received by the ActionServer
	void ParallelLineNav::goalCb(){
		goal = *as.acceptNewGoal();
		// Before starting again I have to clear all the old values
		if(goal.corridors_to_skip==-1) {
			corridors_to_skip=input_corridors_to_skip;	// value taken from dynamic reconfigure
		}
		else{
			corridors_to_skip=goal.corridors_to_skip;	// value taken from goal
		}
		marker_id=0;
		travelled_distance=0;
		start_travelled_distance = travelled_distance;
		driveAngle=0;
		oldDriveAngle=0;
		got_initial_pos = true;
		action_flag=0;
		check_angle_err=0;
		middle_dist=0;
		points.resize(0);
		aligned_with_segm=0;
		start_align_last_segm=0;
		reaching_middle_point=false;
		middle_point=false;
		done_angle=0;
		count_driveCommandEmerg=0;
		clusters.resize(0);
		clusters_odom.resize(0);
		points.resize(0);
		points_odom.resize(0);
		old_points_odom.resize(0);
		to_send_clusters.resize(0);
		poles_count=0;
		poles_odom.resize(0);
		poles_odom_last_version.resize(0);
		pidController.reset();
		
		geometry_msgs::Point p;
		p.x=-69999;
		p.y=-69999;
		old_points_odom.push_back(p);
		old_points_odom.push_back(p);

		

		// When this action starts, it needs the clusters from pcl_assembler: send goal to ActionServer to activate it.
		pcl_assembler_client.get_clusters(pcl_assembler_server_name, 1);		

		max_reachable_speed=goal.speed;
		min_travel_length=goal.min_travel_length;


	}

	void ParallelLineNav::preemptCb() {
		as.setPreempted();
		pub_speeds.publish(geometry_msgs::Twist());  // Stop
		ROS_INFO("ParallelLineNavigation: Preempt!");
	}


// Save received PointCloud
	void ParallelLineNav::cloudCb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg) {
		cloudFront = msg;
	}


// Execute the action every time a new PointCloud is received
	void ParallelLineNav::pointcloud_sub(const sensor_msgs::PointCloud2& msg){
		time=ros::WallTime::now();
		if (as.isActive()) {
				convert_in_sensor_frame(clusters_odom, 0);
				processClusters(clusters);
				obj1<<(ros::WallTime::now()-time).toSec()<<"\n";
		}
	}
	

	// Method to store the candidate pole points in each cluster (published by clustering ActionServer)
	void ParallelLineNav::clustersCb(const sensor_msgs::PointCloud2& msg){
		if(use_barycenter_as_point==false){
			store_points(msg);
		}
	}
	
	// Method to store the barycenters of each cluster (published by clustering ActionServer)
	void ParallelLineNav::barycentersCb(const sensor_msgs::PointCloud2& msg){
		if(use_barycenter_as_point==true){
			store_points(msg);
		}
	}
	
	// Method to store the points that will be used to calculate the segment
	void ParallelLineNav::store_points(const sensor_msgs::PointCloud2& msg){
		sensor_msgs::PointCloud2 received_clusters;
		received_clusters=msg;
		
		// store the points of the pointcloud in an array
		pcl::PointCloud<pcl::PointXYZ> _vscan;			
		pcl::fromROSMsg(received_clusters, _vscan);
		
		to_send_clusters.resize(0);
		
		for(pcl::PointCloud<pcl::PointXYZ>::const_iterator item = _vscan.begin(); item != _vscan.end(); item++){
			geometry_msgs::Point p;
			p.x=item->x;
			p.y=item->y;
			p.z=item->z;
			to_send_clusters.push_back(p);
		}

		// I want to convert the points in odom frame here. Then I will have to convert in base_link before use them
		convert_in_odom_frame(to_send_clusters, 1);
	}
	
	
	// Converts the points into the input array in odom frame
	void ParallelLineNav::convert_in_odom_frame(std::vector<geometry_msgs::Point> array, int who){  
	//who == 1 : I am converting clusters points, who == 0 : I am converting two points,  who==2: I am converting two selected points in ipotetical poles
	
		if (who == 1) {	
			clusters_odom.resize(0);
		}
		else if(who == 0){
			points_odom.resize(0);
		}
		else if(who==2){
			candidate_poles_odom.resize(0);
		}
				
		// Get point transform in odom frame
		geometry_msgs::TransformStamped point_transform_msg;
		try {
			point_transform_msg = tf_buffer.lookupTransform("odom", lidar_sensor, ros::Time(0), ros::Duration(3.0));
		} catch (tf2::TransformException& ex) {
			ROS_ERROR_STREAM("Could not lookup odom-sensor transform: " << ex.what());
			return;
		}
		tf::StampedTransform point_transform;
		tf::transformStampedMsgToTF(point_transform_msg, point_transform);
		
		for(int i=0; i<array.size(); i++){
			tf::Point target_point;
			target_point.setX(array[i].x);
			target_point.setY(array[i].y);
			target_point.setZ(array[i].z);
			tf::Point target_point_in_odom=point_transform*target_point;
			geometry_msgs::Point target;
			tf::pointTFToMsg(target_point_in_odom, target);
			if(who == 1) clusters_odom.push_back(target);
			else if(who == 0) points_odom.push_back(target);
			else if (who == 2) candidate_poles_odom.push_back(target);
		}
	}
	
	
	// Converts the points into the input array in current sensor frame
	void ParallelLineNav::convert_in_sensor_frame(std::vector<geometry_msgs::Point> array, int who){ // who=0: clusters   who=1: points	who==2: candidate_poles
		
		std::vector<geometry_msgs::Point> support_clusters;
		support_clusters.resize(0);	
		
		// Get point transform in sensor frame
		geometry_msgs::TransformStamped point_transform_msg;
		try {
			point_transform_msg = tf_buffer.lookupTransform(lidar_sensor, "odom", ros::Time(0), ros::Duration(3.0));
		} catch (tf2::TransformException& ex) {
			ROS_ERROR_STREAM("Could not lookup odom-sensor transform: " << ex.what());
			return;
		}
		tf::StampedTransform point_transform;
		tf::transformStampedMsgToTF(point_transform_msg, point_transform);
		
		for(int i=0; i<array.size(); i++){
			tf::Point target_point;
			target_point.setX(array[i].x);
			target_point.setY(array[i].y);
			target_point.setZ(array[i].z);
			tf::Point target_point_in_odom=point_transform*target_point;
			geometry_msgs::Point target;
			tf::pointTFToMsg(target_point_in_odom, target);
			if(who==0) support_clusters.push_back(target);
			else if (who==1) points.push_back(target);
			else if (who==2) candidate_poles_sensor.push_back(target);
		}
		
		if(who==0 ){
			clusters.resize(0);
			clusters=support_clusters;
		}
	}



// Method that calculates the two nearest points to the robot and the segment between them. Returns cmd_vel commands.
	void ParallelLineNav::processClusters(std::vector<geometry_msgs::Point> scan) {
	
		if(start==1){				// do not publish speeds if the robot is stopped
		
			points.resize(0);
			points=find_two_points(scan);   // points[0]=back point, points[1]=front point 
			
			// here I have to store a copy of the two points in odom frame to convert them in sensor frame if needed
			convert_in_odom_frame(points, 0);  // now points_odom=points but in odom frame
			
			
			
			if(points.size()>=2 && (points[0].x!=0 || points[0].y!=0) && (points[1].x!=0 || points[1].y!=0)){
				
				// Draw the two points selected
				draw_cluster_points();
				
				// Draw the segment between the two points
				draw_cluster_segm();

				// Print info on file
				if(same_pole(points_odom[0], old_points_odom[1])==true){
					obj2<<points_odom[0].x<<", "<<points_odom[0].y<<"\n";
				}
				
				
				// Check the selected points to see if they identify a new pole
				count_poles();
			
				// slope of the segment between this two points
				segm_ang_coeff=(points[1].y-points[0].y)/(points[1].x-points[0].x);
				
				
				// if segm_ang_coeff==0, segment and x axis are parallel;
				// if segm_ang_coeff>0, Scout needs to turn left
				// if segm_ang_coeff<0, Scout needs to turn right 



				// Speed control
				if(use_speed_control==true && reaching_middle_point==false){
					float sreclenmin = 0.3;
					float sreclenmax = 2.0;
					float sx1 = sreclenmin;
					float sx2 = sreclenmax;
					float growth = 0.05;
					
					Rectangle rect_drive_dir = Rectangle::grow_x(sx1, 0.2, -0.2, growth, sx2, *cloudFront, 10);
					visualization_msgs::Marker marker = rect_drive_dir.createMarker(1287, "base_link");
					marker.color.b = 1.0;
					marker.color.a = 0.5;
					viz_pub.publish(marker);
					float speed_rect_len = std::abs(rect_drive_dir.x2 - rect_drive_dir.x1);

					float calc_speed =lerp(this->speed_low, this->speed_high, (speed_rect_len - sreclenmin) / (sreclenmax - sreclenmin));
					
					// smooth
					speed = speed * 0.90 + calc_speed * 0.1;
					if(max_reachable_speed<speed){
						speed=max_reachable_speed;
					}
				} else if(reaching_middle_point==false){
					if(max_reachable_speed>speed_low) 
						speed = speed_low;
					else 
						speed=max_reachable_speed;
				}
				// if reaching_middle_point=true, speed is determined by new_corridor_entrance() method


				pidController.p =ramp_up(p_gain * 2, p_gain, (travelled_distance - start_travelled_distance) / 0.2);
				pidController.i = i_gain;
				pidController.d = d_gain;

				if(isnan(segm_ang_coeff)==0){
					// m = tan of the angle that the line creates with x axis => arctan(m) is the angle formed by the line
					angle_err = std::atan(segm_ang_coeff);
					

					check_angle_err=1;
					start_angle=actual_angle;


					
					if (start_work==true) driveAngle = pidController.calculate(angle_err);	// takes as input the needed correction angle
					

					// correction if drive angle is too small
					if(std::abs(driveAngle)<0.1){
						if(driveAngle>0) driveAngle=0.1;
						else driveAngle=-0.1;
					}


					if(poles_odom.size()>1 && last_poles(points_odom[0], points_odom[1])==false){
						// the new segment has a big different slope: make the robot goes straight for a while before turning	
						ROS_INFO("\033[1;31m \n\n Rotation has caused a change in poles view. Waiting to see last poles again. \n\n \033[0m");
						geometry_msgs::Twist driveCommandEmerg;
						driveCommandEmerg.linear.x = speed;
						driveCommandEmerg.angular.z = 0.0;
						pub_speeds.publish(driveCommandEmerg);	
						count_driveCommandEmerg++;
						
					}
					else{
						geometry_msgs::Twist driveCommand;
						driveCommand.linear.x = speed;
						driveCommand.angular.z = driveAngle;
						pub_speeds.publish(driveCommand);
						count_driveCommandEmerg=0;
					}
				}
				
				if(poles_count==3+corridors_to_skip){
					// Last pole detected: align with middle point
					reaching_middle_point=true;
					points.resize(0);
					convert_in_sensor_frame(points_odom, 1);  // now points = points_odom converted in sensor frame
					new_corridor_entrance(points[0], points[1]);
				}
				
				old_points_odom=points_odom;	
			}
			
			// I need to perform this check regardless if the clustering ActionServer is sending empty points

			if(reaching_middle_point==true){
				if(middle_point==true && std::abs(xm)<=0.1){
					// Robot is near the middle point of the segment 
					ROS_INFO("ParallelLineNav server has reached the middle point of the segment. Stopping..");
					stopCommand.linear.x = 0.0;
					stopCommand.angular.z = 0.0;
					pub_speeds.publish(stopCommand);

					// Print info on file
					obj2<<poles_odom_last_version[poles_odom_last_version.size()-1].x<<", "<<poles_odom_last_version[poles_odom_last_version.size()-1].y<<"\n";

					as.setSucceeded();
					pcl_assembler_client.get_clusters(pcl_assembler_server_name, 0); // when the clustering ActionServer receives another goal, it stops		
				}
			}
			
				
		}
		else
			ROS_INFO("End Line Navigation: check 'start' to start the navigation.");
			
	
	}
		

	//It checks if one of the points in candidate_poles_odom is a new pole to count and store it in poles_odom
	void ParallelLineNav::count_poles(){
		bool candidate=false;
		if(candidate_poles_odom.size()>0){
			if(poles_odom.size()>0){
				// double for: for each point of both I have to check if they are in the radius of the point
				for(int i=0; i<candidate_poles_odom.size(); i++){
					for(int j=0; j<poles_odom.size(); j++){
						if(candidate_poles_odom[i].x>(poles_odom[j].x-pole_radius) && candidate_poles_odom[i].x<(poles_odom[j].x+pole_radius) &&
						     candidate_poles_odom[i].y>(poles_odom[j].y-pole_radius) && candidate_poles_odom[i].y<(poles_odom[j].y+pole_radius)){
						     	// the points belongs to an already present pole
						     	candidate=false;
						     	poles_odom_last_version[j]=candidate_poles_odom[i];
						     	break;
						} 
						else{
							// the pole is far away from any other pole
							candidate=true;
						}
					}
					
					if(candidate==true){
						//it means I have found a new pole that is not in the array yet: store it
					     	poles_count++;
					     	poles_odom.push_back(candidate_poles_odom[i]);
					     	poles_odom_last_version.push_back(candidate_poles_odom[i]);
					     	//ROS_INFO("ParallelLineNav - counted poles: %d", poles_count);
					     	candidate=false;
					}
				}
			}
			else{
				// if we are here, no poles have already been stored in poles_odom array
				convert_in_sensor_frame(candidate_poles_odom, 2);  // now candidate_poles_sensor contains candidate_poles_odom but in sensor frame
				if(candidate_poles_sensor[0].x<0){
					poles_odom.push_back(candidate_poles_odom[0]);
					poles_odom_last_version.push_back(candidate_poles_odom[0]);
					candidate_poles_odom.erase(candidate_poles_odom.begin());
				}
				else{
					poles_odom.push_back(candidate_poles_odom[1]);
					poles_odom_last_version.push_back(candidate_poles_odom[1]);
					candidate_poles_odom.erase(candidate_poles_odom.end());
				}
				poles_count++;	
				
				obj2<<poles_odom[0].x<<", "<<poles_odom[0].y<<"\n";		
			}
			
		}
	}



	// Finds the two cluster's points that are one in front of the robot and one in the back of the robot.
	std::vector<geometry_msgs::Point> ParallelLineNav::find_two_points(std::vector<geometry_msgs::Point> scan){

		geometry_msgs::Point back_point;
		geometry_msgs::Point front_point;
		double front_dist=65535;
		double back_dist=65535;


		// Take the two nearest points to the robot
		for (int i=0; i<scan.size(); i++){
			if(scan[i].x>0){
				if(i==0 || std::sqrt(std::pow(scan[i].x, 2) + std::pow(scan[i].y, 2))<front_dist){
					front_point.x=scan[i].x;
					front_point.y=scan[i].y;
					front_point.z=scan[i].z;
					front_dist=std::sqrt(std::pow(scan[i].x, 2) + std::pow(scan[i].y, 2));
				}
			}
			else{
				if(i==0 || std::sqrt(std::pow(scan[i].x, 2) + std::pow(scan[i].y, 2))<back_dist){
					back_point.x=scan[i].x;
					back_point.y=scan[i].y;
					back_point.z=scan[i].z;
					back_dist=std::sqrt(std::pow(scan[i].x, 2) + std::pow(scan[i].y, 2));
				}
			}
		}


		std::vector<geometry_msgs::Point> maxmin;
		maxmin.push_back(back_point);
		maxmin.push_back(front_point);

		// Save the selected points also in odom_frame for counting procedure
		convert_in_odom_frame(maxmin, 2);			// do not remove! here I am converting points for the poles countings (not clusters)

		return maxmin;
	}



	// Place the robot at the entrance of a new corridor (aligned with middle point)
	void ParallelLineNav::new_corridor_entrance(geometry_msgs::Point p1, geometry_msgs::Point p2){			
		
		ROS_INFO("End Line Navigation: reaching the middle point of the last segment...");
					
		

		if(last_poles(points_odom[0], points_odom[1])==true){

			// Search the middle point of the segment 
			calc_middle_point(p1, p2);
			middle_point=true;

			geometry_msgs::Twist correctionCommand;

			// Robot has to proceeds till xm is approximately 0
			if(std::abs(xm)>0.10){
				//go straight on with fixed speed
				
				if(xm>0) speed = correction_speed;
				else  speed = -correction_speed;
				
				correctionCommand.linear.x =speed;
				correctionCommand.angular.z=driveAngle;
				pub_speeds.publish(correctionCommand);						
			}
		}
		else{
			// The considered pole is not the last one, ignore it and proceeds straight on (caused by high slope difference between subsequent segments)
			geometry_msgs::Twist driveCommandEmerg;
			driveCommandEmerg.linear.x = speed;
			driveCommandEmerg.angular.z = 0.0;
			pub_speeds.publish(driveCommandEmerg);	
			middle_point=false;
		}
		
	} 
	

	// Check if the two poles are the last two or not
	bool ParallelLineNav::last_poles(geometry_msgs::Point p1, geometry_msgs::Point p2){
	
		if(poles_odom.size()<2){
			return true;
		}
		
		if(((p1.x<(poles_odom[(poles_odom.size()-1)].x+pole_radius) && p1.x>(poles_odom[(poles_odom.size()-1)].x-pole_radius)) &&
			(p1.y<(poles_odom[(poles_odom.size()-1)].y+pole_radius) && p1.y>(poles_odom[(poles_odom.size()-1)].y-pole_radius))) || 	
			((p1.x<(poles_odom[(poles_odom.size()-2)].x+pole_radius) && p1.x>(poles_odom[(poles_odom.size()-2)].x-pole_radius)) &&
			(p1.y<(poles_odom[(poles_odom.size()-2)].y+pole_radius) && p1.y>(poles_odom[(poles_odom.size()-2)].y-pole_radius)))){
			
				if(((p2.x<(poles_odom[(poles_odom.size()-1)].x+pole_radius) && p2.x>(poles_odom[(poles_odom.size()-1)].x-pole_radius)) && 
					(p2.y<(poles_odom[(poles_odom.size()-1)].y+pole_radius) && p2.y>(poles_odom[(poles_odom.size()-1)].y-pole_radius))) ||
					((p2.x<(poles_odom[(poles_odom.size()-2)].x+pole_radius) && p2.x>(poles_odom[(poles_odom.size()-2)].x-pole_radius)) && 
					(p2.y<(poles_odom[(poles_odom.size()-2)].y+pole_radius) && p2.y>(poles_odom[(poles_odom.size()-2)].y-pole_radius)))){
					
						return true;
					
					}
			
			}
			
			return false;	
	}
	
	//Check if two points represent the same pole (their distance is less than pole_radius)
	bool ParallelLineNav::same_pole(geometry_msgs::Point candidate, geometry_msgs::Point pole){
	
		if((candidate.x<(pole.x+pole_radius)) && (candidate.x>(pole.x-pole_radius)) && 
			(candidate.y<(pole.y+pole_radius)) && (candidate.y>(pole.y-pole_radius))){
				return true;
		}
		else{
			return false;
		}
	}


	//Calculate the middle point between p1 and p2 and manages the frame conversions
	void ParallelLineNav::calc_middle_point(geometry_msgs::Point p1, geometry_msgs::Point p2){
	
		// First time: I calculate the point
		xm=(p2.x+p1.x)/2;
		ym=(p2.y+p1.y)/2;
		zm=(p2.z+p1.z)/2;
		
		//Convert in odom and save that value
		geometry_msgs::TransformStamped point_transform_msg;
		try {
			point_transform_msg = tf_buffer.lookupTransform("odom", lidar_sensor, ros::Time(0), ros::Duration(3.0));
		} catch (tf2::TransformException& ex) {
			ROS_ERROR_STREAM("Could not lookup odom-sensor transform: " << ex.what());
			return;
		}
		tf::StampedTransform point_transform;
		tf::transformStampedMsgToTF(point_transform_msg, point_transform);			
		
		tf::Point target_point;
		target_point.setX(xm);
		target_point.setY(ym);
		target_point.setZ(zm);
		tf::Point target_point_in_odom=point_transform*target_point;
		geometry_msgs::Point target;
		tf::pointTFToMsg(target_point_in_odom, target);
		
		xm_odom=target.x;
		ym_odom=target.y;
		zm_odom=target.z;
		
		draw_middle_point();
	
	}
	
	// This method draws the middle point of the segment
	void ParallelLineNav::draw_middle_point(){


		visualization_msgs::Marker middle_point;
		middle_point.header.frame_id = "odom";
		middle_point.ns = "middle_point";
		middle_point.id = 3;
		middle_point.type = visualization_msgs::Marker::POINTS;
		middle_point.action = visualization_msgs::Marker::ADD;
		middle_point.scale.x = 0.2;
		middle_point.scale.y = 0.2;
		middle_point.scale.z = 0.2;
		middle_point.color.b = 1;			
		middle_point.color.a = 1;
		middle_point.lifetime = ros::Duration(5.0);
		geometry_msgs::Point target_point;
		target_point.x=xm_odom;
		target_point.y=ym_odom;
		target_point.z=zm_odom;
		middle_point.points.push_back(target_point);
		
		viz_pub.publish(middle_point);
		
	}
	
	
	// This method draws the two selected clusters points
	void ParallelLineNav::draw_cluster_points(){
		// Get point transform in odom frame
		geometry_msgs::TransformStamped point_transform_msg;
		try {
			point_transform_msg = tf_buffer.lookupTransform("odom", lidar_sensor, ros::Time(0), ros::Duration(3.0));
		} catch (tf2::TransformException& ex) {
			ROS_ERROR_STREAM("Could not lookup odom-sensor transform: " << ex.what());
			return;
		}
		tf::StampedTransform point_transform;
		tf::transformStampedMsgToTF(point_transform_msg, point_transform);

		// highlight the found points
		visualization_msgs::Marker cluster_point;
		cluster_point.points.resize(0);
		for(int i=0; i<points.size(); i++){
			cluster_point.header.frame_id = "odom";
			cluster_point.ns = "cluster_points";
			cluster_point.id = i;
			cluster_point.type = visualization_msgs::Marker::POINTS;
			cluster_point.action = visualization_msgs::Marker::ADD;
			cluster_point.scale.x = 0.15;
			cluster_point.scale.y = 0.15;
			cluster_point.scale.z = 0.15;
			cluster_point.color.r = 1;			
			cluster_point.color.a = 1;
			cluster_point.lifetime = ros::Duration(5.0);
			tf::Point target_point;
			target_point.setX(points[i].x);
			target_point.setY(points[i].y);
			target_point.setZ(points[i].z);
			tf::Point target_point_in_odom=point_transform*target_point;
			geometry_msgs::Point target;
			tf::pointTFToMsg(target_point_in_odom, target);
			if (points[i].x!=0 && points[i].y!=0 && points[i].z!=0) cluster_point.points.push_back(target);
		}
		if (cluster_point.points.size()!=0) viz_pub.publish(cluster_point);

		
		geometry_msgs::TransformStamped point_transform_msg2;
		try {
			point_transform_msg2 = tf_buffer.lookupTransform(lidar_sensor, "odom", ros::Time(0), ros::Duration(3.0));
		} catch (tf2::TransformException& ex) {
			ROS_ERROR_STREAM("Could not lookup odom-sensor transform: " << ex.what());
			return;
		}
		tf::StampedTransform point_transform2;
		tf::transformStampedMsgToTF(point_transform_msg2, point_transform2);
		std::vector<geometry_msgs::Point> array=points_odom;
		for(int i=0; i<array.size(); i++){
			tf::Point target_point2;
			target_point2.setX(array[i].x);
			target_point2.setY(array[i].y);
			target_point2.setZ(array[i].z);
			tf::Point target_point_in_odom2=point_transform2*target_point2;
			geometry_msgs::Point target;
			tf::pointTFToMsg(target_point_in_odom2, target);
		}
		
	}


	// This method draws the segment between the choosen cluster points
	void ParallelLineNav::draw_cluster_segm(){
	
	
		// draw the segment between the two points
		visualization_msgs::Marker marker;
		marker.header.frame_id = "odom";
		marker.ns = "cluster_segment";
		marker.id = 2;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = visualization_msgs::Marker::ADD;
		marker.scale.x = 0.05;
		marker.color.r = 1;		
		marker.color.a = 1.0;
		marker.lifetime=ros::Duration(5.0);
		geometry_msgs::Point target_point1;
		target_point1.x=points_odom[0].x;		// first vertex
		target_point1.y=points_odom[0].y;
		target_point1.z=points_odom[0].z;
		if (points[0].x!=0 && points[0].y!=0 && points[0].z!=0) marker.points.push_back(target_point1);
		geometry_msgs::Point target_point2;
		target_point2.x=points_odom[1].x;		// second vertex
		target_point2.y=points_odom[1].y;
		target_point2.z=points_odom[1].z;
		if (points[1].x!=0 && points[1].y!=0 && points[1].z!=0) marker.points.push_back(target_point2);
		if (marker.points.size()==2) viz_pub.publish(marker);
	}


	// Takes odometry info and calculates the travelled distance
	void ParallelLineNav::odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
		if (got_initial_pos) {
			double dx = last_pos.x - msg->pose.pose.position.x;
			double dy = last_pos.y - msg->pose.pose.position.y;
			double d = std::sqrt(dx * dx + dy * dy);
			travelled_distance += d;
		}
		else{
			start_travelled_distance=travelled_distance;		
		}
		last_pos = msg->pose.pose.position;
		got_initial_pos = true;

		double roll, pitch, yaw;
		tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

		if(yaw>=0){				
			actual_angle=yaw;
		}
		else{
			actual_angle=6.28+yaw;
			// the angles between 180 -- 360 are returned by getRPY as -3.14 -- 0			
		}

		//Check if we have rotated enough to align with the current segment
		if(check_angle_err==1){
			// I have to control to not exceed the angle_err

			done_angle=std::min(std::abs(actual_angle-start_angle), std::abs(actual_angle-(start_angle-6.28319)));
			if(done_angle>=std::abs(angle_err)){
				// Robot is aligned with the segment, proceeds with linear speed only
				stopCommand.linear.x = speed;
				stopCommand.angular.z = 0.0;
				pub_speeds.publish(stopCommand);
				driveAngle=0;
				check_angle_err=0;
				if(action_flag==1) aligned_with_segm=1;
			}
		}
		
	}



	// Dynamic reconfigure method
	void ParallelLineNav::dynamicParamCb(const end_row_navigation::EndRowParamConfig& config, uint32_t level) {
	  	this->use_speed_control = config.use_speed_control;
		this->p_gain = config.p_gain;
		this->i_gain = config.i_gain;
		this->d_gain = config.d_gain;
		this->speed_low = config.speed_low;
		this->speed_high = config.speed_high;
		this->start = config.start; // useful to set the parameters and see how they perform whitout made the robot starts
		this->correction_angle=config.correction_angle;
		this->input_corridors_to_skip=config.corridors_to_skip;
		this->correction_speed=config.correction_speed;
		this->pole_radius=config.pole_radius;
		this->use_barycenter_as_point=config.use_barycenter_as_point;
	}

	// Takes vineyard info from a message
	void ParallelLineNav::vineyard_infoCb(const messages::vineyard_info::ConstPtr& msg){
		this->row_dim=msg->row_dim;
		this->corridor_width=msg->corridor_width;
	}
	
	// Takes controller info such as if the robot has been started
	void ParallelLineNav::sub_controller_callback(const messages::controller::ConstPtr& msg){
		start_work=msg->start;		
	}


int main(int argc, char** argv) {
	ros::init(argc, argv, "parallel_line_navigation");

	ParallelLineNav parallelNavigation(ros::this_node::getName());

	ros::spin();

	return 0;
}

