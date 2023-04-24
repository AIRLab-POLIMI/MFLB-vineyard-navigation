#include <fre_row_navigation/crawl_row_node.h>
#include <algorithm>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <array>
#include <vector>
#include "messages/vineyard_info.h"
#include "messages/controller.h"

template <typename V, typename S>
V lerp(V a, V b, S t) {
  return a + t * (b - a);
}

float ramp_up(float begin, float later, float t) {
  if (t > 1.0) {
    return later;
  } else if (t < 0.0) {
    return begin;
  } else {
    return later * t + begin * (1 - t);
  }
}

CrawlRowAction::CrawlRowAction()
    : ph("~"),
      as(nh, "my_crawl_row", false),
      tf_listener(tf_buffer),
      p_gain(0),
      i_gain(0),
      d_gain(0),
      pidController(p_gain, i_gain, d_gain),
      travelled_distance(0),
      got_initial_pos(false) {

	ph.getParam("odom_topic", odom_topic);
	ph.getParam("lidar_sensor", lidar_sensor);
	ph.getParam("storage", storage);
	as.registerGoalCallback(std::bind(&CrawlRowAction::goalCb, this));
	as.registerPreemptCallback(std::bind(&CrawlRowAction::preemptCb, this));

	lidar_front_sub = nh.subscribe("lidar_front", 1, &CrawlRowAction::lidarCb, this);
	cloud_front_sub = nh.subscribe("lidar_front_cloud_filtered", 1, &CrawlRowAction::cloudCb, this);
	odom_sub = nh.subscribe(odom_topic, 1, &CrawlRowAction::odomCb, this);
	vineyard_info_sub=nh.subscribe("vineyard_info", 1, &CrawlRowAction::vineyard_infoCb, this);
	controller_sub=nh.subscribe("controller", 50, &CrawlRowAction::sub_controller_callback, this);

	viz_pub = ph.advertise<visualization_msgs::Marker>("viz", 50);
	array_viz_pub = ph.advertise<visualization_msgs::MarkerArray>("viz_array", 50);
	drive_pub = ph.advertise<geometry_msgs::Twist>("drive_command", 1);
	p_gain_pub=ph.advertise<std_msgs::Float32>("p_gain", 1);
	driveAngle_pub=ph.advertise<std_msgs::Float32>("driveAngle", 1);

	decltype(dyn_param_server)::CallbackType dyn_param_callback = boost::bind(&CrawlRowAction::dynamicParamCb, this, _1, _2);
	dyn_param_server.setCallback(dyn_param_callback);
	
	obj1.open(storage+"/cpu_times/file_crawl_row.txt", std::ofstream::trunc);
	obj2.open(storage+"/trajectory_follow/trajectory.txt", std::ofstream::trunc);
	obj3.open(storage+"/trajectory_follow/lateral_dist.txt", std::ofstream::trunc);
	obj4.open(storage+"/trajectory_follow/angles.txt", std::ofstream::trunc);
	pid_output.open(storage+"/pid_output.txt", std::ofstream::trunc);

	as.start();
	ROS_INFO("CrawlRowAction: Started crawl row action server");

	count=3000;
	work=0;
	work=false;
	finishing_the_row=false;
	done_rows=0;
	
	start_turning_time=ros::Time::now();
	end_turning_time=ros::Time::now();
}

void CrawlRowAction::goalCb() {
	end_turning_time=ros::Time::now();
	goal = *as.acceptNewGoal();
	ROS_INFO("CrawlRowAction: Got new goal!");
	// Before starting again I have to clear all the old values
	count=3000;
	travelled_distance=0;
	got_initial_pos=false; 
	driveAngle=0;
	pidController.reset();



	speed = 0.;
	start_travelled_distance = travelled_distance;
	start_time = ros::Time::now();
	max_reachable_speed=goal.speed;
	min_row_length=goal.min_row_length;
	work=1;
	finishing_the_row=false;
	
}

void CrawlRowAction::preemptCb() {
	as.setPreempted();
	drive_pub.publish(geometry_msgs::Twist());  // Stop
	ROS_INFO("CrawlRowAction: Preempt!");
}





void CrawlRowAction::processLidar(const sensor_msgs::LaserScan& scan) {


	if(work){		// work=1 if goal is active
		double lineShift;

		// Find a cone in the laser scans
		double obstacle_angle_rad = this->obstacle_angle / 180. * M_PI;
		Cone middleCone = Cone::findCone(scan, 0, ray_length, obstacle_angle_rad, this->min_obs_density);

		Cone bestCone = middleCone;
		if (this->multi_cone_enabled && (middleCone.width() < this->min_cone_width / 180 * M_PI)) {
			Cone leftCone = Cone::findCone(scan, 0.4, ray_length, obstacle_angle_rad, this->min_obs_density);
			Cone rightCone = Cone::findCone(scan, -0.4, ray_length, obstacle_angle_rad, this->min_obs_density);

			if (middleCone.width() >= leftCone.width() && middleCone.width() >= rightCone.width()) {
				bestCone = middleCone;
				ROS_INFO("CrawlRowAction: Best cone is the middle cone");
			} else if (leftCone.width() >= rightCone.width()) {
				bestCone = leftCone;
				ROS_INFO("CrawlRowAction: Best cone is the left cone");
			} else {
				bestCone = rightCone;
				ROS_INFO("CrawlRowAction: Best cone is the right cone");
			}
		}

		// If middle was bad but left and right were bad too just go forward
		if (bestCone.width() < this->min_cone_width / 180 * M_PI) {
			bestCone.startAngle = 0.0;
			bestCone.endAngle = 0.0;
			ROS_INFO("CrawlRowAction: LATERAL CONES ARE BAD TOO");
		}

		// If cone is to big, this is probably a hole in the row
		if (bestCone.width() >= this->max_cone_width / 180 * M_PI) {
			bestCone.startAngle = 0.0;
			bestCone.endAngle = 0.0;
			ROS_INFO("CrawlRowAction: PROBABLY A HOLE IN THE ROW");
		}



		// Draw the best cone with the middle line in it (no correction is drawn here)
		viz_pub.publish(bestCone.createMarker(1));

		// Get cone transform
		geometry_msgs::TransformStamped cone_transform_msg;
		try {
			cone_transform_msg = tf_buffer.lookupTransform("base_link", bestCone.frame_id, ros::Time(0));
		} catch (tf2::TransformException& ex) {
			ROS_ERROR_STREAM("Could not lookup lidar transform: " << ex.what());
			return;
		}
		tf::StampedTransform cone_transform;
		tf::transformStampedMsgToTF(cone_transform_msg, cone_transform);

		tf::Point cone_point_in_base_link = cone_transform * bestCone.getMiddleLinePoint();


		// Draw middle row of the cone	
		visualization_msgs::Marker pm;
		pm.header.frame_id = "base_link";
		pm.ns = "cone";
		pm.id = 123;
		pm.type = visualization_msgs::Marker::POINTS;
		pm.action = visualization_msgs::Marker::ADD;
		pm.scale.x = 0.2;
		pm.scale.y = 0.2;
		pm.scale.z = 0.2;
		pm.color.r = 1;
		pm.color.a = 1;
		pm.lifetime=ros::Duration(3.5);
		geometry_msgs::Point p;
		tf::pointTFToMsg(cone_point_in_base_link, p);
		pm.points.push_back(p);
		viz_pub.publish(pm);


		// Use resizing rectangles for correction
		if (cloudFront) {
			float x1 = 0.4;
			float x2 = 0.7;

			Rectangle rect_left = Rectangle::grow_y(x1, x2, 0.1, 0.02, max_lateral_rect_dist, *cloudFront, 8);
			viz_pub.publish(rect_left.createMarker(1, "base_link"));
			float yLeft = std::abs(rect_left.y2);

			Rectangle rect_right = Rectangle::grow_y(x1, x2, -0.1, -0.02, -max_lateral_rect_dist, *cloudFront, 8);
			viz_pub.publish(rect_right.createMarker(2, "base_link"));
			float yRight = std::abs(rect_right.y2);

			obj3<<yLeft<<", "<<yRight<<"\n";
		
			// the side to which you are nearest gives your distance from the center, the other will be 0. 
			// NB: the corridor width must be set correctly!
			yLeft = -std::max(-yLeft + (corridor_width/2), 0.0);
			yRight = std::max(-yRight + (corridor_width/2), 0.0);
			

			double sum = yLeft + yRight;
			// the distance you moved away should be taken divided by ray_length, no division guarantees faster correction
			lineShift = sum ;
			
			
			
			obj2<<last_pos.x<<", "<<last_pos.y<<"\n";



			// Detect end of row
			Rectangle end_of_row_far_rect;
			end_of_row_far_rect.x1 = sensor_min_range+0.8;	// ouster doesn't see 0.8 cm near it: move rectangles of sensor_min_range
			end_of_row_far_rect.x2 = sensor_min_range+1.0;
			end_of_row_far_rect.y1 = -(corridor_width/2+row_dim);
			end_of_row_far_rect.y2 = (corridor_width/2+row_dim);
			visualization_msgs::Marker end_of_row_far_marker = end_of_row_far_rect.createMarker(4, "base_link");
			end_of_row_far_marker.color.a = 0.3;
			bool far_end_of_row = end_of_row_far_rect.getNumPointsInsideRectangle(*cloudFront) < 3;
			if (far_end_of_row) {
				end_of_row_far_marker.color.r = 1;
			}


			Rectangle end_of_row_rect;
			end_of_row_rect.x1 = sensor_min_range+0.4;	// ouster doesn't see 0.8 cm near it: move rectangles of sensor_min_range
			end_of_row_rect.x2 = sensor_min_range+0.8;
			end_of_row_rect.y1 = -(corridor_width/2+row_dim);
			end_of_row_rect.y2 = (corridor_width/2+row_dim);
			visualization_msgs::Marker end_of_row_marker =end_of_row_rect.createMarker(3, "base_link");
			end_of_row_marker.color.a = 0.3;
			bool near_end_of_row = end_of_row_rect.getNumPointsInsideRectangle(*cloudFront) < 3;
			if (near_end_of_row) {
				end_of_row_marker.color.r = 1;
			}
			
			
			Rectangle very_end_of_row_rect;
			very_end_of_row_rect.x1 = sensor_min_range;
			very_end_of_row_rect.x2 = sensor_min_range+0.4;
			very_end_of_row_rect.y1 = -(corridor_width/2+row_dim);
			very_end_of_row_rect.y2 = (corridor_width/2+row_dim);
			visualization_msgs::Marker very_end_of_row_marker =very_end_of_row_rect.createMarker(4, "base_link");
			very_end_of_row_marker.color.a = 0.3;
			very_end_of_row_marker.color.b = 0.3;
			bool very_near_end_of_row = very_end_of_row_rect.getNumPointsInsideRectangle(*cloudFront) < 3;
			if (very_near_end_of_row) {
				very_end_of_row_marker.color.r = 1;
			}

			bool n_and_f_eor = near_end_of_row && far_end_of_row && very_near_end_of_row;
			if (n_and_f_eor) {
				if(finishing_the_row==false){
					finishing_the_row=true;
					start_end_row=travelled_distance;
				}
			} else {
				finishing_the_row=false;
			}

			float travelled = std::abs(travelled_distance - start_travelled_distance);
			bool travelled_enough = travelled > min_row_length;
			bool travelled_out_enough = ((travelled_distance-start_end_row)> end_line_meters_threshold) && travelled_enough && finishing_the_row;
		
			if(travelled_out_enough){
				end_of_row_marker.color.g = 0;
			}
			
			end_of_row_marker.lifetime=ros::Duration(3.5);
			end_of_row_far_marker.lifetime=ros::Duration(3.5);
			viz_pub.publish(end_of_row_marker);
			viz_pub.publish(end_of_row_far_marker);

			if(travelled_out_enough){
				// Stop
				geometry_msgs::Twist driveCommand;
				drive_pub.publish(driveCommand);
				as.setSucceeded();
				done_rows++;
				work=0;
				start_turning_time=ros::Time::now();

				return;
			}


			// Speed control
			if(max_reachable_speed>speed_high) max_reachable_speed=speed_high;    // Goal speed max_reachable_speed has to be less than high_speed
			
			if (use_speed_control) {
				float sreclenmin = 0.3;
				float sreclenmax = 2.0;
				float sx1 = sreclenmin;
				float sx2 = sreclenmax;
				float growth = 0.05;

				// Rectangle to check obstacles in front of the robot and calculate linear speed accordingly
				Rectangle rect_drive_dir = Rectangle::grow_x(sx1, 0.2, -0.2, growth, sx2, *cloudFront, 10);
				visualization_msgs::Marker marker = rect_drive_dir.createMarker(1287, "base_link");
				marker.color.b = 1.0;
				marker.color.a = 0.5;
				marker.lifetime=ros::Duration(3.5);
				viz_pub.publish(marker);
				float speed_rect_len = std::abs(rect_drive_dir.x2 - rect_drive_dir.x1);
				float calc_speed =lerp(this->speed_low, this->speed_high, (speed_rect_len - sreclenmin) / (sreclenmax - sreclenmin));
				
				// To check if there is something in front of my calculated cone
				Rectangle rect_obj_in_front;	
				rect_obj_in_front.x1 = distance_obstacle;
				rect_obj_in_front.x2 = distance_obstacle+0.2;
				rect_obj_in_front.y1 = 0.2;
				rect_obj_in_front.y2 = -0.2;
				visualization_msgs::Marker obj_marker=rect_obj_in_front.createMarker(1288, "base_link");
				obj_marker.color.r = 0.5;
				obj_marker.color.a = 0.5;
				obj_marker.lifetime=ros::Duration(3.5);
				viz_pub.publish(obj_marker);
				
				bool obs;
				if(slow_down_obstacle==true){
					// Using option to slow down if there is an obstacle in front of me
					obs = rect_obj_in_front.getNumPointsInsideRectangle(*cloudFront)>15;
				}
				else{
					// Do not use this option
					obs=false;
				}
				
				// Slow down so we dont overshoot end of row
				if (near_end_of_row || obs) {
					calc_speed = -this->speed_low;
					ROS_INFO("Lowering speed because near end of row is true or there is something in front of me.");
				}
				// smooth
				speed = speed * 0.90 + calc_speed * 0.1;
				if(speed < this->speed_low) speed=this->speed_low;
				
				if(max_reachable_speed<speed){
					speed=max_reachable_speed;
				}
			} else {
				// Min speed is used if use_speed_control option is not checked
				speed=speed_low;	
			}
			

			// Correction and angular speed calculation
			pidController.p =ramp_up(p_gain * 2, p_gain, (travelled_distance - start_travelled_distance) / 0.2);
			pidController.i = i_gain;
			pidController.d = d_gain;

			double atan_dir = std::atan2(cone_point_in_base_link.y(), cone_point_in_base_link.x());
			// cone_point_in_base_link is a point at ray_length distance on the line that divides the cone in the middle 
			oldDriveAngle=driveAngle;
			if (start_work==true) {
				driveAngle = pidController.calculate(atan_dir + lineShift);
			}
			/*
				applied theorems:
				1- an angle in radians is equal to the length of the arc that subtends it divided by the radius
				2- for small angles the arc and chord are similar (almost the same)
				Here we do not divide by radius to speed up the centering process
			*/
			
			// print on file robot orientation wrt odom frame  (angle between scout and odom + angle wrt scout)
			
			// Get point transform in odom frame
			geometry_msgs::TransformStamped point_transform_msg2;
			try {
				point_transform_msg2 = tf_buffer.lookupTransform("scout_link", "odom",  ros::Time(0), ros::Duration(3.0));
			} catch (tf2::TransformException& ex) {
				ROS_ERROR_STREAM("Could not lookup odom-cone_link transform: " << ex.what());
				return;
			}
			double roll, pitch, yaw;
			tf::Quaternion q(point_transform_msg2.transform.rotation.x, point_transform_msg2.transform.rotation.y, point_transform_msg2.transform.rotation.z, point_transform_msg2.transform.rotation.w);
			tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
			double absolute_angle;
			if(std::abs(yaw)>2) absolute_angle=-3.14159;			// I am in another row and I have turned 180 grades respect the initial point
			else absolute_angle=0;
			
			obj4<<std::abs(yaw)<<" "<<driveAngle<<" "<<absolute_angle<<"\n";



			// Create cone_link frame tf (equal to baselink but rotated as the cone)
			geometry_msgs::Quaternion odom_quat=tf::createQuaternionMsgFromYaw(bestCone.middle());

			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp=ros::Time::now();
			odom_trans.header.frame_id="base_link";	
			odom_trans.child_frame_id="cone_link";

			odom_trans.transform.translation.x=0.0; 
			odom_trans.transform.translation.y=0.0;
			odom_trans.transform.translation.z=0.0;
			odom_trans.transform.rotation=odom_quat;

			odom_broadcaster.sendTransform(odom_trans);


			// Publish speeds
			if (far_end_of_row) {
				// In this way the robot stops steering when it is near the end of a row. It is necessary because in this zone cones are altered by the side walls missing.
				geometry_msgs::Twist driveCommand;
				driveCommand.linear.x = speed;
				driveCommand.angular.z = 0;
				drive_pub.publish(driveCommand);
			} else {
				geometry_msgs::Twist driveCommand;
				driveCommand.linear.x = speed;
				driveCommand.angular.z = driveAngle;
				drive_pub.publish(driveCommand);
			}
		} else {
			ROS_ERROR("No cloud");
			geometry_msgs::Twist driveCommand;
			drive_pub.publish(driveCommand);
		}
	}
}


//LaserScan message callback
void CrawlRowAction::lidarCb(const sensor_msgs::LaserScan::ConstPtr& msg) {
	if (as.isActive()) {
		time=ros::WallTime::now();

		// Check if other corridors have to be navigated
		if (done_rows<todo_rows) {
			processLidar(*msg);
		}
		else {
			ROS_INFO("Done all the rows!");
			// Stop
			geometry_msgs::Twist driveCommand;
			drive_pub.publish(driveCommand);	
		}
		obj1<<(ros::WallTime::now()-time).toSec()<<"\n";
	}
}


//PointCloud message callback
void CrawlRowAction::cloudCb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg) {
	cloudFront = msg;
}


// Odometry message callback
void CrawlRowAction::odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
	if (got_initial_pos) {
		double dx = last_pos.x - msg->pose.pose.position.x;
		double dy = last_pos.y - msg->pose.pose.position.y;
		double d = std::sqrt(dx * dx + dy * dy);
		travelled_distance += d;
	}
	last_pos = msg->pose.pose.position;
	got_initial_pos = true;

}


//Dynamic reconfigure callback
void CrawlRowAction::dynamicParamCb(const fre_row_navigation::CrawlRowParamConfig& config, uint32_t level) {
  	this->use_speed_control = config.use_speed_control;
	this->end_line_meters_threshold = config.end_line_meters_threshold+0.8; // the end row rectangle is 0.8m moved forward wrt the robot (so the robot has to travel also that distance)
	this->p_gain = config.p;
	this->i_gain = config.i_gain;
	this->d_gain = config.d_gain;
	this->ray_length = config.ray_length;
	this->max_lateral_rect_dist = config.max_lateral_rect_dist;
	this->min_cone_width = config.min_cone_width;
	this->max_cone_width = config.max_cone_width;
	this->multi_cone_enabled = config.multi_cone_enabled;
	this->obstacle_angle = config.obstacle_angle;
	this->min_obs_density = config.min_obs_density;
	this->speed_low = config.speed_low;
	this->speed_high = config.speed_high;
	this->sensor_min_range=config.sensor_min_range;
	this->distance_obstacle=config.distance_obstacle;
	this->todo_rows=config.todo_rows;
}


// Callback listening to vineyard_info messages
void CrawlRowAction::vineyard_infoCb(const messages::vineyard_info::ConstPtr& msg){

	this->corridor_width = msg->corridor_width;
	this->row_dim = msg->row_dim;

}
// Callback listening to controller messages
void CrawlRowAction::sub_controller_callback(const messages::controller::ConstPtr& msg){
	start_work=msg->start;		
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "crawl_row_node");

	ros::NodeHandle nh;
	CrawlRowAction crawlRowAction;

	ros::spin();

	return 0;
}
