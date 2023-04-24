#include <turn/turn_action.hpp>
#include <ros/console.h>
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <array>
#include <vector>
#include <string>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <turn/TurnParamConfig.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <iostream>
#include <fstream>
	


	float ramp_up(float begin, float later, float t) {
	  if (t > 1.0) {
	    return later;
	  } else if (t < 0.0) {
	    return begin;
	  } else {
	    return later * t + begin * (1 - t);
	  }
	}


	// Constructor
	Turn::Turn(std::string name):
	pid_controller(0.0, 0.0, 0.0),
	n("~"),
	as (nh, name, false){

		n.getParam("odom_topic", odom_topic);
		n.getParam("storage", storage);
		pub_speeds = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);			// topic on which the robot is listening for speed commands
		odom_sub = nh.subscribe(odom_topic, 1, &Turn::odomCb, this);

		as.registerGoalCallback(std::bind(&Turn::goalCb, this));
		as.registerPreemptCallback(std::bind(&Turn::preemptCb, this));
		as.start();

		f = boost::bind(&Turn::dynamicParamCb, this, _1, _2);
		server.setCallback(f);
		
		obj1.open(storage+"/cpu_times/file_turn.txt", std::ofstream::trunc);

		already_rotated_angle=0;
		last_angle=0;
		p_gain=5;
		i_gain=0.03;
		d_gain=0;
		count=0;
		got_initial_pos=false;
		sum=true;

	}


// Method invoked when the ActionServer receives a new goal
	void Turn::goalCb(){
		goal = *as.acceptNewGoal();
		ROS_INFO("Turn Action: turning the robot at end of row.");
		default_turning_angle=goal.default_turning_angle;
		exit_enter=goal.exit_enter;



		// Before starting again I have to clear all the old values
		already_rotated_angle=0;
		last_angle=0;
		count=0;
		//pid_controller.p=0;
		got_initial_pos=false;
		//pid_controller.reset();

	}

	void Turn::preemptCb() {
		as.setPreempted();
		pub_speeds.publish(geometry_msgs::Twist());  // Stop
		ROS_INFO("TurnAction: Preempt!");
	}


// Dynamic Reconfigure method
	void Turn::dynamicParamCb(const turn::TurnParamConfig& config, uint32_t level) {
		this->use_personalized_angles=config.use_personalized_angles;
		this->exit_turning_angle=config.exit_turning_angle;
		this->entrance_turning_angle=config.entrance_turning_angle;
		this->fixed_speed=config.fixed_speed;
	}

	// Method that publish speeds to made the robot turn
	int Turn::turn_action(){
		double rotate_angle;
		double driveAngle;
		if(!use_personalized_angles) {
			if(default_turning_angle<0) 			// default_turning_angle contains the information about the turning side
				rotate_angle=-default_turning_angle;
			else 
				rotate_angle=default_turning_angle;
		}
		else if(exit_enter){					//exit_enter=1 means I want to turn to enter in a new row
			if(default_turning_angle<0)			// default_turning_angle contains the information about the turning side
				rotate_angle=-entrance_turning_angle;			
			else
				rotate_angle=entrance_turning_angle;
		}
		else if(!exit_enter){ 							//exit_enter=0 means I want to turn to go out of a row
			if(default_turning_angle<0)			// default_turning_angle contains the information about the turning side	
				rotate_angle= -exit_turning_angle;
			else
				rotate_angle= exit_turning_angle;
		}


		
		if(rotate_angle<0)	turning_side=-1;
		else if(rotate_angle>=0)	turning_side=1;

		// I have to remove the already rotated angle
		rotate_angle=rotate_angle-std::abs(already_rotated_angle)*turning_side;
		if((rotate_angle<=0 && turning_side==1) || (rotate_angle>=0 && turning_side==-1)){
			as.setSucceeded();

			geometry_msgs::Twist finalCommand;
			finalCommand.linear.x = 0.0;
			finalCommand.angular.z = 0.0;
			pub_speeds.publish(finalCommand);
		}
		else{
			geometry_msgs::Twist driveCommand;
			driveCommand.linear.x = 0.0;
			driveCommand.angular.z = fixed_speed*turning_side;
			pub_speeds.publish(driveCommand);
		}
		return 1;
	}

	void Turn::odomCb(const nav_msgs::Odometry::ConstPtr& msg) {	
		time=ros::WallTime::now();		
		double roll, pitch;
		double angle_step;
		
		count++;
		if(count % 10 == 0) {			//reduce responce frequency to 100 Hz
		
			if (as.isActive()) {
				tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
				tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
				
				if (got_initial_pos) {
					if(yaw<0){				
						yaw=6.28+yaw;
					}
					
					if(turning_side==1) angle_step=std::min(std::abs(yaw-last_angle), std::abs(yaw-(last_angle-6.28319)));
					else if(turning_side==-1) angle_step=std::min(std::abs(yaw-last_angle), std::abs(yaw-(last_angle+6.28319)));

					if(std::abs(angle_step)<1.57 && progress(last_angle, yaw)){ 	//HP: no more than 90 degrees can be travelled between subsequent odometry messages
					
						already_rotated_angle += angle_step;
						sum=true;
						// in this way an oscillation like 359 -> 1 -> 358 doesn't create problems
					}
					else{
						sum=false;
					}
				}
				else{
					if(yaw>=0){				
						initial_angle=yaw;
					}
					else{
						initial_angle=6.28+yaw;
						// the angles between 180 -- 360 are returned by getRPY as -3.14 -- 0			
					}
				}
				if(yaw>=0 && sum==true){				
					last_angle=yaw;
				}
				else if(yaw<0 && sum==true){
					last_angle=6.28+yaw;
					// the angles between 180 -- 360 are returned by getRPY as -3.14 -- 0			
				}
				got_initial_pos = true;
				turn_action();
			}
		
		obj1<<(ros::WallTime::now()-time).toSec()<<"\n";
		}
	}
	
	
	// Returns a boolean that indicates if the robot is oscillating while rotating
	bool Turn::progress(double last_angle, double yaw){
		if(turning_side==1){
			if(last_angle<yaw){
				// normal behaviour
				return true;
			}
			else if(last_angle>yaw && yaw<1.0 && last_angle>6.20){
				// passing over 0 degrees
				return true;
			}
			else {
				// wrong backward oscillation 
				return false;
			}
		}
		else if(turning_side==-1){
			if(last_angle>yaw){
				// normal behaviour
				return true;
			}
			else if(last_angle<yaw && last_angle<1.0 && yaw > 6.20){
				// passing over 2 degrees
				return true;
			}
			else{
				// wrong forward oscillation
				return false;
			}
		}
	}



int main(int argc, char** argv) {
	ros::init(argc, argv, "turn");

	Turn turn(ros::this_node::getName());

	ros::spin();

	return 0;
}
