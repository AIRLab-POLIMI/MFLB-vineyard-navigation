#include <string>
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
#include <turn/TurnAction.h>
#include <turn/TurnParamConfig.h>
#include <fre_row_navigation/pid.h>
#include <fstream>


class Turn{

	private:
		ros::NodeHandle n;
		ros::NodeHandle nh;
		ros::Publisher pub_speeds;
		ros::Subscriber odom_sub;

		actionlib::SimpleActionServer<turn::TurnAction> as;
		turn::TurnGoal goal;

		dynamic_reconfigure::Server<turn::TurnParamConfig> server;
		dynamic_reconfigure::Server<turn::TurnParamConfig>::CallbackType f;

		bool use_personalized_angles;
		double exit_turning_angle;
		double entrance_turning_angle;
		double default_turning_angle;

		int exit_enter;
		double initial_angle;
		double already_rotated_angle;
		bool got_initial_pos;
		std::string odom_topic;
		float p_gain;
		float i_gain;
		float d_gain;
		double last_angle;
		double fixed_speed;
		double yaw;
		int turning_side;
		int count;
		
		ros::WallTime time;
		std::ofstream obj1;
		std::string storage;
		
		bool sum;

		PID pid_controller;

	public:
		Turn(std::string name);
		void goalCb();
		void preemptCb();
		void dynamicParamCb(const turn::TurnParamConfig& config, uint32_t level);
		int turn_action();
		void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
		bool progress(double last_angle, double yaw);
};
