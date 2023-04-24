#include <cmath>
#include "Robot.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"

		
		// sets all the parameters
		void Robot::set_speeds(double left, double right){
			left_speed=left;
			right_speed=right;
		}

		double Robot::get_left_speed(){ return left_speed; }

		double Robot::get_right_speed(){ return right_speed; }


		void Robot::set_actual_angle(double angle){
			actual_angle=angle;
		}

		void Robot::set_actual_angle(geometry_msgs::Quaternion orientation){
			actual_orientation=orientation;
		}

		double Robot::get_actual_angle(){
			return actual_angle;		
		}

		geometry_msgs::Quaternion Robot::get_actual_orientation(){
			return actual_orientation;		
		}


		void Robot::set_actual_position(geometry_msgs::Point position){
			actual_position=position;
		}
	
		geometry_msgs::Point Robot::get_actual_position(){
			return actual_position;		
		}


		void Robot::set_view(sensor_msgs::PointCloud2 v){
			view=v;
		}

		sensor_msgs::PointCloud2 Robot::get_view(){
			return view;
		}


		void Robot::set_actual_turning_side(std::string side){
			actual_turning_side=side;
		}

		std::string Robot::get_actual_turning_side(){
			return actual_turning_side;
		}


		void Robot::set_moving(double value){
			moving=value;
		}

		bool Robot::get_moving(){
			return moving;
		}

