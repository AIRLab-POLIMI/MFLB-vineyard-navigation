#ifndef ROBOT
#define ROBOT

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"

class Robot {

	public:
		
		// sets all the parameters
		void set_speeds(double left, double right);
		double get_left_speed();
		double get_right_speed();

		void set_actual_angle(double angle);
		void set_actual_angle(geometry_msgs::Quaternion orientation);
		double get_actual_angle();
		geometry_msgs::Quaternion get_actual_orientation();

		void set_actual_position(geometry_msgs::Point position);
		geometry_msgs::Point get_actual_position();
		
		void set_view(sensor_msgs::PointCloud2 v);
		sensor_msgs::PointCloud2 get_view();

		void set_actual_turning_side(std::string side);
		std::string get_actual_turning_side();

		void set_moving(double value);
		bool get_moving();


	private:
		double left_speed;
		double right_speed;
		geometry_msgs::Point actual_position;				// wrt absolute XY plane
		geometry_msgs::Quaternion actual_orientation;			//quaternion wrt absolute XY plane
		double actual_angle;						//rad wrt absolute XY plane
		sensor_msgs::PointCloud2 view;
		std::string actual_turning_side;					// 0:left, 1:right
		bool moving;

		

};
#endif 
