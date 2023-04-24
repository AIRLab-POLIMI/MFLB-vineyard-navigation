#include "ros/ros.h"
#include "std_msgs/String.h"
#include "messages/controller.h"
#include "messages/vineyard_info.h"
#include <iostream>  
#include <string>
#include <dynamic_reconfigure/server.h>
#include <navigation/navigation_controllerConfig.h>




class controller_manager{
	private: 
		bool start;
		std::string initial_turning_side;
		double corridor_width;
		double row_dim;
		double row_length;
		double todo_rows;
		bool path_from_file;

		ros::NodeHandle n;
		ros::Publisher pub;
		ros::Publisher vineyard_info_pub;

	public: 		
		controller_manager(){
			start=false;
			initial_turning_side="left";
			pub=n.advertise<messages::controller>("controller", 50);
			vineyard_info_pub=n.advertise<messages::vineyard_info>("vineyard_info", 50);

		}

		// Dynamic reconfigure callback
		void callback(navigation::navigation_controllerConfig &config, uint32_t level) {
		ROS_INFO("Configuration changed");
			start=config.start_cmd;
			initial_turning_side=config.initial_turning_side;
			corridor_width=config.corridor_width;
			row_dim=config.row_dim;
			row_length=config.row_length;
			path_from_file=config.path_from_file;
		}


		void pub_callback(const ros::TimerEvent&){

			// publish control info message
			messages::controller mex;
			mex.start=start;
			mex.initial_turning_side=initial_turning_side;
			mex.row_length=row_length;
			mex.path_from_file=path_from_file;
			pub.publish(mex);

			// publish vineyard info message
			messages::vineyard_info msg;
			msg.corridor_width=corridor_width;
			msg.row_dim=row_dim;
			vineyard_info_pub.publish(msg);
		}
};



int main(int argc, char **argv){ 
	ros::init(argc, argv, "controller");
	ros::NodeHandle n1;
	controller_manager c;

	
	dynamic_reconfigure::Server<navigation::navigation_controllerConfig> server;
	dynamic_reconfigure::Server<navigation::navigation_controllerConfig>::CallbackType f;

	f=boost::bind(&controller_manager::callback, &c, _1, _2);
	server.setCallback(f);

	ros::Timer timer=n1.createTimer(ros::Duration(0.02), &controller_manager::pub_callback, &c); // 50 Hz
	

	ros::spin();
	return 0;
}
