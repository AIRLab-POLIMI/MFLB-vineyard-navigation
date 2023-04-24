#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>  
#include <string>
#include <fstream>
#include <fre_row_navigation/pid.h>
#include <dynamic_reconfigure/server.h>
#include <fre_row_navigation/PidTesterConfig.h>




class pid_tester{
	private: 
		std::ofstream obj1;
		
		ros::NodeHandle n;
		
		double p_gain, i_gain, d_gain;
		std::string storage;
		bool start;
		double old_output, output, error, input;
		
		PID pidController;
		

	public: 		
		pid_tester():pidController(p_gain, i_gain, d_gain){
			obj1.open("/home/veronica/Scrivania/pid_output.txt", std::ofstream::trunc);
			p_gain=0;
			i_gain=0;
			d_gain=0;
			start=false;
			
			n.getParam("/pid_tester/storage", storage);
			
			old_output=0;
			error=0;
		}

// Dynamic reconfigure callback
		void callback(fre_row_navigation::PidTesterConfig &config, uint32_t level) {
		ROS_INFO("Configuration changed");
			
			p_gain=config.p_gain;
			i_gain=config.i_gain;
			d_gain=config.d_gain;
			start=config.start;
			
			pidController.p=p_gain;
			pidController.i=i_gain;
			pidController.d=d_gain;
			
			if(start==true) 	pub_callback();
		}


		//void pub_callback(const ros::TimerEvent&){
		void pub_callback(){
			if(start==true){
				for(int i=0; i<1000; i++){
					if(i==0 || i==1 || i==2){
						input=0;
						old_output=0;
						obj1<<std::setprecision(15)<<ros::WallTime::now().toSec()<<" ";
						obj1<<pidController.calculate(input)<<"\n";
					}
					else{
						input=100;
						error=input-old_output;
						obj1<<std::setprecision(15)<<ros::WallTime::now().toSec()<<" ";
						output=pidController.calculate(error);
						obj1<<output<<"\n";
						old_output=output;
					}
				}
			}
			
		}
};



int main(int argc, char **argv){ 
	ros::init(argc, argv, "pid_tester");
	ros::NodeHandle n1;
	pid_tester c;

	
	dynamic_reconfigure::Server<fre_row_navigation::PidTesterConfig> server;
	dynamic_reconfigure::Server<fre_row_navigation::PidTesterConfig>::CallbackType f;

	f=boost::bind(&pid_tester::callback, &c, _1, _2);
	server.setCallback(f);


	ros::spin();
	return 0;
}
