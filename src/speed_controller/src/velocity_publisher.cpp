#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sstream>



#include <dynamic_reconfigure/server.h>
#include <speed_controller/velocity_inputsConfig.h>


class speed_manager{
	private: 
		double linear_speed;
		double angular_speed;
		ros::NodeHandle n2;

	public: 		
		speed_manager(){
			linear_speed=0.0;
			angular_speed=0.0;

		}

		void callback(speed_controller::velocity_inputsConfig &config, uint32_t level) {
		ROS_INFO("Changed config.");
			linear_speed=(double)config.linear_vel;
			angular_speed=(double)config.angular_vel;
		}

		double get_linear_speed(){
			return linear_speed;
		}

		double get_angular_speed(){
			return angular_speed;
		}

		void pub_callback(const ros::TimerEvent&){
			ros::Publisher pub;
			pub=n2.advertise<geometry_msgs::Twist>("cmd_vel", 50);
			geometry_msgs::Twist mex;
			
			mex.linear.x=(double) this->get_linear_speed();
			mex.linear.y=0;
			mex.linear.z=0;

			mex.angular.x=0;	
			mex.angular.y=0;
			mex.angular.z=(double) this->get_angular_speed();
			// angular speed vector is a vector perpendicular to xy (it is on z)
			
			pub.publish(mex);
		}
};



int main(int argc, char **argv){ 
	ros::init(argc, argv, "velocities_publisher");
	ros::NodeHandle n1;
	speed_manager p;
	
	

	
	dynamic_reconfigure::Server<speed_controller::velocity_inputsConfig> server;
	dynamic_reconfigure::Server<speed_controller::velocity_inputsConfig>::CallbackType f;

	f=boost::bind(&speed_manager::callback, &p, _1, _2);
	server.setCallback(f);

	ros::Timer timer=n1.createTimer(ros::Duration(0.02), &speed_manager::pub_callback, &p); // 50 Hz

	ros::spin();
	
	return 0;

}

