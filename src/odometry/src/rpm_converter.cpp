#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "messages/wheel_velocities.h"
#include "scout_msgs/ScoutStatus.h"
#include "scout_msgs/ScoutMotorState.h"
#include <cmath>






class converter{
	private:
		ros::Publisher pub;
		ros::Subscriber sub;
		double converted_rpm;


	public:
		ros::NodeHandle n; 
		
		converter(){
			pub = n.advertise<messages::wheel_velocities>("wheel_velocities", 50);
			sub=n.subscribe("/scout_status", 50, &converter::callbackStatus, this);
		}


		void callbackStatus(const scout_msgs::ScoutStatus::ConstPtr& msg){
			messages::wheel_velocities mex;

			mex.header.stamp=ros::Time::now();

			// motor_states[0] = front right wheel
			// motor_states[1] = back right wheel 
			// motor_states[2] = front left wheel 
			// motor_states[3] = back left wheel 


			// NB: the two values of rpm are not perfectly equal on the same side. If only one is 0 means also the other one has to be.
			if(msg->motor_states[2].rpm!=0 && msg->motor_states[3].rpm!=0){
				converted_rpm=(get_sign(msg)*msg->motor_states[2].rpm)/39.84;
				mex.v_left=(((converted_rpm*0.165)*2)*M_PI)/60;
			}	
			else {
				mex.v_left=0.0;
			}
			if(msg->motor_states[0].rpm!=0 && msg->motor_states[1].rpm!=0){
				converted_rpm=(msg->motor_states[0].rpm)/39.84;
				mex.v_right=(((converted_rpm*0.165)*2)*M_PI)/60;
			}
			else {
				mex.v_right=0.0;			
			}

			pub.publish(mex);
		}

		// left side has to be changed cause his sign is inverted
		double get_sign(const scout_msgs::ScoutStatus::ConstPtr& msg){
			if(msg->motor_states[0].rpm>=0 && msg->motor_states[1].rpm>=0){ //two right motors positive: go ahead or turn left
				if(msg->motor_states[2].rpm<=0 && msg->motor_states[3].rpm<=0){ // two left motors negative: go ahead		
					return -1;				
				}
				else { // two left motors positive: turn left
					return -1;			
				}
			}
			else{ // two right motors negative: go back or turn right 
				if(msg->motor_states[2].rpm<=0 && msg->motor_states[3].rpm<=0){ // two left motors negative: turn right		
					return -1;				
				}
				else { // two posterior motors positive: go back
					return -1;			
				}
			}
		}
};


int main(int argc, char **argv){ 
	ros::init(argc, argv, "odometry_publisher");
	converter conv;
	ros::spin();
	return 0;
}

