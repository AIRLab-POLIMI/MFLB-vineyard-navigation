#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <string>





class converter{
	private:
		ros::Publisher pub;
		ros::Subscriber sub;
		tf::TransformBroadcaster odom_broadcaster;

	public:
		ros::NodeHandle n; 
		
		converter(){
			pub = n.advertise<nav_msgs::Odometry>("my_odom", 50);
			sub=n.subscribe("/gazebo/model_states", 50, &converter::callbackStatus, this);
		}



		void callbackStatus(const gazebo_msgs::ModelStates::ConstPtr& msg){
			nav_msgs::Odometry mex;


			int i=find_scout_index(msg);

			mex.header.stamp=ros::Time::now();
			mex.header.frame_id="odom";
			mex.child_frame_id="scout_link";
			mex.pose.pose=msg->pose[i];
			mex.twist.twist=msg->twist[i];

			// msg.pose[0] msg.twist[0]: world position and speed
			// msg.pose[1] msg.pose[1]: robot position and speed

			pub.publish(mex);

			// creation and publishing of message with robot tf
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp=ros::Time::now();
			odom_trans.header.frame_id="odom";	
			odom_trans.child_frame_id="scout_link";



			odom_trans.transform.translation.x=msg->pose[i].position.x; 
			odom_trans.transform.translation.y=msg->pose[i].position.y;
			odom_trans.transform.translation.z=msg->pose[i].position.z;
			odom_trans.transform.rotation=msg->pose[i].orientation;

			odom_broadcaster.sendTransform(odom_trans);



		}


		// find the position in which Gazebo returns Scout position & speed
		int find_scout_index(const gazebo_msgs::ModelStates::ConstPtr& msg){
			for (int j=(msg->name).size()-1; j>0; j--){
				if(msg->name[j]== "scout/"){
					return j;			
				}
			}
			return -1;
		}

};


int main(int argc, char **argv){ 
	ros::init(argc, argv, "odometry_sim");
	converter conv;
	ros::spin();
	return 0;
}
