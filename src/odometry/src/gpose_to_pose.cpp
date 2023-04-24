#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <sstream>
#include <cmath>

class pub_sub{
	private:
		ros::Publisher pub;
		ros::Subscriber sub;

	public:
		ros::NodeHandle n; 
		
		
		pub_sub(){
			pub = n.advertise<geometry_msgs::PoseStamped>("/Robot_2/ground_pose_conv", 50);
			sub=n.subscribe("/Robot_2/ground_pose", 50, &pub_sub::callbackNewGroundPose, this);
		}



		// subscriber callback
		void callbackNewGroundPose(const geometry_msgs::Pose2D::ConstPtr& msg){
			// received a new message

			geometry_msgs::PoseStamped mex;
			mex.header.stamp=ros::Time::now();
			mex.header.frame_id="world";
			mex.pose.position.x=msg->x;
			mex.pose.position.y=msg->y;
			mex.pose.position.z=0;
			mex.pose.orientation=tf::createQuaternionMsgFromYaw(msg->theta);
			
			pub.publish(mex);
		}
};

int main(int argc, char **argv){ 
	ros::init(argc, argv, "groundpose_converter");
	ros::NodeHandle n;
	pub_sub converter;
	ros::spin();
	return 0;
}
