#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include "messages/wheel_velocities.h"
#include "messages/velocities.h"
#include <cmath>
#include <fstream>



class pub_sub_odom{
	private:
		ros::Publisher odom_pub;
		ros::Publisher vel_pub;
		ros::Subscriber sub;

		tf::TransformBroadcaster odom_broadcaster;

		bool publish_tf;

		// Read from message
		double v_left;
		double v_right;

		// Variables to calculate (vx, vy: local plane; vx_ass, vy_ass: global plane)
		double vx;			
		double vy;

		double alpha;
		double minus_y_icr;

		double vx_ass;
		double vy_ass;
		double velocity;
		double vth;

		// Displacement starts from plane origin
		double x;
		double y;
		double th;

		double dt; 
		double delta_x; 
		double delta_y; 
		double delta_th; 

		ros::Time current_time, last_time;
		
		ros::WallTime time;
		std::ofstream obj1;
 		std::string storage;
		


	public:
		ros::NodeHandle n; 
		
		// constructor		
		pub_sub_odom(){
			odom_pub = n.advertise<nav_msgs::Odometry>("my_odom", 50);
			vel_pub = n.advertise<messages::velocities>("my_vel", 50);
			sub=n.subscribe("/wheel_velocities", 50, &pub_sub_odom::callbackNewVelocities, this);

			n.getParam("publish_tf", publish_tf);

			n.getParam("/odometry/alpha", alpha);
			n.getParam("/odometry/minus_y_icr", minus_y_icr);
			n.getParam("/odometry/storage", storage);
			ROS_INFO("ALPHA: %f, Y: %f", alpha, minus_y_icr);

			
			// Displacement starts from plane origin
			x= 0.0;
			y= 0.0;
			th=0.0;
			velocity=0.0;


			// Initialization time istants
				
			current_time=ros::Time::now();
			last_time = ros::Time::now();
			
			obj1.open(storage+"/odometry.csv", std::ofstream::trunc);
		}



		void callbackNewVelocities(const messages::wheel_velocities::ConstPtr& msg){
			time=ros::WallTime::now();

			current_time=ros::Time::now();

			v_left=msg->v_left;
			v_right=msg->v_right;


			dt=(current_time-last_time).toSec();


			//vx, vy e vth formulas
			double s1, s2, s3, s4;
			double coeff;
			
			vy=0;
			

			s1=2*minus_y_icr;
			coeff=alpha/s1;
			s2=minus_y_icr*v_left;
			s3=minus_y_icr*v_right;
			s4=s2+s3;
			vx=coeff*s4;

			s2=v_right-v_left;
			vth=coeff*s2; 




			// speed wrt absolute plane
			vx_ass=vx*cos(th) - vy*sin(th);
			vy_ass=vx*sin(th) + vy*cos(th);
			velocity=sqrt(pow(vx_ass,2)+pow(vy_ass,2));


			// displacement wrt absolute plane origin
			delta_x=vx_ass*dt;
			delta_y=vy_ass*dt;
			delta_th=vth*dt; 

			// new position on absolute plane
			x+=delta_x;
			y+=delta_y;
			th+=delta_th;


			/****************************************************************************************************/
			// Creation and publishing of message with new robot tf
			geometry_msgs::Quaternion odom_quat=tf::createQuaternionMsgFromYaw(th);

			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp=current_time;
			odom_trans.header.frame_id="odom";	
			odom_trans.child_frame_id="scout_link";

			odom_trans.transform.translation.x=x; 
			odom_trans.transform.translation.y=y;
			odom_trans.transform.translation.z=0.0;
			odom_trans.transform.rotation=odom_quat;

			if(publish_tf) odom_broadcaster.sendTransform(odom_trans);

			
			// Creation and publishing of odometry message
			nav_msgs::Odometry odom;
			odom.header.stamp = current_time;
			odom.header.frame_id="odom";

			odom.pose.pose.position.x=x;
			odom.pose.pose.position.y=y;
			odom.pose.pose.position.z=0.0;
			odom.pose.pose.orientation=odom_quat;

			odom.child_frame_id="scout_link";
			odom.twist.twist.linear.x=vx_ass;		// I give the absolute, but I should pass the relative (nothing changes in this case)
			odom.twist.twist.linear.y=vy_ass;
			odom.twist.twist.angular.z=vth;

			odom_pub.publish(odom);


			// set up for next iteration
			last_time=current_time;	
			
			obj1<<(ros::WallTime::now()-time).toSec()<<"\n";
		}
};


int main(int argc, char **argv){ 
	ros::init(argc, argv, "odometry_publisher");
	pub_sub_odom odom;
	ros::spin();
	return 0;
}

