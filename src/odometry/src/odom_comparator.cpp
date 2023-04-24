#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include "messages/velocities.h"
#include "messages/odom_comparator.h"
#include <cmath>
#include <tf/transform_broadcaster.h>




class pub_sub{
	public:
		pub_sub(ros::NodeHandle n){
			pub = n.advertise<messages::odom_comparator>("odom_comparator",10);

			av_x_sum=0;
			av_y_sum=0;
			av_th_sum=0;
			av_x_n=0;
			av_y_n=0;
			av_th_n=0;

			av_linear_vel_sum=0;
			av_angular_vel_sum=0;
			av_linear_vel_n=0;
			av_angular_vel_n=0;
		}

		void callback(const nav_msgs::Odometry::ConstPtr& scout_odom, const nav_msgs::Odometry::ConstPtr& my_odom){
			
				double  vx_ass,  vy_ass, vel;

				messages::odom_comparator msg;
				// construct the message
				msg.timestamp=ros::Time::now();


				// these speeds are absolute (I have decided to pass them in odometry.cpp)
				vx_ass=scout_odom->twist.twist.linear.x;
				vy_ass=scout_odom->twist.twist.linear.y;
				vel=sqrt(pow(vx_ass,2)+pow(vy_ass,2));

				msg.scout_velocities.linear_speed=vel;
				msg.scout_velocities.angular_speed=scout_odom->twist.twist.angular.z;

				vx_ass=my_odom->twist.twist.linear.x;
				vy_ass=my_odom->twist.twist.linear.y;
				vel=sqrt(pow(vx_ass,2)+pow(vy_ass,2));

				msg.my_velocities.linear_speed=vel;
				msg.my_velocities.angular_speed=my_odom->twist.twist.angular.z;
				msg.scout_pose=scout_odom->pose.pose;
				msg.my_pose=my_odom->pose.pose;

				double x_err=scout_odom->pose.pose.position.x-my_odom->pose.pose.position.x;
				double y_err=scout_odom->pose.pose.position.y-my_odom->pose.pose.position.y;

				tf::Pose pose;
  				tf::poseMsgToTF(scout_odom->pose.pose, pose);
  				double scout_yaw_angle = tf::getYaw(pose.getRotation());

				tf::poseMsgToTF(my_odom->pose.pose, pose);
  				double my_yaw_angle = tf::getYaw(pose.getRotation());		


				msg.x_err=abs(x_err);
				msg.y_err=abs(y_err);
				msg.th_err=abs(angle_err(scout_yaw_angle,my_yaw_angle));

				average_dist_err(x_err, y_err, msg.th_err);

				msg.average_x_err=av_x;
				msg.average_y_err=av_y;
				msg.average_th_err=av_th;

				// it is interesting to know the medium err on a linear meter and on a complete rotation
				msg.meter_err_x=abs((scout_odom->pose.pose.position.x-my_odom->pose.pose.position.x)/scout_odom->pose.pose.position.x);
				msg.meter_err_y=abs((scout_odom->pose.pose.position.y-my_odom->pose.pose.position.y)/scout_odom->pose.pose.position.y);
	
				msg.error_distance=dist(scout_odom->pose.pose.position, my_odom->pose.pose.position);
				
				average_vel_err(msg.scout_velocities.linear_speed, msg.my_velocities.linear_speed, msg.scout_velocities.angular_speed, msg.my_velocities.angular_speed);

				msg.linear_vel_err=abs(msg.scout_velocities.linear_speed-msg.my_velocities.linear_speed);
				msg.average_lin_vel_err=av_linear_vel_sum/av_linear_vel_n;
				msg.angular_vel_err=abs(msg.scout_velocities.angular_speed-msg.my_velocities.angular_speed);
				msg.average_ang_vel_err=av_angular_vel_sum/av_angular_vel_n;

				pub.publish(msg);

			}

			double average_vel_err(double scout_lin, double my_lin, double scout_ang, double my_ang){
				av_linear_vel_sum+=abs(scout_lin-my_lin);
				av_angular_vel_sum+=abs(scout_ang-my_ang);
			
				av_linear_vel_n++;
				av_angular_vel_n++;
			}

			double dist(geometry_msgs::Point scout_pos, geometry_msgs::Point my_pos){
				return sqrt(pow((scout_pos.x-my_pos.x), 2)+(pow((scout_pos.y-my_pos.y), 2)));
			}

			double angle_err(double an1, double an2){
				double sup_1=an1-an2;
				double sup_2=6.28319-an1+an2;
				return fmin(abs(sup_1), abs(sup_2));
			}

			void average_dist_err(double x, double y, double th){
				av_x_sum+=abs(x);
				av_y_sum+=abs(y);
				av_th_sum+=abs(th);

				av_x_n++;
				av_y_n++;
				av_th_n++;

				av_x=av_x_sum/av_x_n;
				av_y=av_y_sum/av_y_n;
				av_th=av_th_sum/av_th_n;
			}
		

	private:
		ros::Publisher pub;
		
		double av_x_sum, av_x_n, av_x;
		double av_y_sum, av_y_n, av_y;
		double av_th_sum, av_th_n, av_th;

		double av_linear_vel_sum, av_angular_vel_sum, av_linear_vel_n, av_angular_vel_n;
		
};


int main(int argc, char** argv){
	ros::init(argc, argv, "handler");
	ros::NodeHandle n;

	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
	
	
	message_filters::Subscriber<nav_msgs::Odometry> sub_scout_odom(n,"/my_raw_odom",10);
	message_filters::Subscriber<nav_msgs::Odometry> sub_my_odom(n,"/my_odom",10);	
	
	pub_sub* my_pub_sub=new pub_sub(n);

	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_scout_odom, sub_my_odom);
	sync.registerCallback(boost::bind(&pub_sub::callback, my_pub_sub, _1, _2)); 

	ros::spin();
	return 0;
}
