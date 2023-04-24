#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include "position.hpp"
#include "messages/wheel_velocities.h"
#include "messages/odom_param.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cmath>




class param_estimator{
	public:
		param_estimator(){

  
			
			pub = n.advertise<messages::odom_param>("odom_param",10);
			
			is_initial_point=true;
			seq=0;

			int_vr_dt=0;
			int_vl_dt=0;
			dist=0;
			phi=0;
			passed_by_start=0;
			

			received_msg_num=0;
			clock_wise=false;
			anti_clock_wise=false;


			current_time=ros::Time::now();
			last_time = ros::Time::now();
			
			min_lap_time=1.0;
			
		}

		void callback(const geometry_msgs::PoseStamped::ConstPtr& ground_pose, const messages::wheel_velocities::ConstPtr& wheel_velocities){
					

			received_msg_num++;
			v_left=(double)wheel_velocities->v_left;
			v_right=(double)wheel_velocities->v_right;


			
			tf::Pose pose;
  			tf::poseMsgToTF(ground_pose->pose, pose);
  			double yaw_angle = tf::getYaw(pose.getRotation());

			// optitrack returns positive radians from 0 to 180°, then negative from 180° to 0°
			// when it returns negative ones I have to convert them as positive: 180°+(180°+optitrack_angle)

			if(yaw_angle<0){
				double neg_yaw_angle=yaw_angle;
				yaw_angle=(2*3.14159)+neg_yaw_angle;
			}


			if(is_initial_point){
				last_point.setXYTheta(ground_pose->pose.position.x, ground_pose->pose.position.y, yaw_angle);
				initial_angle=yaw_angle;
				is_initial_point=false;
				last_lap_time=ros::Time::now().toSec();
			}
			else if (v_left!=0.0 && v_right!=0.0){			
				last_point=actual_point;			
			}
			
			if(v_left==0.0 && v_right==0.0){
				last_lap_time=ros::Time::now().toSec();
			}
			


			actual_point.setXYTheta(ground_pose->pose.position.x, ground_pose->pose.position.y, yaw_angle);
						
			ROS_INFO("GET: initial_angle: %f, actual_angle: %f, last_angle: %f, v_left: %f, v_right: %f", initial_angle, actual_point.getTheta(), last_point.getTheta(), v_left, v_right);
			
			if(v_left!=0.0 && v_right!=0.0){ // the bag starts with some 0s speeds that I want to ignore
				if (filter_optitrack_data(received_msg_num)){
					ROS_INFO("SET: initial_angle: %f, actual_angle: %f, last_angle: %f", initial_angle, actual_point.getTheta(), last_point.getTheta());
					// we are in a stable window
					
					last_time=current_time;
					current_time=ros::Time::now();
					final_angle=actual_point.getTheta();

					laps_number(actual_point, last_point, v_left, v_right);
					ROS_INFO("Laps number: %f", passed_by_start);

					update_values(v_right, v_left, (current_time-last_time).toSec());

					estimation(); // should be called only at the end
				}
				else {
					// jumps the istants with problems
					last_time=current_time;
					current_time=ros::Time::now();
				}
			}
			else{
				// jumps the istants with velocity=0
				filter_optitrack_data(received_msg_num);
				last_time=current_time;
				current_time=ros::Time::now();

			}
			
		}


		double filter_optitrack_data(double n){
			// I need to apply a filter on optitrack's data to stabilize them
				if (v_left<0 && v_right>0 || anti_clock_wise){
				
					if(clock_wise==false){
					// only a direction per bag is permitted: if another has been detected, it means there is a data oscillation
						//ROS_INFO("anti clockwise branch. actual_angle: %f, last_angle: %f", actual_point.getTheta(), last_point.getTheta());
						anti_clock_wise=true;
						clock_wise=false;

						if (last_point.getTheta()>=actual_point.getTheta() && actual_point.getTheta()<0.3 && last_point.getTheta()>6.0){
							// anti clock-wise && angle is not increasing && passing on 0°
							// do nothing!
							ROS_INFO("doing nothing: last_point: %f > actual_point:%f", last_point.getTheta(), actual_point.getTheta());				
						}							
						else if(last_point.getTheta()>=actual_point.getTheta()){
							// anti clock-wise && angle is not increasing && not near the 0°
							ROS_INFO("exchange points: last_point: %f > actual_point: %f", last_point.getTheta(), actual_point.getTheta());
							actual_point=last_point;
						}
						else if(actual_point.getTheta()>last_point.getTheta() && abs((actual_point.getTheta()-last_point.getTheta()))>0.1 && actual_point.getTheta()>6.0){
							// anti c-w && angle is increasing but because the new came back to 360° instead of increasing
							ROS_INFO("exchange points (case2): last_point: %f > actual_point: %f", last_point.getTheta(), actual_point.getTheta());
							actual_point=last_point;
						}
							
					}
					else{
						return 0;	// means we are in a non-stable window of data (clock-wise/anti clock-wise alternate)	
					}
				}
				else if (v_left>0 && v_right<0 || clock_wise){
					if(anti_clock_wise==false){ 	
					// only a direction per bag is permitted: if another has been detected, it means there is a data oscillation
						anti_clock_wise=false;
						clock_wise=true;
							
						if(last_point.getTheta()<=actual_point.getTheta() && actual_point.getTheta()>6.0 && last_point.getTheta()<0.3){
								// clock-wise && angle is not decreasing && near the 0°
								// do nothing!
								ROS_INFO("doing nothing: last_point: %f < actual_point: %f", last_point.getTheta(), actual_point.getTheta());				
						}
						else if(last_point.getTheta()<=actual_point.getTheta()){
							// clock-wise && angle is not decreasing && not near the 0°
							actual_point=last_point;
							ROS_INFO("exchange points: last_point: %f < actual_point: %f",last_point.getTheta(),actual_point.getTheta());				
						}else if(actual_point.getTheta()<last_point.getTheta() && abs((actual_point.getTheta()-last_point.getTheta()))>0.1){
							//  c-w && angle is decreasing but because the new came back to 0° instead of decreasing to 360°
							ROS_INFO("exchange points (case2): last_point: %f > actual_point: %f", last_point.getTheta(), actual_point.getTheta());
							actual_point=last_point;
						}
					}
					else{
						return 0; // means we are in a non-stable window of data (clock-wise/anti clock-wise alternate)
					}
				}
				else if(v_left>0 && v_right>0 || v_left<0 && v_right<0){
					// means I am moving linearly and I want to estimate alpha: I don't care what happens to angles		
					return 1;
				}
				else{
					// velocities are zero because the robot is not moving yet, but i have to avoid that the angle becames smaller (or bigger, it depends on the rotational direction) than the initial one.
					return 0; // wheel velocities are zero: do nothing
				}

			

			if(n>50 && v_left!=0 && v_right!=0){
				return 1; //means it can go on and update the values
			}
			else {
				return 0;  //means we are in a non-stable window of data	
			}
		}


		// returns how many times the position is passed by 0°
		void laps_number(position p1, position p2, double v_left, double v_roght){
				if (actual_point.getTheta()==initial_angle && actual_point.getTheta()!=last_point.getTheta()){
					increase_laps();
				}
				else if (v_left<0 && v_right>0){
				//ROS_INFO("ANTI CLOCK-WISE (V_LEFT: %f, V_RIGHT: %f), \n initial angle: %f", v_left, v_right, initial_angle);
				// anti clock-wise
					if(actual_point.getTheta()>initial_angle && initial_angle>last_point.getTheta()){
						increase_laps();
					}
					else if (initial_angle>last_point.getTheta() && last_point.getTheta()>actual_point.getTheta()){
						increase_laps();
					}
					else if (last_point.getTheta()>actual_point.getTheta() && actual_point.getTheta()>initial_angle && last_point.getTheta()>6.0 && actual_point.getTheta()<0.3){
						increase_laps();
					}		
				}
				else if(v_left>0 && v_right<0){
				//ROS_INFO("CLOCK-WISE (V_LEFT: %f, V_RIGHT: %f)", v_left, v_right);
				// clock-wise
					if(last_point.getTheta()>initial_angle && initial_angle>actual_point.getTheta()){
						increase_laps();
					}
					else if (initial_angle>actual_point.getTheta() && actual_point.getTheta()>last_point.getTheta()){
						increase_laps();
					}
					else if (actual_point.getTheta()>last_point.getTheta() && last_point.getTheta()>initial_angle){
						increase_laps();
					}
				}					
		}

		// increase laps count only if physical time is occured to do a lap
		void increase_laps(){
				double now=ros::Time::now().toSec();
				double sup=now-last_lap_time;
				ROS_INFO("INCREASE_LAPS: now: %f, last_lap: %f, min_lap: %f\n Difference: %f", now, last_lap_time, min_lap_time, sup);
				if(sup>=min_lap_time){	
					passed_by_start++;
					last_lap_time=ros::Time::now().toSec();
				}
		}

		// updates values that need to be updated at each iteration
		void update_values(double v_left, double v_right, double dt){
			int_vr_dt+=v_right*dt;
			int_vl_dt+=v_left*dt;
			//ROS_INFO("int_vr_dt: %f, int_vl_dt: %f", int_vr_dt, int_vl_dt);
			dist+=last_point.distance(last_point, actual_point);
			phi=last_point.rotated_angle(initial_angle, final_angle, passed_by_start, anti_clock_wise); 
			double converted_phi=phi*57.2958; // grades
			ROS_INFO("ROTATED ANGLE: %f --->  %f", converted_phi, phi); 		
		}		
			
		// estimate the parameters and publish on a odom_param message	
		void estimation(){
			
			alpha= estimate_alpha();
			minus_y_icr=estimate_minusyicr();
			messages::odom_param msg;
			msg.header.seq=seq++;
			msg.header.stamp=ros::Time::now();
			msg.alpha=alpha;
			msg.minus_y_icr=minus_y_icr;
			pub.publish(msg);
			
		}

		// estimates the alpha parameter
		double estimate_alpha(){
			double denom=int_vr_dt+int_vl_dt;
			return ((2*dist)/denom);
		}

		// estimates the minus y icr parameter
		double estimate_minusyicr(){
			double denom=2*phi;
			double num=int_vr_dt-int_vl_dt;
			return (num/denom);
		}

	private:
		ros::Publisher pub;
		ros::Subscriber sub_ground_pose;
		ros::Subscriber sub_wheel_velocities;

		ros::NodeHandle n;

		position last_point;
		position actual_point;
		ros::Time current_time, last_time; 

		double dist;
		double phi;
		double alpha;
		double minus_y_icr;
		double v_left;
		double v_right;
		double int_vl_dt;
		double int_vr_dt;
		double seq;
		bool is_initial_point;
		

		double initial_angle;
		double final_angle;
		double passed_by_start;

		double received_msg_num;
		double clock_wise;
		double anti_clock_wise;

		double min_lap_time;										//fixed param!! CHANGE IT! Now is 1 sec
		double last_lap_time;
};


int main(int argc, char** argv){
	ros::init(argc, argv, "param_estimator");
	ros::NodeHandle n;
	param_estimator estimator;

	typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, messages::wheel_velocities> MySyncPolicy;
	message_filters::Subscriber<geometry_msgs::PoseStamped> sub_ground_pose(n,"/Robot_2/ground_pose_conv",10);	
	message_filters::Subscriber<messages::wheel_velocities> sub_wheel_velocities(n,"/wheel_velocities",10);	

	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_ground_pose, sub_wheel_velocities);
	sync.registerCallback(boost::bind(&param_estimator::callback, &estimator, _1, _2));

	ros::spin();
	return 0;
}

