#include "ros/ros.h"
#include <ros/console.h>
#include <sstream>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "messages/odom_pointcloud_sync.h"
#include "messages/controller.h"
#include "messages/filtering_instr.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <boost/thread.hpp>
#include "Robot.hpp"
#include "fre_row_navigation/client.hpp"
#include "turn/client.hpp"
#include "end_row_navigation/client.hpp"
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread/thread.hpp>
#include <ros/callback_queue.h>
#include <fstream>


class Handler{

	private: 
		bool work;
		ros::Subscriber sub_controller;
		ros::Publisher filtering_instr_pub;
		ros::NodeHandle n;
		Robot robot;
		int in_row_result, end_of_row_result;	
		RowCrawlClient in_client;
		ParallelLineNavigationClient end_client;
		TurnClient turn_client;
		std::string in_row_server_name, end_row_server_name, turn_server_name, lidar_topic_name, file_path;
		std::string turning_side;
		int turns_count;
		tf2::Matrix3x3 m;
		double points_height;
		double lateral_dist;
		double row_length;
		ros::CallbackQueue my_callback_queue;
		std::ofstream obj1;
		std::ifstream file_input;
		ros::Time time;
		std::string storage;
	
		int read_tojump_rows;
		bool path_from_file;
		

	public:
		Handler(){

			work=false;
			n.getParam("/sim_handler/lidar_topic_name", lidar_topic_name);
			ROS_INFO("TOPIC LIDAR NAME: %s", lidar_topic_name.c_str());
			
			sub_controller= n.subscribe("controller", 50, &Handler::sub_controller_callback, this);

			filtering_instr_pub=n.advertise<messages::filtering_instr>("filtering_instr", 1);

			n.getParam("/sim_handler/in_row_server_name", in_row_server_name);
			n.getParam("/sim_handler/turn_server_name", turn_server_name);
			n.getParam("/sim_handler/end_row_server_name", end_row_server_name);
			n.getParam("/sim_handler/file_path", file_path);
			n.getParam("/sim_handler/storage", file_path);

			end_of_row_result=1;
			turns_count=0;
			path_from_file=false;
			
			read_tojump_rows=0;
			
			obj1.open(storage+"/nav_times/file_change_row.csv", std::ofstream::trunc);
			file_input.open(file_path, std::ifstream::in);

		}



		// Listen to topic published by controller node
		void sub_controller_callback(const messages::controller::ConstPtr& msg){
			work=msg->start;
			row_length=msg->row_length;
		
			if(msg->path_from_file==false)	{
				robot.set_actual_turning_side(msg->initial_turning_side);
				path_from_file=false;
			}
			else path_from_file=true;
			
			if(robot.get_actual_turning_side()!=msg->initial_turning_side) turns_count=0;		
		}


		void handle_work(){
			/* in_row returns 0 when it reaches the end of the row, end_of_row() returns 1 when it is at the beginning of the row */	
			

			if (work && end_of_row_result){
				// I am in a row: start procedure for navigation in row
				in_row_result=in_row();
				end_of_row_result=0;		
			}
			else if(work && in_row_result){
				// I am at the end of the row: start procedure for navigation at the end of the row
				time=ros::Time::now();
				end_of_row_result=!end_of_row();
				obj1<<(ros::Time::now()-time).toSec()<<"\n";			
				in_row_result=0;	
			}
			else if(!work){
				ROS_INFO("The robot has been stopped!");		
			}
		
		}

		// Procedure for the in row navigation
		int in_row(){
			in_row_navigation_algo();
			return 1;	 // robot is out of a row
		}


		// This method starts the FRE21 client for free space cone detection & in_row navigation
		void in_row_navigation_algo(){
			
			if(path_from_file==false){
				in_client.start_client(n, 20.0, 2.0, in_row_server_name);	// max_speed , min_row_length 
				read_tojump_rows=-1;		
			}
			else if(path_from_file==true){
				// read data for navigation
				if(file_input.good()){
					char c1 = file_input.get();
					char c2 = file_input.get();
					read_tojump_rows=c1-'0';

			
					if(c2=='L'){
						robot.set_actual_turning_side("left");
						turns_count=0;
					}
					else if(c2=='R'){
						robot.set_actual_turning_side("right");
						turns_count=0;
					}
					// set turning side has to be done before starting the client because start_client is a blocking method
					
	
					in_client.start_client(n, 20.0, 2.0, in_row_server_name);
						
							
					c1=file_input.get();	// to get rid of the char between two subsequent configurations
					
				}
				else{
					ROS_INFO("Error while reading input file.");
				}
			}
		}


		// Procedure for the end of row navigation (change corridor navigation)
		int end_of_row(){
			int something_went_wrong= 0;
			// if turn is ok, start end of row navigation
			if(turn(0, next_turning_side())){
				is_end_row_active(true);

				/*ACTION TO NAVIGATE AT THE END OF A ROW*/
				if (end_client.start_client(n, 5.0, 2.0, end_row_server_name, read_tojump_rows)){
					is_end_row_active(false);
					if(turn(1, next_turning_side())){
						turns_count++;
					}
					else{
						ROS_INFO("Turn at the entrance of a new row went wrong.");
						something_went_wrong=1;				
					}
				}
				else{
					is_end_row_active(false);
					ROS_INFO("Something went wrong during navigation at end row.");	
					something_went_wrong=1;	
				}


			}
			else{
				ROS_INFO("Turn at the exit of a new row went wrong.");		
				something_went_wrong=1;			
			}

			return something_went_wrong;
		}
		
		// Publishes a message with the instruction about the filtering operations
		void is_end_row_active(bool status){

			int side_to_remove;
			if(next_turning_side()=="right") side_to_remove=1;
			else side_to_remove=-1;
			
			messages::filtering_instr msg;
			msg.radius_filtering=true;
			msg.zone_filtering=status;
			msg.downsampling_filtering=true;
			msg.side_to_remove=side_to_remove;
			filtering_instr_pub.publish(msg);
		}

		// This method calls a Turn client that contact the action to turn the robot
		int turn(int where, std::string turning_side){
			if(turning_side=="left")
				return turn_client.start_client(1.57, 0.5, where, turn_server_name);
			else
				return turn_client.start_client(-1.57, 0.5, where, turn_server_name);
		}

		// This method returns the next turning side
		std::string next_turning_side(){
			if (turns_count==0)
				return robot.get_actual_turning_side();
			else if(turns_count%2==0){	
				return robot.get_actual_turning_side();
			}	
			else{
				if(robot.get_actual_turning_side()=="right")
					return "left";
				else
					return "right";
			}
				
		}

};


int main(int argc, char **argv){ 
	ros::init(argc, argv, "handler");
	ros::NodeHandle n;


	Handler handler;

	ros::Rate r(50); // 50 hz
	while (ros::ok())
	{
		handler.handle_work();
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
