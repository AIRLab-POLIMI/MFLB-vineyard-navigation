#include <cmath>
#include "position.hpp"
#include "ros/ros.h"

		
		// sets all the parameters
		void position::setXYTheta(double coord_x, double coord_y, double ang_theta){
			x=coord_x;
			y=coord_y;
			theta =ang_theta;
		}

		double position::getX(){
			return x;		
		}

		double position::getY(){
			return y;		
		}

		double position::getTheta(){
			return theta;		
		}

		// returns the distance between the two points p1 and p2
		double position::distance(position p1, position p2){
			return sqrt(pow((p2.getX()-p1.getX()),2)+pow((p2.getY()-p1.getY()),2));
		}



		// returns the rotated angle between two positions (absolute)
		double position::rotated_angle(double initial_angle, double final_angle, double passed_by_zero, bool anti_clock_wise){
			double phi;
			double sup_1, sup_2, sup_3;
			if(anti_clock_wise){ 
				// anti clock-wise
				if(initial_angle<final_angle){
					sup_1=final_angle-initial_angle;
					sup_2=6.28319*passed_by_zero;
					ROS_INFO("N_LAPS*360=%f, final-initial=%f", sup_2, sup_1);
					phi=sup_1+sup_2;
				}
				else if(initial_angle>final_angle){
					sup_1=6.28319-initial_angle;
					sup_2=sup_1+final_angle;
					sup_3=6.28319*passed_by_zero;
					ROS_INFO("N_LAPS*360=%f, 6.28-initial+final=%f", sup_3, sup_2);
					phi=sup_2+sup_3;
				}
				else if(initial_angle==final_angle){
					phi=6.28319*passed_by_zero;
				}
			}
			else{
				// clock_wise
				if(initial_angle<final_angle){
					sup_1=6.28319-final_angle;
					sup_2=sup_1+initial_angle;				
					sup_3=6.28319*passed_by_zero;
					phi=sup_2+sup_3;
					ROS_INFO("N_GIRI*360=%f, 360-fin+ini=%f", sup_3, sup_2);
				}
				else if(initial_angle>final_angle){
					sup_1=initial_angle-final_angle;
					sup_2=6.28319*passed_by_zero;
					phi=sup_1+sup_2;
					ROS_INFO("N_LAPS*360=%f, ini-fin=%f", sup_2, sup_1);			
				}
			}
			return phi;
		}

