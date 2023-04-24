#include <fre_row_navigation/pid.h>
#include <iostream>
#include "ros/ros.h"

double PID::calculate(double error) {
	// error= difference between my signal and the ideal one. Difference between my actual position and the position in which I would like to be
	integrator += error;
//						  proportional to the error
//						  |	      proportional to the error's integral
//						  |	         |		       proportional to the error derivative
//       		          |          |                 |		       
//		 				  v	         v		           v

	double result = p * error + i * integrator + d * (error - lastError);
	lastError = error;


	return result;
}

void PID::reset(){
	integrator=0;
	lastError=0;
}
