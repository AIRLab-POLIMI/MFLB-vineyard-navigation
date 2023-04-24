// C++ program to find the regression line using least square in 2D
#include <bits/stdc++.h>
#include "least_square.hpp"
#include <math.h>
 

	// Function to calculate k
	void Least_square::calculateK()
	{

		// sum of array x
		double sx = std::accumulate(X.begin(), X.end(), 0.0);
		
		k=sx/n;

	}

	// Function to calculate m
	void Least_square::calculateM()
	{

		// sum of array x
		double sx = std::accumulate(X.begin(), X.end(), 0.0);
		
		// sum of array y
		double sy = std::accumulate(Y.begin(), Y.end(), 0.0);

		// for sum of product of x and y
		double sxsy = 0;

		// sum of square of x
		double sx2 = 0;
		double sy2=0;
		for(int i = 0; i < n; i++)
		{
			sxsy += X[i] * Y[i];
			sx2 += X[i] * X[i];
			sy2+=Y[i]*Y[i];
		}
		double den=(n * sx2) - (sx * sx);
		
		if(den != 0){
			//m =( (n * sxsy)-( sx * sy) )/((n * sx2) - (sx * sx));  	// case y=mx+q
			m =( (n * sxsy)-( sx * sy) )/((n * sy2) - (sy * sy));		// case x=my+q
		}
		else{
			// special case: the line is parallel to y axis. Her equation is x=k
			m = std::numeric_limits<double>::quiet_NaN();
		}

	}

	// Function to find the least regression line
	void Least_square::leastRegLine()
	{

		// Finding m
		calculateM();
		
		if(isnan(m)==false){

			double sumX = std::accumulate(X.begin(), X.end(), 0.0);
			double meanX = sumX / n;
			double sumY = std::accumulate(Y.begin(), Y.end(), 0.0);
			double meanY = sumY / n;
			// Calculating q
			//q = meanY - (m * meanX); 	// case y=mx+q
			q = meanX - (m * meanY);	// case x=my+q
			//q=((sy*sx2)-(sx*sxsy))/((n*sx2)-(sx*sx));   equivalent
		}
		else{
			// special case: the line is parallel to y axis. Her equation is x=k
			q = std::numeric_limits<double>::quiet_NaN();
		}
	}

	// Function to find coefficient of the regression line
	std::vector<double> Least_square::getRegressionLine(std::vector<double> X_input, std::vector<double>Y_input){
	
		X.resize(0);
		Y.resize(0);
		
		X=X_input;
		Y=Y_input;

		n=(int) X.size();


		leastRegLine();	
		
		std::vector<double> coeff;
		if(isnan(m)==false && isnan(q)==false){
			coeff.push_back(m);
			coeff.push_back(q);
			ROS_INFO("m: %f  q: %f", m, q);
		}
		else{
			calculateK();
			coeff.push_back(k);
			coeff.push_back(std::numeric_limits<double>::quiet_NaN());
		}
		return coeff;
		
	}

	// Function to project a point on the model
	std::vector<double> Least_square::getPointOnModel(pcl::PointXYZ point, double m_in, double q_in){
		proj_point.resize(0);
		// calculate line (m'x+q') perpendicular to model and passing through the point
		mp=-(1/m_in); 
		//qp= point.y + ((-mp)*point.x);			// case y=mx+q
		qp= point.x - (mp*point.y);			// case x=my+q
		
		
		// calculate intersection between line and model: founded point is the projection
//		x_intersection=(qp-q_in)/(m_in-mp);		// case y=mx+q
//		y_intersection=m_in*x_intersection+q_in;	// case y=mx+q
		
		y_intersection=(qp-q_in)/(m_in-mp);		// case x=my+q
		x_intersection=m_in*y_intersection+q_in;	// case x=my+q
		
		
		proj_point.push_back(x_intersection);
		proj_point.push_back(y_intersection);
		return proj_point;
	}
 
 	std::vector<double> Least_square::getPointOnYParallelModel(pcl::PointXYZ point, double k_in){
 		proj_point.resize(0);
		
		proj_point.push_back(k_in);	// x value is equal to k
		proj_point.push_back(point.y);  // y value remains the same
		return proj_point;
 	}

