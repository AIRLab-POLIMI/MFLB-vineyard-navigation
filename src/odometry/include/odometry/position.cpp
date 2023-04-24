#include <cmath>
#include "position.h"

//class position {

	//public:
		
		
		// sets all the parameters
		void setXYTheta(double coord_x, double coord_y, double ang_theta){
			x=coord_x;
			y=coord_y;
			theta =ang_theta;
		}

		double getX(){
			return x;		
		}

		double getY(){
			return y;		
		}

		double getTheta(){
			return theta;		
		}

		// returns the distance between the two points p1 and p2
		double distance(position p1, position p2){
			return sqrt(pow((p2.getX()-p1.getX()),2)+pow((p2.getY()-p1.getY()),2));
		}

		// returns the rotated angle between two positions
		double rotated_angle(position p1, position p2){
			return p2.getTheta()-p1.getTheta();
		}

	//private:
	//	double x;
	//	double y;
	//	double theta;


//};

//int main(int argc, char** argv){}
