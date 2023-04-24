#ifndef POSITION
#define POSITION

class position {

	public:
		
		// sets all the parameters
		void setXYTheta(double coord_x, double coord_y, double ang_theta);

		double getX();

		double getY();

		double getTheta();

		// returns the distance between the two points p1 and p2
		double distance(position p1, position p2);

		// returns the rotated angle between two positions
		double rotated_angle(double p1, double p2, double passed_by_zero, bool anti_clock_wise);



	private:
		double x;
		double y;
		double theta;

		

};
#endif 
