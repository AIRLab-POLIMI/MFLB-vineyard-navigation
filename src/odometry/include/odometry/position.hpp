
class position {

	public:
		position();
		
		// sets all the parameters
		void setXYTheta(double coord_x, double coord_y, double ang_theta);

		double getX();

		double getY();

		double getTheta();

		// returns the distance between the two points p1 and p2
		double distance(position p1, position p2);

		// returns the rotated angle between two positions
		double rotated_angle(position p1, position p2);

	private:
		double x;
		double y;
		double theta;

};
