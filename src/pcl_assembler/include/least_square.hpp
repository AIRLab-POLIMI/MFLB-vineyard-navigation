#ifndef LEAST_SQUARE
#define LEAST_SQUARE

#include "ros/ros.h"
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>

class Least_square{

	public:
		void calculateM();
		void leastRegLine();
		std::vector<double> getRegressionLine(std::vector<double> X_input, std::vector<double>Y_input);
		std::vector<double> getPointOnModel(pcl::PointXYZ point, double m, double q);
		std::vector<double> getPointOnYParallelModel(pcl::PointXYZ point, double k_in);
		void calculateK();
	
	private:
		std::vector <double> X, Y;
		int n;
		std::vector<double> proj_point;
		double q,m,k;
		double mp, qp;
		double x_intersection, y_intersection;
		//double sx, sy, sxsy, sx2;
};

#endif 
