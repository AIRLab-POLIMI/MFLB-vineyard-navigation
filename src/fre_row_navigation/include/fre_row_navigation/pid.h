#pragma once
#include <iostream>
#include "ros/ros.h"

class PID {
public:
	PID(double p, double i, double d)
	    : p(p), i(i), d(d), lastError(0.0), integrator(0.0) {}
	double calculate(double error);
	void reset();
	double p;
	double i;
	double d;
	double lastError;
	double integrator;
};
