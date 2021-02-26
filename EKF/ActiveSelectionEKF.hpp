#ifndef ACTIVE_SELECTION_HPP
#define ACTIVE_SELECTION_HPP


#include "ExtendedKalmanFilter.hpp"
#include "../iCub/Robot.hpp"
#include <vector>
#include <nlopt.hpp>
#include <fstream>
#include <iostream>

typedef struct _dataOpt {
	ExtendedKalmanFilterBodySchema *ekf;
	std::vector<double> boundsOffset;
	bool costSensitive;
	float costCoef;
	std::vector<double> prevTheta;
} dataOpt;


class ActiveSelectionEKF {
public:
	ActiveSelectionEKF(ExtendedKalmanFilterBodySchema *ekf, nlopt::opt *opt, iCubRobot *robot);
	~ActiveSelectionEKF();
	static double f(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
	void optimize(std::vector<double> &dhParameters);
	double computeError(double e, int n, std::vector<double> joints);
private:
	ExtendedKalmanFilterBodySchema *ekf;
	nlopt::opt *opt;
	iCubRobot *robot;
	bool activeLearning = true;
	bool costSensitive = false;
	float costCoef;
	int maxIter = 50;
	int maxTime = 1;
	int inputDim = 3;
	std::vector<double> lowerBounds;
	std::vector<double> upperBounds;
	std::vector<double> boundsOffset;
	std::ofstream outputfile;
	dataOpt dtOpt;
	

	bool readFileParameters();
};

#endif
