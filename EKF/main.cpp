#include "ExtendedKalmanFilter.hpp"
#include "ActiveSelectionEKF.hpp"

#include <nlopt.hpp>
#include <vector>

#define NRJOINTS 3
#define NRPARAMETERS 4*NRJOINTS



int main(int argc, char const *argv[])
{
	std::vector<double> dhParameters;
	
	ExtendedKalmanFilterBodySchema ekf;
	iCubRobot robot;
	nlopt::opt opt(nlopt::GN_DIRECT_L, ekf.getJointNr());

	ActiveSelectionEKF alEKF(&ekf, &opt, &robot);

	alEKF.optimize(dhParameters);
	
	//lowerBound[0] = -94.5;
	//lowerBound[1] = 0;
	//lowerBound[2] = -36.27;
	//lowerBound[3] = 15.385;
	//lowerBound[4] = -90.0;
	//lowerBound[5] = -90.0;
	//lowerBound[6] = -19.8;
	//upperBound[0] = 9.45;
	//upperBound[1] = 160.8; 
	//upperBound[2] = 79.5;
	//upperBound[3] = 105.88;
	//upperBound[4] = 90.0;
	//upperBound[5] = 0;
	//upperBound[6] = 39.6; 
	
	
	

	

	/*
	*/
	return 0;
}




