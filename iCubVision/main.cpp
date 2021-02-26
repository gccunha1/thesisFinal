#include <stdio.h>
// Get all OS and signal processing YARP classes
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include "ArucoDetector.hpp"
using namespace yarp::os;
using namespace yarp::sig;
int main() {
	Eigen::Matrix4d T;
	ArucoDetector detector(true);
	int id;

	while(1) {
		detector.getPose(T, id);
	}

	return 0;
}