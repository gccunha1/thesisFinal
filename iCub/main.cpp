#include <cstdio>
#include <yarp/os/SystemClock.h>
#include <yarp/os/Property.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/sig/Vector.h>
#include <iostream>
#include <vector>
#include "Robot.hpp"

//using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;



int main(int argc, char *argv[]) {
	GazeController g;

    g.setFixationPoint(stod(argv[1]), stod(argv[2]), stod(argv[3]));
	
    return 0;

}