#ifndef _ROBOT_HPP_
#define _ROBOT_HPP_


#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>

#include <yarp/os/SystemClock.h>
#include <yarp/os/Property.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/sig/Vector.h>
#include <vector>
#include <yarp/os/Time.h>
#include <yarp/math/Math.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GazeControl.h>
#include "../utils/utils.hpp"
#include "../iCubWorld/simobjloader.h"
#include "../iCubWorld/worldclient.hpp"
#include "../iCubVision/ArucoDetector.hpp"
#include "../iCubLimb/iCubLimb.hpp"

#include <yarp/os/all.h>
#include <stdio.h>
#include <string>
#include <iostream>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


Eigen::Matrix4d robot2worldFrame(Eigen::Matrix4d T);
Eigen::Matrix4d world2robotFrame(Eigen::Matrix4d T);
Eigen::Matrix4d cameraPose2matrix(std::vector<double> &tvec, std::vector<double> &rvec);
Matrix eigenMatrix2yarp(Eigen::MatrixXd T);




class GazeController {
public:
	GazeController();
	bool setFixationPoint(double x, double y, double z);
	~GazeController();
	bool getLeftEyePose(Vector &x, Vector &o);
private:
	IGazeControl *igaze = NULL;
	PolyDriver *clientGazeCtrl = NULL;
};

class MarkersController {
public:
	void inputHandPose(Eigen::Matrix4d T);
	Eigen::Matrix4d getMarkerPose(std::string id);
	MarkersController();
	~MarkersController();
private:
	iCubWorldClient client;
    SimSMarker *palmMarker;
    SimSMarker *backMarker;
    SimSSph *palmSph;
    SimSSph *backSph; 
    std::vector<int> ind;
	const double offset = 0.12;
	Eigen::Vector3d tMatrix2worldAngles(Eigen::Matrix4d T);
};

class iCubRobot {

	public:
		iCubRobot();
		~iCubRobot();
		bool move(std::vector<double> encValues, std::string bodyPartID);
		bool look(double x, double y, double z);
		int getJointCount();
		bool getRightArmPose(std::vector<double> &z, std::vector<double> &o, std::vector<double> jointAngles, std::vector<double> &realPose, double &reliability, int jnt, bool predict = false);
		bool getRightArmPose(std::vector<double> &z, std::vector<double> &o, std::vector<double> &realPose, double &reliability, int jnt = 7, bool predict = false);
		bool getRightArmJointValues(std::vector<double> &thetas);
		std::vector<double> getDHparameters(int njoints);
		Eigen::Matrix4d getTmatrix(std::vector<double> jointAngles, int jnt);
	private:
		iCub::iKin::iCubEye *eye;
		GazeController gaze;
		ArucoDetector detector;
		int jnts;
		bool useMarkers = true;
		std::vector<double> dhParameters{ 
			0.32,       0,     M_PI/2,    0,
			0,    -0.055,     M_PI/2,    -M_PI/2,
			-0.233647,  -1.433,     M_PI/2,    -15*M_PI/180 - M_PI/2,
			0, -1.0774,     M_PI/2,    -M_PI/2,
			0,       0,    -M_PI/2,    -M_PI/2,
			-0.15, -1.5228,    -M_PI/2,    -M_PI/2-15*M_PI/180,
			0.15,       0,     M_PI/2,    0,
			0,  -1.373,     M_PI/2,    -M_PI/2,
			0,       0,     M_PI/2,    M_PI/2,
			0.625,      0.16,          0,    M_PI
		};
		std::vector<double> dhParametersiKin{ 
			0, -1.0774,     M_PI/2,    -M_PI/2,
			0,       0,    -M_PI/2,    -M_PI/2,
			-0.15, -1.5228,    -M_PI/2,    -M_PI/2-15*M_PI/180,
			0.15,       0,     M_PI/2,    0,
			0,  -1.373,     M_PI/2,    -M_PI/2,
			0,       0,     M_PI/2,    M_PI/2,
			0.625,      0.16,          0,    M_PI
		};
		GenericRightArm *arm = NULL;

		Vector encoders;
		Vector command_position;
		Vector command_velocity;

		Property options;
		PolyDriver rightArmControl;
		
		IPositionControl *pos;
		IVelocityControl *vel;
		IEncoders *enc;

		MarkersController markers;

		Eigen::Matrix4d getLeftEyeTmatrix();
		
};



#endif