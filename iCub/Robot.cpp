#include "Robot.hpp"
#include <Eigen/Dense>
#include <exception>
#include <iostream>
#include <unistd.h>





class initFailed: public std::exception {
  virtual const char* what() const throw() {
    return "Failure initializing iCub simulator.";
  }
} initException;




GazeController::GazeController() {
	Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    option.put("local","/client/gaze");
    clientGazeCtrl = new PolyDriver(option);

    if (clientGazeCtrl->isValid()) {
       clientGazeCtrl->view(igaze);
    }
    igaze->setStabilizationMode(false);
    igaze->setSaccadesMode(false);

}

bool GazeController::setFixationPoint(double x, double y, double z) {
	yarp::sig::Vector fp(3);
    fp[0]=x;                                    // x-component [m]
    fp[1]=y;                                    // y-component [m]
    fp[2]=z;                                    // z-component [m]
    	
    if(!igaze->lookAtFixationPointSync(fp))
    	return false;             // request to gaze at the desired fixation point and wait for reply (sync method)
    igaze->waitMotionDone();                        // wait until the operation is done
    yarp::sig::Vector xf;
    //igaze->getFixationPoint(xf);                     // retrieve the current fixation point
    //std::cout<<"final error = "<<yarp::math::norm(fp-xf)<<std::endl;       // return a measure of the displacement error
    return true;
}

bool GazeController::getLeftEyePose(Vector &x, Vector &o) {
	if(igaze->getLeftEyePose(x,o)) {
		std::cout << "Pose: " << std::endl;
		for (int i = 0; i < 3; ++i)
		{
			std::cout << x[i] << std::endl;
		}
		for (int i = 0; i < 4; ++i)
		{
			std::cout << o[i] << std::endl;
		}
		return true;
	}
	return false;
}

GazeController::~GazeController() {
	delete clientGazeCtrl;
}

MarkersController::MarkersController() {
	palmMarker = new SimSMarker(0.0,0.0,0.0,0.0,0.0,0.0, "marker4_5cm.x", "marker1_4_4_50.bmp", "front", offset);
    backMarker = new SimSMarker(0.0,0.0,0.0,0.0,0.0,0.0, "marker4_5cm.x", "marker2_4_4_50.bmp", "back", offset);
    //palmSph = new SimSSph(0,0,0,0,0,0,1,0,0,0.01);
    //backSph = new SimSSph(0,0,0,0,0,0,1,0,0,0.01);
    ind.resize(10, 0);
    client.sendCommand(palmMarker->makeObjectBottle(ind, false));
	client.sendCommand(backMarker->makeObjectBottle(ind, false));    
	//client.sendCommand(palmSph->makeObjectBottle(ind, false));
	//client.sendCommand(backSph->makeObjectBottle(ind, false));  
}

MarkersController::~MarkersController() {
	delete palmMarker;
	delete backMarker;
	//delete palmSph;
	//delete backSph;
}

void MarkersController::inputHandPose(Eigen::Matrix4d T) {
	Eigen::Matrix4d worldMatrix = robot2worldFrame(T), markerPoseHandFrame, markerPoseWorldFrame;
	Eigen::Vector3d markerOrientation; // x y z - moving axis

	markerPoseWorldFrame = worldMatrix * palmMarker->getPose(); 

	//std::cout << T << std::endl;

	markerOrientation = tMatrix2worldAngles(markerPoseWorldFrame);
	palmMarker->setObjectPosition(markerPoseWorldFrame(0,3)/10,markerPoseWorldFrame(1,3)/10,markerPoseWorldFrame(2,3)/10);
	palmMarker->setObjectRotation(markerOrientation[0], markerOrientation[1], markerOrientation[2]);
	//palmSph->setObjectPosition(markerPoseWorldFrame(0,3)/10,markerPoseWorldFrame(1,3)/10,markerPoseWorldFrame(2,3)/10);
	
	markerPoseWorldFrame = worldMatrix * backMarker->getPose(); 

	markerOrientation = tMatrix2worldAngles(markerPoseWorldFrame);
	backMarker->setObjectPosition(markerPoseWorldFrame(0,3)/10,markerPoseWorldFrame(1,3)/10,markerPoseWorldFrame(2,3)/10);
	backMarker->setObjectRotation(markerOrientation[0], markerOrientation[1], markerOrientation[2]);
	//backSph->setObjectPosition(markerPoseWorldFrame(0,3)/10,markerPoseWorldFrame(1,3)/10,markerPoseWorldFrame(2,3)/10);

	client.sendCommand(palmMarker->moveObjectBottle());
	client.sendCommand(backMarker->moveObjectBottle());
	client.sendCommand(palmMarker->rotateObjectBottle());
	client.sendCommand(backMarker->rotateObjectBottle());

	//client.sendCommand(palmSph->moveObjectBottle());
	//client.sendCommand(backSph->moveObjectBottle());
}

Eigen::Matrix4d MarkersController::getMarkerPose(std::string id) {
	if(id == "palm") {
		return palmMarker->getPose();
	} else if(id == "back") {
		return backMarker->getPose();
	} else {
		printf("Error: getMarkerPose() check id.\n");
		Eigen::MatrixXd ret;
		return ret;
	}
}

Eigen::Vector3d MarkersController::tMatrix2worldAngles(Eigen::Matrix4d T) {
	Eigen::Vector3d ret;
	double g,b,a;
	double tol = 0.000001;
	double sy = sqrt(pow(T(1,2), 2) + pow(T(2,2), 2));
	
	bool singular = sy < tol;

	b = atan2(T(0,2), sy);

	if(!singular) {
		a = atan2(-T(0,1), T(0,0));
    	g = atan2(-T(1,2), T(2,2));
	} else {
		a = atan2(T(1,0), T(1,1));
        g = 0;
	}

	//std::cout << g << " " << b << " " << a << std::endl;

	ret[0] = g*180/M_PI;
	ret[1] = b*180/M_PI;
	ret[2] = a*180/M_PI;
	return ret;
}

iCubRobot::iCubRobot() {
	
	Network::init();
	
	this->options.put("device", "remote_controlboard");
	this->options.put("local", "/test/client");                 //local port names
	this->options.put("remote", "/icubSim/right_arm");         //where we connect to
	std::cout << "waiting for encoders" <<std::endl;
    this->rightArmControl.open(options);
    std::cout << "waiting for encoders" <<std::endl;
    if (!rightArmControl.isValid()) {
	    printf("Device not available.  Here are the known devices:\n");
	    printf("%s", Drivers::factory().toString().c_str());
	    throw(initException);
	}

	bool ok;
    ok = rightArmControl.view(this->pos) && rightArmControl.view(this->vel) && rightArmControl.view(this->enc);
    printf("waiting for encoders");
    if (!ok) {
        printf("Problems acquiring interfaces\n");
        throw(initException);
    }

    this->jnts = 0;
	pos->getAxes(&(this->jnts));
	std::cout << "Number of Joints: " << this->jnts << std::endl;

	Vector tmp;
	tmp.resize(this->jnts);

	this->encoders.resize(this->jnts);
	this->command_position.resize(this->jnts);
	this->command_velocity.resize(this->jnts);

	// we need to set reference accelerations used to generate the velocity
	// profile, here 50 degrees/sec^2 
	int i;
	for (i = 0; i < this->jnts; i++) {
	    tmp[i] = 50.0;
	}
	printf("waiting for encoders");
	pos->setRefAccelerations(tmp.data());
	

	//fisrst read all encoders
    //
    printf("waiting for encoders");
    while(!enc->getEncoders(this->encoders.data()))
    {
        Time::delay(0.1);
        printf(".");
    }
    printf("\n;");

    printf("\n\nInitialization successful!\n\n");

    this->arm = new GenericRightArm(this->dhParametersiKin);

}


int iCubRobot::getJointCount() {
	return this->jnts;
}

bool iCubRobot::move(std::vector<double> encValues, std::string bodyPartID) {
	/*if(encValues.size() < this->jnts) {
		std::cout << "[WARNING] Input vector has less than " << this -> jnts << " values. Moving first " << encValues.size() << " joints." << std::endl;
	} else if(encValues.size() > this->jnts) {
		std::cout << "[WARNING] Input vector has more than " << this -> jnts << " values. Ignoring extra values." << std::endl;
	}*/
	int i;
	
	for (i = 0; i < this->jnts && i < encValues.size(); i++) {
		this->command_position[i] = encValues[i];
	}
	this->command_position[7] = 60.0; //para não ter os dedos tortos
	if(!this->pos->positionMove(this->command_position.data())) {
		std::cout << "Falso!" << std::endl;
		return false;
	}

    bool done=false;

    i = 0;
 	while(!done) {
        this->pos->checkMotionDone(&done);
        Time::delay(0.1);
        i++;
        if(i >= 100){
        	printf("[robot.move]: Motion not completed.\n");
        	return false;
        }
    }

    this->getRightArmJointValues(encValues);
    
    
    //move markers
    if (useMarkers) {
    	markers.inputHandPose(getTmatrix(encValues, encValues.size()));
    }

    

    return true;
}

bool iCubRobot::look(double x, double y, double z) { 
	std::cout << "Looking to: " << x/10 << " " << y/10 << " " << z/10 << std::endl;
	return gaze.setFixationPoint(x/10, y/10, z/10);
}

bool iCubRobot::getRightArmPose(std::vector<double> &z, std::vector<double> &o, std::vector<double> jointAngles, std::vector<double> &realPose, double &reliability, int jnt,  bool predict) {
	
	if(z.size() != 3)
		z.resize(3);
	if(o.size() != 4)
		o.resize(4);
	if(realPose.size()!=7)
		realPose.resize(7);
	
	Eigen::Matrix4d T, markerPose;
	double rotAngle, sum = 0.0;
	std::vector<double> tvec, rvec, readJoints(7);
	int id;
	bool success;

	if(!this->getRightArmJointValues(readJoints)) 
		return false;

	

	//T.setIdentity();
	T.setZero();
    T(0,1)=-1.0;
    T(1,2)=-1.0;
    T(2,0)=1.0;
    T(3,3)=1.0;
	//std::cout << "joints: " << std::endl;
	//printStdVector(jointAngles);
	
	int i;
	for (i = 0; i < 3; ++i) {
		T = T*evalDHMatrix(dhParameters[4*i], dhParameters[4*i+1],     dhParameters[4*i+2],    dhParameters[4*i+3]);
	}

	for (i = 0; i < jnt; ++i) {
		T = T*evalDHMatrix(dhParameters[4*i + 12], dhParameters[4*i + 13], dhParameters[4*i + 14], jointAngles[i]*M_PI/180 + dhParameters[4*i + 15]);
		//std::cout << dhParameters[4*i + 12] << ", " << dhParameters[4*i + 13] << ", " << dhParameters[4*i + 14] << ", " << jointAngles[i]*M_PI/180 + dhParameters[4*i+ + 15] << std::endl;
	}
	std::cout << "True pose: \n"<< T << std::endl;

	inverseKinematics(z, o, T);
	if(predict) {
		//printStdVector(jointAngles);
		return true;
	}
	realPose[0] = z[0]; realPose[1] = z[1]; realPose[2] = z[2]; 
	realPose[3] = o[0]; realPose[4] = o[1]; realPose[5] = o[2]; realPose[6] = o[3];  
	
	for (i = 0; i < 5; ++i) {
		success = detector.getPose(markerPose, id);
		if(success) break;
		Time::delay(0.1);
	}
	if (success) {  
		if(id == 1){
			printf("Front\n");
			T = getLeftEyeTmatrix() * markerPose * invertTransformMatrix(markers.getMarkerPose("palm"));
		}
		else {
			printf("Back\n");
			T = getLeftEyeTmatrix() * markerPose * invertTransformMatrix(markers.getMarkerPose("back"));
		}
		
		std::cout << "Marker pose robot: \n" << T << std::endl;

		inverseKinematics(z, o, T);
		reliability = predictSampleReliability(markerPose);
		return true;
	} else {
		std::cout << "Hand not found" << std::endl;
		return false;
	}

	
	
	//if(z[1] <= 1) {
	//	std::cout << "Hand not found" << std::endl;
	//	return false;
	//}
	//else
	//	return true;
}



Eigen::Matrix4d cameraPose2matrix(std::vector<double> &tvec, std::vector<double> &rvec) {
	Eigen::Matrix4d cameraPose;
	double theta = sqrt(rvec[0]*rvec[0] + rvec[1]*rvec[1] + rvec[2]*rvec[2]);
	cameraPose.setZero();
	Eigen::Vector3d r;
	Eigen::Matrix3d R;


	for (int i = 0; i < 3; ++i) {
		/* code */
		cameraPose(i,3) = tvec[i]*10; //Passar para dm
		r[i] = rvec[i]/theta; 
	}
	cameraPose(3,3) = 1;
	 
	R(0, 1) = -r[2];
	R(1, 0) = r[2];
	R(0, 2) = r[1];
	R(2, 0) = -r[1];
	R(1, 2) = -r[0];
	R(2, 1) = r[0];

	R = cos(theta)*Eigen::MatrixXd::Identity(3,3) + (1-cos(theta))*r*r.transpose() + sin(theta)*R; 

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			cameraPose(i,j) = R(i,j);
		}
	}

	std::cout << "Marker pose: \n" << cameraPose << std::endl;

	return cameraPose;
}

Eigen::Matrix4d iCubRobot::getLeftEyeTmatrix() {
	Eigen::Matrix4d eyePose;
	yarp::sig::Vector x(3), o(4);
	if(!gaze.getLeftEyePose(x, o)) {
		printf("GetLeftEyePose Error");
	}
	eyePose.setZero();


	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			eyePose(i,j) = o[i]*o[j]*(1-cos(o[3]));
		}
	}

	for (int i = 0; i < 3; ++i) {
		eyePose(i,i) += cos(o[3]);
	}

	eyePose(0,1) += -o[2]*sin(o[3]);
	eyePose(0,2) += o[1]*sin(o[3]);
	eyePose(1,0) += o[2]*sin(o[3]);
	eyePose(1,2) += -o[0]*sin(o[3]);
	eyePose(2,0) += -o[1]*sin(o[3]);
	eyePose(2,1) += o[0]*sin(o[3]);
	eyePose(3,3) = 1;

	for (int i = 0; i < 3; ++i) {
		eyePose(i,3) = x[i]*10; //passar para decímetros
	}

	std::cout << "Eye pose: \n" << eyePose << std::endl;

	return eyePose;
}

Eigen::Matrix4d iCubRobot::getTmatrix(std::vector<double> jointAngles, int jnt) {
	Eigen::Matrix4d T;
	yarp::sig::Vector q;
	yarp::sig::Matrix pose;
	copyVector(q, jointAngles);
	int i;
	//T.setIdentity();
	T.setZero();
    T(0,1)=-1.0;
    T(1,2)=-1.0;
    T(2,0)=1.0;
    T(3,3)=1.0;
	for (i = 0; i < 3; ++i) {
		T = T*evalDHMatrix(dhParameters[4*i], dhParameters[4*i+1],     dhParameters[4*i+2],    dhParameters[4*i+3]);
	}

	for (i = 0; i < jnt; ++i) {
		T = T*evalDHMatrix(dhParameters[4*i + 12], dhParameters[4*i + 13], dhParameters[4*i + 14], jointAngles[i]*M_PI/180 + dhParameters[4*i+ + 15]);
		//std::cout << dhParameters[4*i + 12] << ", " << dhParameters[4*i + 13] << ", " << dhParameters[4*i + 14] << ", " << jointAngles[i]*M_PI/180 + dhParameters[4*i+ + 15] << std::endl;
	}


	iCub::iKin::iKinChain *chain;
	chain = this->arm->asChain();
	for (i = 0; i < 7; ++i)
	{
		q[i] *= M_PI/180;
	}
	chain->setAng(q);
	pose = chain->getH();

	//std::cout << "Normal!\n" << T << std::endl;
	for (i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			T(i,j) = pose(i,j);
		}
	}
	//std::cout << "iKin!\n" << T << std::endl;
	//std::cout << pose[0] << " "<< pose[1] << " "<< pose[2] << " "<< pose[3] << " "<< pose[4] << " "<< pose[5] << " "<< pose[6] << " " << std::endl;

	return T;
}

bool iCubRobot::getRightArmPose(std::vector<double> &z, std::vector<double> &o, std::vector<double> &realPose, double &reliability, int jnt, bool predict) {
	
	std::vector<double> jointValues(7);
	this->getRightArmJointValues(jointValues);
	std::cout << "joints: ";
	printStdVector(jointValues);

	return this->getRightArmPose(z, o, jointValues, realPose, reliability, jnt, predict);
}

bool iCubRobot::getRightArmJointValues(std::vector<double> &thetas){
	int i;
	bool ret=enc->getEncoders(encoders.data());
                 
    if (!ret) {
        fprintf(stderr, "Error receiving encoders, check connectivity with the robot\n");
    	return false;
    }
    else { 
        //printf("%.1lf %.1lf %.1lf %.1lf\n", encoders[0], encoders[1], encoders[2], encoders[3]);
    	for (i = 0; i < thetas.size(); ++i) {
			thetas[i] = encoders[i];
		}
    	return true;
    }	
}

iCubRobot::~iCubRobot(){
	rightArmControl.close();
}

std::vector<double> iCubRobot::getDHparameters(int njoints) {
	std::vector<double> parameters(4*njoints);
	int i;
	for (i = 0; i < 4*njoints; ++i) {
		parameters[i] = this->dhParameters[i+12];
	}

	return parameters;
}



Eigen::Matrix4d robot2worldFrame(Eigen::Matrix4d T) {
	Eigen::Matrix4d Trw;
	Trw << 0, -1,  0,      0,
		   0,  0,  1, 5.976,
		  -1,  0,  0, -0.26,
		   0,  0,  0,      1;

	return Trw*T; //robo está em dm
}

Eigen::Matrix4d world2robotFrame(Eigen::Matrix4d T) {
	Eigen::Matrix4d Trw, Twr;
	Trw << 0, -1,  0,      0,
		   0,  0,  1, 5.976,
		  -1,  0,  0, -0.26,
		   0,  0,  0,      1;

	Twr = invertTransformMatrix(Trw);

	return Twr*T;
}

Matrix eigenMatrix2yarp(Eigen::MatrixXd T) {
	Matrix ret(T.rows(), T.cols());
	for (int i = 0; i < T.rows(); ++i) {
		for (int j = 0; j < T.cols(); ++j) {
			ret[i][j] = T(i,j);
		}
	}
}
