#include "../iCubWorld/simobjloader.h"
#include "../iCubWorld/worldclient.hpp"
#include "../iCub/Robot.hpp"
#include "../iCubVision/ArucoDetector.hpp"
#include "../utils/utils.hpp"

#include <fstream>

using namespace yarp::sig;

int main(int argc, char const *argv[]) {
	SimSModel marker(0,0,0,0,0,0,"marker4_5cm.x", "marker1_4_4_50.bmp");
	Vector x(3), o(4);
	ArucoDetector detector(false);
	GazeController gaze;
	iCubWorldClient client;
	std::vector<int> ind(10,0);
	std::vector<double> rvec, tvec;
	double posError, oriError;
	int id, nSamples = 7;
	double px, py, pz, rx, ry, rz;

	std::vector<double> start(6), end(6);
	start[0] = -0.3; end[0] = 0.3; // x
	start[1] = 0.7; end[1] = 1.2; // y
	start[2] = 0.1; end[2] = 0.5; // z
	start[3] = -180; end[3] = 180; // rx
	start[4] = -180; end[4] = 180; // ry
	start[5] = -180; end[5] = 180; // rz

	std::vector<std::vector<double>> testValues;
	testValues.resize(6);
	for (int i = 0; i < testValues.size(); ++i) {
		testValues[i].resize(nSamples);
		for (int j = 0; j < testValues[i].size(); ++j) {
			testValues[i][j] = j*(end[i] - start[i])/(testValues[i].size()-1) + start[i];
			std::cout << testValues[i][j] << " ";
		}
		std::cout << std::endl;
	}


	Eigen::Matrix4d realPoseWorld, realPoseRobot, leftEyeRobot, measuredPoseRobot, cameraPose;
	Matrix rot(3,3);

	ofstream posErrorFile, oriErrorFile;
	posErrorFile.open ("posErrorSamples.txt");
	oriErrorFile.open ("oriErrorSamples.txt");

	client.sendCommand(marker.makeObjectBottle(ind, false));

	for(int i = 0; i < pow(nSamples, testValues.size()); i++) {
		//Request marker pose
		printf("Input pose: [px py pz rx ry rz]: \n");
		//std::cin >> px;
		//std::cin >> py;
		//std::cin >> pz;
		//std::cin >> rx;
		//std::cin >> ry;
		//std::cin >> rz;
		px = testValues[0][i%nSamples]; py = testValues[1][(i/nSamples)%nSamples]; pz = testValues[2][(i/((int)pow(nSamples,2)))%nSamples];
		rx = testValues[3][(i/((int)pow(nSamples,3)))%nSamples]; ry = testValues[4][(i/((int)pow(nSamples,4)))%nSamples]; rz = testValues[5][(i/((int)pow(nSamples,5)))%nSamples];

		printf("%f, %f, %f, %f, %f, %f\n",  px, py, pz, rx, ry, rz);

		//euler angles to T matrix
		realPoseWorld = combineRotTrans(eulerXYZ(rx*M_PI/180, ry*M_PI/180, rz*M_PI/180), 10*px, 10*py, 10*pz);
		//realPos2robotFrame
		realPoseRobot = world2robotFrame(realPoseWorld);

		//std::cout << "Real pose: \n" << realPoseRobot<< std::endl;
		
		//set position and rotation
		marker.setObjectPosition(px,py,pz);
		marker.setObjectRotation(rx,ry,rz);
		//make bottle
		//sendcommand
		client.sendCommand(marker.moveObjectBottle());
		client.sendCommand(marker.rotateObjectBottle());
		
		//olhar
		//gaze.setFixationPoint(realPoseRobot(0,3)/10, realPoseRobot(1,3)/10, realPoseRobot(2,3)/10);
		
		//getEyeTransform
		gaze.getLeftEyePose(x, o);
		leftEyeRobot = combineRotTrans(axisAngle2matrix(o[0], o[1], o[2], o[3]), 10*x[0], 10*x[1], 10*x[2]);
		
		//Detect marker
		if(detector.getPose(cameraPose, id)) {
			//getmarkerpose
			//markerpose 2 robot frame
			measuredPoseRobot = leftEyeRobot * cameraPose;
			//std::cout << "Measured pose: \n" << measuredPoseRobot<< std::endl;

			computeKinematicsError(posError, oriError, realPoseRobot, measuredPoseRobot);

			inverseKinematics(tvec, rvec, cameraPose);
			
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					rot[i][j] = cameraPose(i,j);
				}
			}

			Vector axisAngle = yarp::math::dcm2axis(rot); 	
			posErrorFile << tvec[0] << " "<< tvec[1] << " "<< tvec[2] << " "<< axisAngle[0] << " "<< axisAngle[1] << " "<< axisAngle[3] << " " << posError << std::endl;
			oriErrorFile << tvec[0] << " "<< tvec[1] << " "<< tvec[2] << " "<< axisAngle[0] << " "<< axisAngle[1] << " "<< axisAngle[3] << " " << oriError << std::endl;
		}
	}

	posErrorFile.close();
	oriErrorFile.close();

	return 0;
}



