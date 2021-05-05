#include "ArucoDetector.hpp"
#include "../utils/utils.hpp"



ArucoDetector::ArucoDetector(bool debugFlag) {
	debugger = debugFlag;
	if(debugFlag)
		imagePort.open("/aruco/image/in/debug");
	else
		imagePort.open("/aruco/image/in/exec");
	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

	cameraMatrix = cv::Mat::zeros(3,3,CV_64F);
	distCoeffs = cv::Mat::zeros(1,4,CV_64F);
    cameraMatrix.at<double>(0,0) = 514.681;
    cameraMatrix.at<double>(1,1) = 514.681;
    cameraMatrix.at<double>(2,2) = 1;
    cameraMatrix.at<double>(0,2) = 320.0;
    cameraMatrix.at<double>(1,2) = 240.0;

	dp = cv::aruco::DetectorParameters::create();
}

bool ArucoDetector::getPose(Eigen::Matrix4d &T, int &id) {
	ImageOf<PixelRgb> *image = imagePort.read(false);  // read an image
	cv::Mat imageCopy;
	cv::Mat rMat;
    Eigen::Matrix3d rotMat;

    if (image!=NULL) { // check we actually got something
		printf("We got an image of size %dx%d\n", (int) image->width(), (int) image->height());
    } else {
    	printf("Image not found. Check connection with imagePort.\n");
    	return false;
    }

    cv::Mat cvImage = yarp::cv::toCvMat(*image);

    if(debugger) {
	    cvImage.copyTo(imageCopy);
    }
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(cvImage, dictionary, corners, ids);

    if (ids.size() > 0) {
    	if(debugger)
    		cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

    	std::vector<cv::Vec3d> rvecs, tvecs;
    	cv::aruco::estimatePoseSingleMarkers(corners, 0.04, cameraMatrix, distCoeffs, rvecs, tvecs);
    	
	    if(debugger)	
	    	for(int i = 0; i < ids.size(); i++)
	            cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
    	
    	
    	id = ids[0];


    	cv::Rodrigues(rvecs[0], rMat);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rotMat(i,j) = rMat.at<double>(i,j);
            }
        }

        T = combineRotTrans(rotMat, 10*tvecs[0][0], 10*tvecs[0][1], 10*tvecs[0][2]);
    	printf("Detected pose: \n");
        std::cout << T << std::endl;

    	if(debugger) {
	    	cv::imshow("out", imageCopy);
	    	cv::waitKey(50);
	    }

    	return true;

    } else {
    	printf("Marker not found\n");
    	if(debugger) {
	    	cv::imshow("out", imageCopy);
	    	cv::waitKey(50);
	    }
    	return false;
    }
}