#ifndef ARUCO_DETECTOR_HPP
#define ARUCO_DETECTOR_HPP


#include <stdio.h>
/* Get all OS and signal processing YARP classes */
#include <yarp/os/all.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/sig/ImageFile.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core.hpp>
#include <yarp/cv/Cv.h>

#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco/dictionary.hpp>


#include <Eigen/Dense>
using namespace yarp::os;
using namespace yarp::sig;

class ArucoDetector {
public:
	ArucoDetector(bool debugFlag = false);
	bool getPose(Eigen::Matrix4d &T, int &id);
private:
	bool debugger;
	Network yarp; // set up yarp
  	BufferedPort<ImageOf<PixelRgb>> imagePort;  // make a port for reading images
  	cv::Ptr<cv::aruco::Dictionary> dictionary;
  	cv::Mat cameraMatrix, distCoeffs;
  	cv::Ptr<cv::aruco::DetectorParameters> dp;
};

#endif
