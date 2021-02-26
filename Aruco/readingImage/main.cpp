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

using namespace yarp::os;
using namespace yarp::sig;
int main() {
  Network yarp; // set up yarp
  BufferedPort<ImageOf<PixelRgb> > imagePort;  // make a port for reading images
  imagePort.open("/aruco/image/in");  // give the port a name
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(2, 2, 0.015, 0.005, dictionary);
  cv::Mat imageCopy;
  
  cv::Mat cameraMatrix(3, 3, CV_64F), distCoeffs = cv::Mat::zeros(1,4,CV_64F);

  cameraMatrix.at<double>(0,0) = 257.34;
  cameraMatrix.at<double>(1,1) = 257.34;
  cameraMatrix.at<double>(2,2) = 1;
  cameraMatrix.at<double>(0,2) = 160;
  cameraMatrix.at<double>(1,2) = 120;

  cv::Ptr<cv::aruco::DetectorParameters> dp = cv::aruco::DetectorParameters::create();
  //dp->minDistanceToBorder = 3;
  //dp->adaptiveThreshWinSizeMin = 0;
  //dp->adaptiveThreshWinSizeMax = 300;
  //dp->maxMarkerPerimeterRate = 20;
  //dp->adaptiveThreshWinSizeStep = 100;
  //dp->adaptiveThreshConstant = 7;
  int i = 0;
  while (1) { 
    ImageOf<PixelRgb> *image = imagePort.read();  // read an image
    if (image!=NULL) { // check we actually got something
      printf("We got an image of size %dx%d\n", image->width(), image->height());
      //file::write(*image, "test.jpg", file::FORMAT_JPG);
    }

    cv::Mat cvImage1=yarp::cv::toCvMat(*image);
    //cv::namedWindow("test",1);
    //cv::imshow("test",cvImage1);
    
    cvImage1.copyTo(imageCopy);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(cvImage1, dictionary, corners, ids);
    // if at least one marker detected
    if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        cv::Vec3d rvec, tvec;
        int valid = estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs, rvec, tvec);
        // if at least one board marker detected
        if(valid > 0)
            cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
    } else {
      std::cout << "No markers :(" << std::endl;
    }
    
    cv::imshow("out", imageCopy);
    cv::waitKey(50);
  }
  return 0;
}
