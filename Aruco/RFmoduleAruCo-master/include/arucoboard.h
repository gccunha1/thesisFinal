// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Pedro Vicente <pvicente@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#ifndef ARUCOBOARD_H
#define ARUCOBOARD_H

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>
#include <yarp/sig/Image.h>
#include <yarp/math/Math.h>

#include <fstream>
#include <string>

#include <highgui.h>
#include <cv.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>

#include "ARUCOBOARD_IDL.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;
using namespace cv;

class ArucoBoardModule: public RFModule, public ARUCOBOARD_IDL {
    string moduleName;
    /*Name of ports to be open*/
	string posePortName, imageInPortName, imageOutPortName,handlerPortName;
	/*Ports to be open*/
    BufferedPort<Bottle> posePort;
	BufferedPort<ImageOf<PixelBgr> > imageInPort;
	BufferedPort<ImageOf<PixelBgr> > imageOutPort;
	RpcServer handlerPort;
private:
    bool _closing;
    bool _createImage;
    IplImage *_cvImage;
    int _markersX, _markersY, _markerLength, _markerSeparation, _dictionaryId, _margins, _borderBits;
    float _axisLength;
    Mat _camMatrix, _distCoeffs;
    Ptr<aruco::Dictionary> _dictionary;
    Ptr<aruco::GridBoard> _board;
    Ptr<aruco::Board> _Sboard;
    Ptr<aruco::DetectorParameters> _detectorParams;
    Vec3d RotAng(const Mat &R);
public:
    
    double getPeriod();
    bool configure(yarp::os::ResourceFinder &rf);
    bool updateModule();
    bool interruptModule();
    bool close();

    // IDL functions
    bool attach(yarp::os::RpcServer &source);
    bool quit();
};
#endif // TRANSLATOR_H
