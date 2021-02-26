// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Pedro Vicente <pvicente@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include "arucoboard.h"

using namespace yarp::os;
using namespace std;
using namespace cv;

// ArucoBoard Module
double ArucoBoardModule::getPeriod() {
    return 0.0;
}

/*IDL Functions*/
bool ArucoBoardModule::quit() {
    cout << "Received Quit command in RPC" << endl;
    _closing = true;
    return true;
}

Vec3d ArucoBoardModule::RotAng(const Mat &R) {
    Vec3d r;
    // Same function as in iKin library but with opencv Mat
    // Euler Angles as XYZ (see dcm2angle.m)
    r[0] = atan2(-R.at<double>(2,1),R.at<double>(2,2));
    r[1] = asin(R.at<double>(2,0));
    r[2] = atan2(-R.at<double>(1,0),R.at<double>(0,0));

    return r;
}
bool ArucoBoardModule::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}
// End IDL functions

bool ArucoBoardModule::updateModule() {

    ImageOf<PixelBgr> *inputImage;
    Mat ImageMat;
    Bottle rpose, tpose;
    vector< int > ids;
    vector< vector< Point2f > > corners, rejected;
    Vec3d rvec,rvec_euler, tvec;
    yarp::sig::Vector rvec_axis;
    yarp::sig::Matrix rmat_yarp(4,4);
    Mat rMat;
    if(imageInPort.getInputCount()<=0)
        return true;
    inputImage = imageInPort.read(false);
    if(inputImage==NULL)
        return true;
    if(_createImage) {
        _cvImage = cvCreateImage(cvSize(inputImage->width(), inputImage->height()), IPL_DEPTH_8U, 3 );
        _createImage=false;
    }

    cvCopy((IplImage*)inputImage->getIplImage(),_cvImage);
    ImageMat = cvarrToMat(_cvImage);
    aruco::detectMarkers(ImageMat, _dictionary, corners, ids, _detectorParams, rejected);
    aruco::refineDetectedMarkers(ImageMat,_Sboard,corners, ids,rejected);
    int markersOfBoardDetected = 0;
    /**/
    if(ids.size() > 0) {
        markersOfBoardDetected = aruco::estimatePoseBoard(corners, ids, _Sboard, _camMatrix, _distCoeffs, rvec, tvec);
    }
    if(ids.size() > 0) {
        aruco::drawDetectedMarkers(ImageMat, corners, ids);
    }
    if(markersOfBoardDetected > 0) {
        aruco::drawAxis(ImageMat, _camMatrix, _distCoeffs, rvec, tvec, _axisLength);
        // Convertion to Rotation Matrix
        Rodrigues(rvec, rMat);
        for(int ii=0;ii<4;ii++) {
            for(int jj=0;jj<4;jj++){
                rmat_yarp(ii,jj) = rMat.at<double>(ii,jj);
            }
        }
        rvec_axis = dcm2axis(rmat_yarp);
        //rvec_euler = ArucoBoardModule::RotAng(rMat);
/*
        rpose.addDouble(rvec_euler[0]);
        rpose.addDouble(rvec_euler[1]);
        rpose.addDouble(rvec_euler[2]);

/*OLD/
        tpose.addDouble(tvec[0]*3.5/100.0);
        tpose.addDouble(tvec[1]*3.5/100.0);
        tpose.addDouble(tvec[2]*3.5/100.0);
/*NEW*/
/*
        tpose.addDouble(tvec[0]*7.22/275.0);
        tpose.addDouble(tvec[1]*7.22/275.0);
        tpose.addDouble(tvec[2]*7.22/275.0);

        //send.addList() = rpose;
        //send.addList() = tpose; 

        //** FINAL BOTTLE **/
        Bottle & send = posePort.prepare();
        send.clear();
/*OLD/
        send.addDouble(tvec[0]*0.035/100.0); // convert to meters
        send.addDouble(tvec[1]*0.035/100.0);
        send.addDouble(tvec[2]*0.035/100.0);
/*new*/
        send.addDouble(tvec[0]*0.0722/275.0); // convert to meters
        send.addDouble(tvec[1]*0.0722/275.0);
        send.addDouble(tvec[2]*0.0722/275.0);
        send.addDouble(rvec_axis[0]);
        send.addDouble(rvec_axis[1]);
        send.addDouble(rvec_axis[2]);
        send.addDouble(rvec_axis[3]);
        posePort.write();
    }
    IplImage tmpImage = (IplImage) ImageMat;
    cvReleaseImage(&_cvImage);
    _cvImage = cvCloneImage(&tmpImage);
    ImageOf <PixelBgr> &outImage = imageOutPort.prepare();
    outImage.wrapIplImage(_cvImage);
    imageOutPort.write();
    return !_closing;
}

bool ArucoBoardModule::interruptModule() {
    cout << "Interrupting your module, for port cleanup" << endl;

    posePort.interrupt();
    imageInPort.interrupt();
    imageOutPort.interrupt();

    return true;
}

bool   ArucoBoardModule::close() {

    /* optional, close port explicitly */
    cout << "Calling close function\n";

    posePort.close();
    imageInPort.close();
    imageOutPort.close();

    return true;
}

bool   ArucoBoardModule::configure(yarp::os::ResourceFinder &rf) {
    _closing = false;
    _createImage=true;
    /* module name */
    moduleName = rf.check("name", Value("arucoboard"),
                          "Module name (string)").asString();

    setName(moduleName.c_str());

    /* port names */
    posePortName  = "/" + moduleName + "/pose:o";
    imageInPortName = "/" + moduleName + "/image:i";
    imageOutPortName = "/" + moduleName + "/image:o";

    /* open ports */
    if (!posePort.open(
            posePortName.c_str()))
    {
        cout << getName() << ": unable to open port"
        << posePortName << endl;
        return false;
    }

    if (!imageInPort.open(
            imageInPortName.c_str()))
    {
        cout << getName() << ": unable to open port"
        << imageInPortName << endl;
        return false;
    }

    if (!imageOutPort.open(
            imageOutPortName.c_str()))
    {
        cout << getName() << ": unable to open port"
        << imageOutPortName << endl;
        return false;
    }


    handlerPortName = "/" + moduleName + "/rpc:i";
    handlerPort.open(handlerPortName.c_str());
    attach(handlerPort);
    /*iCub left parameters*/
    /*fx,0,cx;0,fy,cy;0,0,1*/
    _camMatrix = (Mat_<double>(3,3) << 215.444,0,168.92, 0,215.737,129.72, 0,0,1);
    /*k1,k2,p1,p2*/
    _distCoeffs = (Mat_<double>(1,4) << -0.386704,0.12594,0.000743205, -0.0030906);
    /*Aruco specifications*/
/*OLD*//*
    _markersX = 3;
    _markersY = 3;
    _markerLength = 100;
    _markerSeparation = 20;
*/
/*NEW*/
    _markersX = 1;
    _markersY = 3;
    _markerLength = 275;
    _markerSeparation = 15;

    _dictionaryId = 8;
    _margins = _markerSeparation;
    _borderBits = 1;
    _axisLength = 0.5f * ((float)min(_markersX, _markersY) * (_markerLength + _markerSeparation) + _markerSeparation);
    _dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(_dictionaryId));
    _board = aruco::GridBoard::create(_markersX, _markersY, float(_markerLength), float(_markerSeparation), _dictionary);
    _Sboard = _board.staticCast<aruco::Board>();
    _detectorParams = aruco::DetectorParameters::create();
    return true;
}
