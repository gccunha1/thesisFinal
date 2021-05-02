 /*
  * Copyright (C) 2013 RobotCub Consortium, European Commission FP6 Project IST-004370
  * Author:  Afonso Gonçalves
  * email:   agoncalves@isr.ist.utl.pt
  * website: www.robotcub.org
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  * http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
  */

#ifndef SIMOBJLOADERMODULE_H
#define SIMOBJLOADERMODULE_H

#include <string>
#include <vector>
#include <yarp/os/all.h>

#include <Eigen/Dense>


using namespace std;
using namespace yarp::os;

enum iCubArm {LEFT, RIGHT};
enum objType {BOX, SBOX, SPH, SSPH, CYL, SCYL, MODEL, SMODEL};

class SimObject {
protected:
    double positionX0;
    double positionY0;
    double positionZ0;

    double positionX;
    double positionY;
    double positionZ;

    double rotationX;
    double rotationY;
    double rotationZ;

    double colorR;
    double colorG;
    double colorB;

public:
    int objSubIndex;

    void setObjectPosition();
    void setObjectPosition(double posx, double posy,  double posz);
    void setObjectRotation(double rotx, double roty,  double rotz);
    void setObjectColor   (double red,  double green, double blue);

    virtual Bottle   makeObjectBottle(vector<int>& ind, bool collision = true)  = 0;
    virtual Bottle   grabObjectBottle(iCubArm arm)                     = 0;
    virtual Bottle   dropObjectBottle(iCubArm arm)                     = 0;
    virtual Bottle   moveObjectBottle()                                = 0;
    virtual Bottle rotateObjectBottle()                                = 0;
    virtual Bottle deleteObject()                                      = 0; //no implementation yet
};

class SimBox: public SimObject {
private:
    double sizeX;
    double sizeY;
    double sizeZ;

public:
    SimBox(double posx,  double posy,  double posz,
           double rotx,  double roty,  double rotz,
           double red,   double green, double blue,
           double sizx,  double sizy,  double sizz);
    virtual Bottle   makeObjectBottle(vector<int>& ind, bool collision = true);
    virtual Bottle deleteObject();
    virtual Bottle   moveObjectBottle();
    virtual Bottle rotateObjectBottle();
    virtual Bottle   grabObjectBottle(iCubArm arm);
    virtual Bottle   dropObjectBottle(iCubArm arm);

};

class SimSBox: public SimObject {
private:
    double sizeX;
    double sizeY;
    double sizeZ;

public:
    SimSBox(double posx,  double posy,  double posz,
            double rotx,  double roty,  double rotz,
            double red,   double green, double blue,
            double sizx,  double sizy,  double sizz);
    virtual Bottle   makeObjectBottle(vector<int>& ind, bool collision = true);
    virtual Bottle deleteObject();
    virtual Bottle   moveObjectBottle();
    virtual Bottle rotateObjectBottle();
    virtual Bottle   grabObjectBottle(iCubArm arm);
    virtual Bottle   dropObjectBottle(iCubArm arm);
};

class SimSph: public SimObject {
private:
    double radius;

public:
    SimSph(double posx,  double posy,  double posz,
           double rotx,  double roty,  double rotz,
           double red,   double green, double blue,
           double rad);
    virtual Bottle   makeObjectBottle(vector<int>& ind, bool collision = true);
    virtual Bottle deleteObject();
    virtual Bottle   moveObjectBottle();
    virtual Bottle rotateObjectBottle();
    virtual Bottle   grabObjectBottle(iCubArm arm);
    virtual Bottle   dropObjectBottle(iCubArm arm);
};

class SimSSph: public SimObject {
private:
    double radius;

public:
    SimSSph(double posx,  double posy,  double posz,
            double rotx,  double roty,  double rotz,
            double red,   double green, double blue,
            double rad);
    virtual Bottle   makeObjectBottle(vector<int>& ind, bool collision = true);
    virtual Bottle deleteObject();
    virtual Bottle   moveObjectBottle();
    virtual Bottle rotateObjectBottle();
    virtual Bottle   grabObjectBottle(iCubArm arm);
    virtual Bottle   dropObjectBottle(iCubArm arm);
};

class SimCyl: public SimObject {
private:
    double radius;
    double height;

public:
    SimCyl(double posx,  double posy,  double posz,
           double rotx,  double roty,  double rotz,
           double red,   double green, double blue,
           double rad,   double hei);
    virtual Bottle   makeObjectBottle(vector<int>& ind, bool collision = true);
    virtual Bottle deleteObject();
    virtual Bottle   moveObjectBottle();
    virtual Bottle rotateObjectBottle();
    virtual Bottle   grabObjectBottle(iCubArm arm);
    virtual Bottle   dropObjectBottle(iCubArm arm);
};

class SimSCyl: public SimObject {
private:
    double radius;
    double height;

public:
    SimSCyl(double posx,  double posy,  double posz,
            double rotx,  double roty,  double rotz,
            double red,   double green, double blue,
            double rad,   double hei);
    virtual Bottle   makeObjectBottle(vector<int>& ind, bool collision = true);
    virtual Bottle deleteObject();
    virtual Bottle   moveObjectBottle();
    virtual Bottle rotateObjectBottle();
    virtual Bottle   grabObjectBottle(iCubArm arm);
    virtual Bottle   dropObjectBottle(iCubArm arm);
};

class SimModel: public SimObject {
private:
    std::string mesh;
    std::string texture;

public:
    SimModel(double posx,  double posy,  double posz,
             double rotx,  double roty,  double rotz,
             std::string mes, std::string tex);
    virtual Bottle   makeObjectBottle(vector<int>& ind, bool collision = true);
    virtual Bottle deleteObject();
    virtual Bottle   moveObjectBottle();
    virtual Bottle rotateObjectBottle();
    virtual Bottle   grabObjectBottle(iCubArm arm);
    virtual Bottle   dropObjectBottle(iCubArm arm);
};

class SimSModel: public SimObject {
private:
    std::string mesh;
    std::string texture;

public:
    SimSModel( double posx,  double posy,  double posz,
              double rotx,  double roty,  double rotz,
              std::string mes, std::string tex);
    virtual Bottle   makeObjectBottle(vector<int>& ind, bool collision = true);
    virtual Bottle deleteObject();
    virtual Bottle   moveObjectBottle();
    virtual Bottle   rotateObjectBottle();
    virtual Bottle   grabObjectBottle(iCubArm arm);
    virtual Bottle   dropObjectBottle(iCubArm arm);
};

class SimSMarker: public SimSModel {
private:
    Eigen::Matrix4d markerPose;
public:
    SimSMarker(double posx,  double posy,  double posz,
              double rotx,  double roty,  double rotz,
              std::string mes, std::string tex, std::string type, double offset);
    Eigen::Matrix4d getPose();
};

// class SimWorld {
// protected:

// public:
//     SimWorld();
//     SimWorld(const Bottle& threadTable, vector<Bottle>& threadObject);
//     Bottle deleteAll();
//     void resetSimObjectIndex();

//     SimObject *simTable;
//     vector<SimObject*> simTarget;
//     vector<SimObject*> simTool;
//     vector<int> objSubIndex;
// };

/*

class CtrlThread:public RateThread {
protected:

    Port      *simObjLoaderCmdInputPort;
    RpcClient *simObjLoaderSimOutputPort;

    Bottle threadTable;
    vector<Bottle> threadObject;
    SimWorld simWorld;

public:
    CtrlThread(RpcClient *simObjLoaderSimOutput,
               Port *simObjLoaderCmdInput,
               const double period,
               const Bottle table,
               vector<Bottle> object);
    bool threadInit();
    void threadRelease();
    void run();
    void clearWorld();
    void writeSim(Bottle cmd);
    void replyCmd(Bottle cmd);
};

class SimObjLoaderModule: public RFModule {
    string moduleName;

    string      simObjLoaderSimOutputPortName;
    RpcClient   simObjLoaderSimOutputPort;

    string      simObjLoaderCmdInputPortName;
    Port        simObjLoaderCmdInputPort;

    int threadPeriod;
    int numberObjs;

    CtrlThread *ctrlThread;

public:
    double getPeriod();
    bool configure(yarp::os::ResourceFinder &rf);
    bool updateModule();
    bool interruptModule();
    bool close();
};
*/
#endif /* SIMOBJLOADERMODULE_H */
