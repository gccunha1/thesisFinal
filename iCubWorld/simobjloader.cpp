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

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>

#include "simobjloader.h"

using namespace std;
using namespace yarp::os;
//using namespace yarp::sig;

// double SimObjLoaderModule::getPeriod() {

//     return 1;
// }

// bool   SimObjLoaderModule::updateModule() {

//     return true;
// }

// bool   SimObjLoaderModule::configure(yarp::os::ResourceFinder &rf) {

//     Bottle table;
//     Bottle temp;
//     char objectName[] = {'o', 'b', 'j', '0', '\0'};

//     /* module name */
//     moduleName = rf.check("name", Value("simobjloader"),
//                           "Module name (string)").asString();

//     setName(moduleName.c_str());

//     /* port names */
//     simObjLoaderSimOutputPortName  = "/";
//     simObjLoaderSimOutputPortName += getName( rf.check("simObjLoaderSimOutputPort",
//                                      Value("/sim:o"),
//                                      "Loader output port(string)")
//                                      .asString() );

//     simObjLoaderCmdInputPortName  = "/";
//     simObjLoaderCmdInputPortName += getName( rf.check("simObjLoaderCmdInputPort",
//                                      Value("/cmd:i"),
//                                      "Loader input port(string)")
//                                      .asString() );

//     /* open ports */
//     if (!simObjLoaderSimOutputPort.open(
//             simObjLoaderSimOutputPortName.c_str())) {

//         cout << getName() << ": unable to open port"
//         << simObjLoaderSimOutputPortName << endl;
//         return false;
//     }

//     if (!simObjLoaderCmdInputPort.open(
//             simObjLoaderCmdInputPortName.c_str())) {

//         cout << getName() << ": unable to open port"
//         << simObjLoaderCmdInputPortName << endl;
//         return false;
//     }

//     /* Rate thread period */
//     threadPeriod = rf.check("threadPeriod", Value(0.5),
//         "Control rate thread period key value(double) in seconds ").asDouble();

//     /* Read Table Configuration */
//     table = rf.findGroup("table");


//     /* Read the Objects configurations */
//     numberObjs  = rf.findGroup("objects").find("number").asInt();
//     vector<Bottle> object;
//     for ( int n=0 ; n<numberObjs ; n++ ) {
//         objectName[3]++;
//         //cout << objectName;
//         temp = rf.findGroup("objects").findGroup(objectName);
//         object.push_back(temp);
//         temp.clear();
//     }

//     /* Create the control rate thread */
//     ctrlThread = new CtrlThread(&simObjLoaderSimOutputPort,
//                                 &simObjLoaderCmdInputPort,
//                                 threadPeriod,
//                                 table, object);
//     /* Starts the thread */
//     if (!ctrlThread->start()) {
//         delete ctrlThread;
//         return false;
//     }

//     return true;
// }

// bool   SimObjLoaderModule::interruptModule() {

//     cout << "Interrupting your module, for port cleanup" << endl;

//     simObjLoaderSimOutputPort.interrupt();
//     simObjLoaderCmdInputPort.interrupt();

//     return true;
// }

// bool   SimObjLoaderModule::close() {

//     /* optional, close port explicitly */
//     cout << "Calling close function\n";

//     ctrlThread->stop();
//     delete ctrlThread;
//     simObjLoaderSimOutputPort.close();
//     simObjLoaderCmdInputPort.close();

//     return true;
// }

// CtrlThread::CtrlThread(RpcClient *simObjLoaderSimOutput,
//                        Port *simObjLoaderCmdInput,
//                        const double period, const Bottle table,
//                        vector<Bottle> object)
//                        :RateThread(int(period*1000.0)) {

//     simObjLoaderSimOutputPort = simObjLoaderSimOutput;
//     simObjLoaderCmdInputPort  = simObjLoaderCmdInput;
//     threadTable  = table;
//     threadObject = object;
// }

// bool CtrlThread::threadInit() {

//     cout << endl << "thread initialization" << endl;

//     // Make a connection to the iCub Simulator world port:
//     (*simObjLoaderSimOutputPort).addOutput("/icubSim/world");

//     srand (time(NULL));

//     Bottle simCmd;
//     simCmd.addString("world");
//     simCmd.addString("set");
//     simCmd.addString("mdir");

//     //const std::string icubRootEnvPath = yarp::os::getenv("ICUB_ROOT");
//     //const std::string localModelsPath = "/contrib/src/poeticon/poeticonpp/simobjloader/models";

//     const std::string icubContribEnvPath = yarp::os::getenv("ICUBcontrib_DIR");
//     const std::string localModelsPath    = "/share/ICUBcontrib/contexts/simobjloader/models";
//     const std::string modelsPath         = icubContribEnvPath + localModelsPath;
//     simCmd.addString(modelsPath);
//     writeSim(simCmd);

//     simWorld = SimWorld(threadTable, threadObject);

//     Time::delay(0.1);

//     simCmd = simWorld.deleteAll();
//     writeSim(simCmd);

//     Time::delay(0.1);

//     simCmd.clear();
//     simWorld.simTable->setObjectPosition();
//     simCmd = simWorld.simTable->makeObjectBottle(simWorld.objSubIndex, false);
//     writeSim(simCmd);

//     Time::delay(0.1);

//     for ( int n=0 ; n<threadObject.size() ; n++ ) {
//         simCmd.clear();
//         simWorld.simTarget[n]->setObjectPosition();
//         simCmd = simWorld.simTarget[n]->makeObjectBottle(simWorld.objSubIndex);
//         writeSim(simCmd);
//         Time::delay(0.1);
//     }

//     for ( int n=0 ; n<threadObject.size() ; n++ ) {
//         simCmd.clear();
//         simWorld.simTool[n]->setObjectPosition();
//         simCmd = simWorld.simTool[n]->makeObjectBottle(simWorld.objSubIndex);
//         writeSim(simCmd);
//         Time::delay(0.1);
//     }

//     return true;
// }

// void CtrlThread::threadRelease() {

//     cout << endl << "thread release" << endl;
// }

// void CtrlThread::run() {

//     Bottle controlCmd;
//     Bottle simCmd;
//     int objIndex  = 0;
//     int toolIndex = 0;
//     NetInt32 code;


//     if( (*simObjLoaderCmdInputPort).read(controlCmd, true) ) {
//         code = Vocab::encode((controlCmd.get(0)).asString());

//         cout << (controlCmd.get(0)).asString() << endl;

//         switch(code) {
//             case VOCAB3('d','e','l'): {
//                 cout << "Clear world" <<endl;

//                 //Clear the World:
//                 simCmd = simWorld.deleteAll();
//                 writeSim(simCmd);

//                 //reply to the control manager
//                 controlCmd.clear();
//                 controlCmd.addVocab(code);
//                 replyCmd(controlCmd);

//                 break;
//             }
//             case VOCAB4('t','o','o','l'): {
//                 cout << "Create table and tool (on the table)." <<endl;

//                 //Clear the World:
//                 simCmd = simWorld.deleteAll();
//                 writeSim(simCmd);

//                 //Create the table:
//                 simCmd = simWorld.simTable->makeObjectBottle(simWorld.objSubIndex, false);
//                 writeSim(simCmd);

//                 //Create the tool on the table:
//                 toolIndex = controlCmd.get(1).asInt();
//                 if ( toolIndex==0 ) {
//                     toolIndex = rand() % simWorld.simTarget.size() + 1;
//                 }
//                 simWorld.simTarget[toolIndex-1]->setObjectPosition(
//                                         threadTable.get(4).asDouble()+0.05,
//                                         threadTable.get(5).asDouble()+((threadTable.get(2).asDouble())/2)+0.1,
//                                         0.35);
//                 simCmd = simWorld.simTarget[toolIndex-1]->makeObjectBottle(simWorld.objSubIndex);
//                 writeSim(simCmd);
//                 simWorld.simTarget[toolIndex-1]->setObjectRotation(0, -50, 0);
//                 simCmd = simWorld.simTarget[toolIndex-1]->rotateObjectBottle();
//                 writeSim(simCmd);

//                 //reply to the control manager the tool ID
//                 controlCmd.clear();
//                 controlCmd.addVocab(code);
//                 controlCmd.addInt(toolIndex);
//                 replyCmd(controlCmd);

//                 break;
//             }
//             case VOCAB3('o','b','j'): {

//                 cout << "Create the tool in the hand, the objects on the table." <<endl;

//                 //Clear the World:
//                 simCmd = simWorld.deleteAll();
//                 writeSim(simCmd);

//                 //Create the table:
//                 simCmd = simWorld.simTable->makeObjectBottle(simWorld.objSubIndex, false);
//                 writeSim(simCmd);

//                 //Create one tool in the hand:
//                 toolIndex = controlCmd.get(1).asInt();
//                 if ( toolIndex==0 ) {
//                     toolIndex = rand() % simWorld.simTarget.size() + 1;
//                 }
//                 simWorld.simTarget[toolIndex-1]->setObjectPosition(0.23, 0.70, 0.20);    //left arm end effector position
//                 simCmd = simWorld.simTarget[toolIndex-1]->makeObjectBottle(simWorld.objSubIndex);
//                 writeSim(simCmd);
//                 //simWorld.simTarget[toolIndex-1]->setObjectRotation(70, 120, 30);
//                 simWorld.simTarget[toolIndex-1]->setObjectRotation(-65, -5, 110);
//                 simCmd = simWorld.simTarget[toolIndex-1]->rotateObjectBottle();
//                 writeSim(simCmd);
//                 simCmd = simWorld.simTarget[toolIndex-1]->grabObjectBottle(LEFT);        //left arm by default
//                 writeSim(simCmd);

//                 //Create one object:
//                 objIndex = controlCmd.get(2).asInt();
//                 if ( objIndex==0 ) {
//                     objIndex = rand() % simWorld.simTarget.size() + 1;
//                 }
//                 simWorld.simTarget[objIndex-1]->setObjectPosition(
//                                         threadTable.get(4).asDouble()+0.1,
//                                         threadTable.get(5).asDouble()+((threadTable.get(2).asDouble())/2)+0.1,
//                                         0.45);
//                 simCmd = simWorld.simTarget[objIndex-1]->makeObjectBottle(simWorld.objSubIndex);
//                 writeSim(simCmd);

//                 //reply to the control manager the tool and object IDs
//                 controlCmd.clear();
//                 controlCmd.addVocab(code);
//                 controlCmd.addInt(toolIndex);
//                 controlCmd.addInt(objIndex);
//                 replyCmd(controlCmd);

//                 break;
//             }
//             case VOCAB4('d','e','s','c'): {
//                 //moves table to pos:
//                 simWorld.simTable->setObjectPosition(threadTable.get(4).asDouble(),
//                                                      threadTable.get(5).asDouble(),
//                                                      threadTable.get(6).asDouble());
//                 simCmd = simWorld.simTable->moveObjectBottle();
//                 writeSim(simCmd);

//                 //moves object to table
//                 objIndex = controlCmd.get(1).asInt();
//                 if ( objIndex==0 ) {
//                     objIndex = rand() % simWorld.simTarget.size() + 1;
//                 }
//                 simWorld.simTarget[objIndex-1]->setObjectPosition(threadTable.get(4).asDouble()+0.05,
//                                                                   threadTable.get(5).asDouble()+((threadTable.get(2).asDouble())/2)+0.1,
//                                                                   0.35);
//                 simCmd = simWorld.simTarget[objIndex-1]->moveObjectBottle();
//                 writeSim(simCmd);
//                 simWorld.simTarget[objIndex-1]->setObjectRotation(0, -50, 0);
//                 simCmd = simWorld.simTarget[objIndex-1]->rotateObjectBottle();
//                 writeSim(simCmd);

//                 //reply to the control manager
//                 controlCmd.clear();
//                 controlCmd.addVocab(code);
//                 replyCmd(controlCmd);
//                 break;
//             }
//             case VOCAB3('e','f','f'): {
//                 //moves table to pos:
//                 simWorld.simTable->setObjectPosition(threadTable.get(4).asDouble(),
//                                                      threadTable.get(5).asDouble(),
//                                                      threadTable.get(6).asDouble());
//                 simCmd = simWorld.simTable->moveObjectBottle();
//                 writeSim(simCmd);

//                 //moves object to table
//                 objIndex = controlCmd.get(1).asInt();
//                 if ( objIndex==0 ) {
//                     objIndex = rand() % simWorld.simTarget.size() + 1;
//                 }
//                 simWorld.simTarget[objIndex-1]->setObjectPosition(threadTable.get(4).asDouble()+0.05,
//                                                                   threadTable.get(5).asDouble()+((threadTable.get(2).asDouble())/2)+0.1,
//                                                                   0.35);
//                 simCmd = simWorld.simTarget[objIndex-1]->moveObjectBottle();
//                 writeSim(simCmd);
//                 simWorld.simTarget[objIndex-1]->setObjectRotation(0, -50, 0);
//                 simCmd = simWorld.simTarget[objIndex-1]->rotateObjectBottle();
//                 writeSim(simCmd);

//                 //reply to the control manager
//                 controlCmd.clear();
//                 controlCmd.addVocab(code);
//                 replyCmd(controlCmd);
//                 break;
//             }
//             case VOCAB4('g','r','a','b'): {
//                 // moves tool to hand
//                 toolIndex = controlCmd.get(1).asInt();
//                 if ( toolIndex==0 ) {
//                     toolIndex = rand() % simWorld.simTool.size() + 1;
//                 }
//                 simWorld.simTool[toolIndex-1]->setObjectPosition(0.23, 0.70, 0.20);    //left arm end effector position
//                 simCmd = simWorld.simTool[toolIndex-1]->moveObjectBottle();
//                 writeSim(simCmd);
//                 //simWorld.simTool[toolIndex-1]->setObjectRotation(70, 120, 30);
//                 simWorld.simTool[toolIndex-1]->setObjectRotation(-65, -5, 110);
//                 simCmd = simWorld.simTool[toolIndex-1]->rotateObjectBottle();
//                 writeSim(simCmd);
//                 simCmd = simWorld.simTool[toolIndex-1]->grabObjectBottle(LEFT);        //left arm by default
//                 writeSim(simCmd);

//                 //reply to the control manager
//                 controlCmd.clear();
//                 controlCmd.addVocab(code);
//                 replyCmd(controlCmd);
//                 break;
//             }
//             case VOCAB4('d','r','o','p'): {
//                 //magnet off
//                 toolIndex = controlCmd.get(1).asInt();

//                 simCmd = simWorld.simTool[toolIndex-1]->dropObjectBottle(LEFT);        //left arm by default
//                 writeSim(simCmd);
//                 //moves tool to pos0
//                 simWorld.simTool[toolIndex-1]->setObjectPosition();
//                 simCmd = simWorld.simTool[toolIndex-1]->moveObjectBottle();
//                 writeSim(simCmd);

//                 //reply to the control manager
//                 controlCmd.clear();
//                 controlCmd.addVocab(code);
//                 replyCmd(controlCmd);
//                 break;
//             }
//             case VOCAB4('c','l','e','a'): {
//                 clearWorld();

//                 //reply to the control manager
//                 controlCmd.clear();
//                 controlCmd.addVocab(code);
//                 replyCmd(controlCmd);
//                 break;
//             }
//             case VOCAB4('r','e','m','o'): {
//                 //moves object to pos 0
//                  objIndex = controlCmd.get(1).asInt();
//                 if ( objIndex==0 ) {
//                     objIndex = rand() % simWorld.simTarget.size() + 1;
//                 }
//                 simWorld.simTarget[objIndex-1]->setObjectPosition();
//                 simCmd = simWorld.simTarget[objIndex-1]->moveObjectBottle();
//                 writeSim(simCmd);
//                 //moves table to pos 0
//                 simWorld.simTable->setObjectPosition();
//                 simCmd = simWorld.simTable->moveObjectBottle();
//                 writeSim(simCmd);

//                 //reply to the control manager
//                 controlCmd.clear();
//                 controlCmd.addVocab(code);
//                 replyCmd(controlCmd);
//                 break;
//             }
//         }
//     }
// }

// void CtrlThread::writeSim(Bottle cmd) {

//     Bottle response;
//     (*simObjLoaderSimOutputPort).write(cmd,response);
// }

// void CtrlThread::clearWorld() {

//     Bottle simCmd;

//     for ( int n=0 ; n<threadObject.size() ; n++ ) {
//         simWorld.simTarget[n]->setObjectPosition();
//         simCmd = simWorld.simTarget[n]->moveObjectBottle();
//         writeSim(simCmd);
//         Time::delay(0.1);
//     }

//     for ( int n=0 ; n<threadObject.size() ; n++ ) {
//         simWorld.simTool[n]->setObjectPosition();
//         simCmd = simWorld.simTool[n]->moveObjectBottle();
//         writeSim(simCmd);
//         Time::delay(0.1);
//     }

//     Time::delay(0.1);

//     simWorld.simTable->setObjectPosition();
//     simCmd = simWorld.simTable->moveObjectBottle();
//     writeSim(simCmd);
// }

// void CtrlThread::replyCmd(Bottle cmd) {

//     (*simObjLoaderCmdInputPort).reply(cmd);
// }

void SimObject::setObjectPosition(double posx, double posy,  double posz) {

    positionX = posx;
    positionY = posy;
    positionZ = posz;
}


void SimObject::setObjectRotation(double rotx, double roty,  double rotz) {

    rotationX = rotx;
    rotationY = roty;
    rotationZ = rotz;
}

void SimObject::setObjectColor(double red,  double green, double blue) {

    colorR = red;
    colorG = green;
    colorB = blue;
}

SimBox::SimBox(
               double posx,  double posy,  double posz,
               double rotx,  double roty,  double rotz,
               double red,   double green, double blue,
               double sizx,  double sizy,  double sizz) {

    

    positionX = posx;
    positionY = posy;
    positionZ = posz;

    rotationX = rotx;
    rotationY = roty;
    rotationZ = rotz;

    colorR = red;
    colorG = green;
    colorB = blue;

    sizeX = sizx;
    sizeY = sizy;
    sizeZ = sizz;
}

Bottle SimBox::makeObjectBottle(vector<int>& ind, bool collision) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("box");
    cmd.addDouble(sizeX);
    cmd.addDouble(sizeY);
    cmd.addDouble(sizeZ);
    cmd.addDouble(positionX);
    cmd.addDouble(positionY);
    cmd.addDouble(positionZ);
    cmd.addDouble(colorR);
    cmd.addDouble(colorG);
    cmd.addDouble(colorB);
    if (collision == false) {
        cmd.addString("false");
    }

    ind[BOX]++;
    objSubIndex=ind[BOX];

    return cmd;
}

Bottle SimBox::moveObjectBottle() {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("set");
    cmd.addString("box");
    cmd.addInt   (objSubIndex);
    cmd.addDouble(positionX);
    cmd.addDouble(positionY);
    cmd.addDouble(positionZ);
    return cmd;
}

Bottle SimBox::rotateObjectBottle() {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("rot");
    cmd.addString("box");
    cmd.addInt   (objSubIndex);
    cmd.addDouble(rotationX);
    cmd.addDouble(rotationY);
    cmd.addDouble(rotationZ);
    return cmd;
}

Bottle SimBox::grabObjectBottle(iCubArm arm) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("grab");
    cmd.addString("box");
    cmd.addInt   (objSubIndex);
    switch(arm) {
        case RIGHT:
            cmd.addString("right");
            break;
        case LEFT:
            cmd.addString("left");
            break;
        default:
            cmd.addString("right");
    }
    cmd.addInt(1);
    return cmd;
}

Bottle SimBox::dropObjectBottle(iCubArm arm) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("grab");
    cmd.addString("box");
    cmd.addInt   (objSubIndex);
    switch(arm) {
        case RIGHT:
            cmd.addString("right");
            break;
        case LEFT:
            cmd.addString("left");
            break;
        default:
            cmd.addString("right");
    }
    cmd.addInt(0);
    return cmd;
}

Bottle SimBox::deleteObject() {
    //not done yet, needs to delete every object and then recreate every one except the one to be deleted
    Bottle cmd;

    return cmd;
}

SimSBox::SimSBox(
                 double posx,  double posy,  double posz,
                 double rotx,  double roty,  double rotz,
                 double red,   double green, double blue,
                 double sizx,  double sizy,  double sizz) {

    

    positionX = posx;
    positionY = posy;
    positionZ = posz;

    rotationX = rotx;
    rotationY = roty;
    rotationZ = rotz;

    colorR = red;
    colorG = green;
    colorB = blue;

    sizeX = sizx;
    sizeY = sizy;
    sizeZ = sizz;
}

Bottle SimSBox::makeObjectBottle(vector<int>& ind, bool collision) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("sbox");
    cmd.addDouble(sizeX);
    cmd.addDouble(sizeY);
    cmd.addDouble(sizeZ);
    cmd.addDouble(positionX);
    cmd.addDouble(positionY);
    cmd.addDouble(positionZ);
    cmd.addDouble(colorR);
    cmd.addDouble(colorG);
    cmd.addDouble(colorB);
    if (collision == false) {
        cmd.addString("false");
    }

    ind[SBOX]++;
    objSubIndex=ind[SBOX];
    std::cout << "2" << ind[SBOX] <<", " << objSubIndex << std::endl;

    return cmd;
}

Bottle SimSBox::deleteObject() {
    //not done yet, needs to delete every object and then recreate every one except the one to be deleted
    Bottle cmd;

    return cmd;
}

Bottle SimSBox::moveObjectBottle() {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("set");
    cmd.addString("sbox");
    cmd.addInt   (objSubIndex);
    cmd.addDouble(positionX);
    cmd.addDouble(positionY);
    cmd.addDouble(positionZ);
    return cmd;
}

Bottle SimSBox::rotateObjectBottle() {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("rot");
    cmd.addString("sbox");
    cmd.addInt   (objSubIndex);
    cmd.addDouble(rotationX);
    cmd.addDouble(rotationY);
    cmd.addDouble(rotationZ);
    return cmd;
}

Bottle SimSBox::grabObjectBottle(iCubArm arm) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("grab");
    cmd.addString("sbox");
    cmd.addInt   (objSubIndex);
    switch(arm) {
        case RIGHT:
            cmd.addString("right");
            break;
        case LEFT:
            cmd.addString("left");
            break;
        default:
            cmd.addString("right");
    }
    cmd.addInt(1);
    return cmd;
}

Bottle SimSBox::dropObjectBottle(iCubArm arm) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("grab");
    cmd.addString("sbox");
    cmd.addInt   (objSubIndex);
    switch(arm) {
        case RIGHT:
            cmd.addString("right");
            break;
        case LEFT:
            cmd.addString("left");
            break;
        default:
            cmd.addString("right");
    }
    cmd.addInt(0);
    return cmd;
}

SimSph::SimSph(
               double posx,  double posy,  double posz,
               double rotx,  double roty,  double rotz,
               double red,   double green, double blue,
               double rad) {

    

    positionX = posx;
    positionY = posy;
    positionZ = posz;

    rotationX = rotx;
    rotationY = roty;
    rotationZ = rotz;

    colorR = red;
    colorG = green;
    colorB = blue;

    radius = rad;
}

Bottle SimSph::makeObjectBottle(vector<int>& ind, bool collision) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("sph");
    cmd.addDouble(radius);
    cmd.addDouble(positionX);
    cmd.addDouble(positionY);
    cmd.addDouble(positionZ);
    cmd.addDouble(colorR);
    cmd.addDouble(colorG);
    cmd.addDouble(colorB);
    if (collision == false) {
        cmd.addString("false");
    }

    ind[SPH]++;
    objSubIndex=ind[SPH];

    return cmd;
}

Bottle SimSph::deleteObject() {
    //not done yet, needs to delete every object and then recreate every one except the one to be deleted
    Bottle cmd;

    return cmd;
}

Bottle SimSph::moveObjectBottle() {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("set");
    cmd.addString("sph");
    cmd.addInt   (objSubIndex);
    cmd.addDouble(positionX);
    cmd.addDouble(positionY);
    cmd.addDouble(positionZ);
    return cmd;
}

Bottle SimSph::rotateObjectBottle() {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("rot");
    cmd.addString("sph");
    cmd.addInt   (objSubIndex);
    cmd.addDouble(rotationX);
    cmd.addDouble(rotationY);
    cmd.addDouble(rotationZ);
    return cmd;
}

Bottle SimSph::grabObjectBottle(iCubArm arm) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("grab");
    cmd.addString("sph");
    cmd.addInt   (objSubIndex);
    switch(arm) {
        case RIGHT:
            cmd.addString("right");
            break;
        case LEFT:
            cmd.addString("left");
            break;
        default:
            cmd.addString("right");
    }
    cmd.addInt(1);
    return cmd;
}

Bottle SimSph::dropObjectBottle(iCubArm arm) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("grab");
    cmd.addString("sph");
    cmd.addInt   (objSubIndex);
    switch(arm) {
        case RIGHT:
            cmd.addString("right");
            break;
        case LEFT:
            cmd.addString("left");
            break;
        default:
            cmd.addString("right");
    }
    cmd.addInt(0);
    return cmd;
}

SimSSph::SimSSph(
                 double posx,  double posy,  double posz,
                 double rotx,  double roty,  double rotz,
                 double red,   double green, double blue,
                 double rad) {

    

    positionX = posx;
    positionY = posy;
    positionZ = posz;

    rotationX = rotx;
    rotationY = roty;
    rotationZ = rotz;

    colorR = red;
    colorG = green;
    colorB = blue;

    radius = rad;
}

Bottle SimSSph::makeObjectBottle(vector<int>& ind, bool collision) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("ssph");
    cmd.addDouble(radius);
    cmd.addDouble(positionX);
    cmd.addDouble(positionY);
    cmd.addDouble(positionZ);
    cmd.addDouble(colorR);
    cmd.addDouble(colorG);
    cmd.addDouble(colorB);
    if (collision == false) {
        cmd.addString("false");
    }

    ind[SSPH]++;
    objSubIndex=ind[SSPH];

    return cmd;
}

Bottle SimSSph::deleteObject() {
    //not done yet, needs to delete every object and then recreate every one except the one to be deleted
    Bottle cmd;

    return cmd;
}

Bottle SimSSph::moveObjectBottle() {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("set");
    cmd.addString("ssph");
    cmd.addInt   (objSubIndex);
    cmd.addDouble(positionX);
    cmd.addDouble(positionY);
    cmd.addDouble(positionZ);
    return cmd;
}

Bottle SimSSph::rotateObjectBottle() {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("rot");
    cmd.addString("ssph");
    cmd.addInt   (objSubIndex);
    cmd.addDouble(rotationX);
    cmd.addDouble(rotationY);
    cmd.addDouble(rotationZ);
    return cmd;
}

Bottle SimSSph::grabObjectBottle(iCubArm arm) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("grab");
    cmd.addString("sshp");
    cmd.addInt   (objSubIndex);
    switch(arm) {
        case RIGHT:
            cmd.addString("right");
            break;
        case LEFT:
            cmd.addString("left");
            break;
        default:
            cmd.addString("right");
    }
    cmd.addInt(1);
    return cmd;
}

Bottle SimSSph::dropObjectBottle(iCubArm arm) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("grab");
    cmd.addString("ssph");
    cmd.addInt   (objSubIndex);
    switch(arm) {
        case RIGHT:
            cmd.addString("right");
            break;
        case LEFT:
            cmd.addString("left");
            break;
        default:
            cmd.addString("right");
    }
    cmd.addInt(0);
    return cmd;
}

SimCyl::SimCyl(
               double posx,  double posy,  double posz,
               double rotx,  double roty,  double rotz,
               double red,   double green, double blue,
               double rad,   double hei) {

    

    positionX = posx;
    positionY = posy;
    positionZ = posz;

    rotationX = rotx;
    rotationY = roty;
    rotationZ = rotz;

    colorR = red;
    colorG = green;
    colorB = blue;

    radius = rad;
    height = hei;
}

Bottle SimCyl::makeObjectBottle(vector<int>& ind, bool collision) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("cyl");
    cmd.addDouble(radius);
    cmd.addDouble(height);
    cmd.addDouble(positionX);
    cmd.addDouble(positionY);
    cmd.addDouble(positionZ);
    cmd.addDouble(colorR);
    cmd.addDouble(colorG);
    cmd.addDouble(colorB);
    if (collision == false) {
        cmd.addString("false");
    }

    ind[CYL]++;
    objSubIndex=ind[CYL];

    return cmd;
}

Bottle SimCyl::deleteObject() {
    //not done yet, needs to delete every object and then recreate every one except the one to be deleted
    Bottle cmd;

    return cmd;
}

Bottle SimCyl::moveObjectBottle() {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("set");
    cmd.addString("cyl");
    cmd.addInt   (objSubIndex);
    cmd.addDouble(positionX);
    cmd.addDouble(positionY);
    cmd.addDouble(positionZ);
    return cmd;
}

Bottle SimCyl::rotateObjectBottle() {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("rot");
    cmd.addString("cyl");
    cmd.addInt   (objSubIndex);
    cmd.addDouble(rotationX);
    cmd.addDouble(rotationY);
    cmd.addDouble(rotationZ);
    return cmd;
}

Bottle SimCyl::grabObjectBottle(iCubArm arm) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("grab");
    cmd.addString("cyl");
    cmd.addInt   (objSubIndex);
    switch(arm) {
        case RIGHT:
            cmd.addString("right");
            break;
        case LEFT:
            cmd.addString("left");
            break;
        default:
            cmd.addString("right");
    }
    cmd.addInt(1);
    return cmd;
}

Bottle SimCyl::dropObjectBottle(iCubArm arm) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("grab");
    cmd.addString("cyl");
    cmd.addInt   (objSubIndex);
    switch(arm) {
        case RIGHT:
            cmd.addString("right");
            break;
        case LEFT:
            cmd.addString("left");
            break;
        default:
            cmd.addString("right");
    }
    cmd.addInt(0);
    return cmd;
}

SimSCyl::SimSCyl(
                 double posx,  double posy,  double posz,
                 double rotx,  double roty,  double rotz,
                 double red,   double green, double blue,
                 double rad,   double hei) {

    

    positionX = posx;
    positionY = posy;
    positionZ = posz;

    rotationX = rotx;
    rotationY = roty;
    rotationZ = rotz;

    colorR = red;
    colorG = green;
    colorB = blue;

    radius = rad;
    height = hei;
}

Bottle SimSCyl::makeObjectBottle(vector<int>& ind, bool collision) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("scyl");
    cmd.addDouble(radius);
    cmd.addDouble(height);
    cmd.addDouble(positionX);
    cmd.addDouble(positionY);
    cmd.addDouble(positionZ);
    cmd.addDouble(colorR);
    cmd.addDouble(colorG);
    cmd.addDouble(colorB);
    if (collision == false) {
        cmd.addString("false");
    }

    ind[SCYL]++;
    objSubIndex=ind[SCYL];

    return cmd;
}

Bottle SimSCyl::deleteObject() {
    //not done yet, needs to delete every object and then recreate every one except the one to be deleted
    Bottle cmd;

    return cmd;
}

Bottle SimSCyl::moveObjectBottle() {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("set");
    cmd.addString("scyl");
    cmd.addInt   (objSubIndex);
    cmd.addDouble(positionX);
    cmd.addDouble(positionY);
    cmd.addDouble(positionZ);
    return cmd;
}

Bottle SimSCyl::rotateObjectBottle() {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("rot");
    cmd.addString("scyl");
    cmd.addInt   (objSubIndex);
    cmd.addDouble(rotationX);
    cmd.addDouble(rotationY);
    cmd.addDouble(rotationZ);
    return cmd;
}

Bottle SimSCyl::grabObjectBottle(iCubArm arm) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("grab");
    cmd.addString("scyl");
    cmd.addInt   (objSubIndex);
    switch(arm) {
        case RIGHT:
            cmd.addString("right");
            break;
        case LEFT:
            cmd.addString("left");
            break;
        default:
            cmd.addString("right");
    }
    cmd.addInt(1);
    return cmd;
}

Bottle SimSCyl::dropObjectBottle(iCubArm arm) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("grab");
    cmd.addString("scyl");
    cmd.addInt   (objSubIndex);
    switch(arm) {
        case RIGHT:
            cmd.addString("right");
            break;
        case LEFT:
            cmd.addString("left");
            break;
        default:
            cmd.addString("right");
    }
    cmd.addInt(0);
    return cmd;
}

SimModel::SimModel(
                   double posx,  double posy,  double posz,
                   double rotx,  double roty,  double rotz,
                   std::string mes, std::string tex) {

    

    positionX = posx;
    positionY = posy;
    positionZ = posz;

    rotationX = rotx;
    rotationY = roty;
    rotationZ = rotz;

    mesh    = mes;
    texture = tex;
}

Bottle SimModel::makeObjectBottle(vector<int>& ind, bool collision) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("model");
    cmd.addString(mesh.c_str());
    cmd.addString(texture.c_str());
    cmd.addDouble(positionX);
    cmd.addDouble(positionY);
    cmd.addDouble(positionZ);
    if (collision == false) {
        cmd.addString("false");
    }

    ind[MODEL]++;
    objSubIndex=ind[MODEL];

    return cmd;
}

Bottle SimModel::deleteObject() {
    //not done yet, needs to delete every object and then recreate every one except the one to be deleted
    Bottle cmd;

    return cmd;
}

Bottle SimModel::moveObjectBottle() {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("set");
    cmd.addString("model");
    cmd.addInt   (objSubIndex);
    cmd.addDouble(positionX);
    cmd.addDouble(positionY);
    cmd.addDouble(positionZ);
    return cmd;
}

Bottle SimModel::rotateObjectBottle() {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("rot");
    cmd.addString("model");
    cmd.addInt   (objSubIndex);
    cmd.addDouble(rotationX);
    cmd.addDouble(rotationY);
    cmd.addDouble(rotationZ);
    return cmd;
}

Bottle SimModel::grabObjectBottle(iCubArm arm) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("grab");
    cmd.addString("model");
    cmd.addInt   (objSubIndex);
    switch(arm) {
        case RIGHT:
            cmd.addString("right");
            break;
        case LEFT:
            cmd.addString("left");
            break;
        default:
            cmd.addString("right");
    }
    cmd.addInt(1);
    return cmd;
}

Bottle SimModel::dropObjectBottle(iCubArm arm) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("grab");
    cmd.addString("model");
    cmd.addInt   (objSubIndex);
    switch(arm) {
        case RIGHT:
            cmd.addString("right");
            break;
        case LEFT:
            cmd.addString("left");
            break;
        default:
            cmd.addString("right");
    }
    cmd.addInt(0);
    return cmd;
}

SimSModel::SimSModel(
                     double posx,  double posy,  double posz,
                     double rotx,  double roty,  double rotz,
                     std::string mes, std::string tex) {

    

    positionX = posx;
    positionY = posy;
    positionZ = posz;

    rotationX = rotx;
    rotationY = roty;
    rotationZ = rotz;

    mesh    = mes;
    texture = tex;
}

Bottle SimSModel::makeObjectBottle(vector<int>& ind, bool collision) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("smodel");
    cmd.addString(mesh.c_str());
    cmd.addString(texture.c_str());
    cmd.addDouble(positionX);
    cmd.addDouble(positionY);
    cmd.addDouble(positionZ);
    if (collision == false) {
        cmd.addString("false");
    }

    std::cout << "ind[SMODEL] = " << ind[SMODEL] << std::endl;

    ind[SMODEL]++;
    objSubIndex=ind[SMODEL];

    return cmd;
}

Bottle SimSModel::deleteObject() {
    //not done yet, needs to delete every object and then recreate every one except the one to be deleted
    Bottle cmd;

    return cmd;
}

Bottle SimSModel::moveObjectBottle() {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("set");
    cmd.addString("smodel");
    cmd.addInt   (objSubIndex);
    cmd.addDouble(positionX);
    cmd.addDouble(positionY);
    cmd.addDouble(positionZ);
    return cmd;
}

Bottle SimSModel::rotateObjectBottle() {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("rot");
    cmd.addString("smodel");
    cmd.addInt   (objSubIndex);
    cmd.addDouble(rotationX);
    cmd.addDouble(rotationY);
    cmd.addDouble(rotationZ);
    return cmd;
}

Bottle SimSModel::grabObjectBottle(iCubArm arm) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("grab");
    cmd.addString("smodel");
    cmd.addInt   (objSubIndex);
    switch(arm) {
        case RIGHT:
            cmd.addString("right");
            break;
        case LEFT:
            cmd.addString("left");
            break;
        default:
            cmd.addString("right");
    }
    cmd.addInt(1);
    return cmd;
}

Bottle SimSModel::dropObjectBottle(iCubArm arm) {

    Bottle cmd;
    cmd.addString("world");
    cmd.addString("grab");
    cmd.addString("smodel");
    cmd.addInt   (objSubIndex);
    switch(arm) {
        case RIGHT:
            cmd.addString("right");
            break;
        case LEFT:
            cmd.addString("left");
            break;
        default:
            cmd.addString("right");
    }
    cmd.addInt(0);
    return cmd;
}

SimSMarker::SimSMarker(double posx,  double posy,  double posz,
              double rotx,  double roty,  double rotz,
              std::string mes, std::string tex, std::string type, double offset) : SimSModel(posx, posy, posz,
             rotx, roty, rotz, mes, tex) {
    if(type == "front") {
        markerPose.setIdentity();
        markerPose(2,3) = offset; 
    } else {
        markerPose.setZero();
        markerPose(2,3) = -offset;
        markerPose(0,0) = 1;
        markerPose(3,3) = 1;
        markerPose(2,2) = -1;
        markerPose(1,1) = -1;
    }
    markerPose(0,3) = -0.25;
}

Eigen::Matrix4d SimSMarker::getPose() {
    return markerPose;
}

// SimWorld::SimWorld() {

// }

// SimWorld::SimWorld(const Bottle& threadTable,
//                    vector<Bottle>& threadObject) {

//     simTarget.resize(threadObject.size());
//     simTool.resize(threadObject.size());
//     objSubIndex.resize(8);
//     for ( int n=0 ; n<8 ; n++ ) {
//         objSubIndex[n]=0;
//     }

//     simTable = new SimSBox(-8, 0.5, -4,
//                            threadTable.get(4).asDouble(),
//                            threadTable.get(5).asDouble(),
//                            threadTable.get(6).asDouble(),
//                            0, 0, 0,
//                            threadTable.get(7).asDouble(),
//                            threadTable.get(8).asDouble(),
//                            threadTable.get(9).asDouble(),
//                            threadTable.get(1).asDouble(),
//                            threadTable.get(2).asDouble(),
//                            threadTable.get(3).asDouble());
//     simTable->objSubIndex=0;

//     for ( int n=0 ; n<threadObject.size() ; n++ ) {
//         if      (threadObject[n].get(1).asString() == "Box") {
//             simTarget[n] = new SimBox(n, 0.5, -4,
//                                       threadTable.get(4).asDouble()-0.05,
//                                       threadTable.get(5).asDouble()+((threadTable.get(2).asDouble())/2)+0.1,
//                                       threadTable.get(6).asDouble()-0.01,
//                                       0, 0, 0,
//                                       threadObject[n].get(5).asDouble(),
//                                       threadObject[n].get(6).asDouble(),
//                                       threadObject[n].get(7).asDouble(),
//                                       threadObject[n].get(2).asDouble(),
//                                       threadObject[n].get(3).asDouble(),
//                                       threadObject[n].get(4).asDouble());
//         }
//         else if (threadObject[n].get(1).asString() == "Sph") {
//             simTarget[n] = new SimSph(n, 0.5, -4,
//                                       threadTable.get(4).asDouble()-0.05,
//                                       threadTable.get(5).asDouble()+((threadTable.get(2).asDouble())/2)+0.1,
//                                       threadTable.get(6).asDouble()-0.01,
//                                       0, 0, 0,
//                                       threadObject[n].get(3).asDouble(),
//                                       threadObject[n].get(4).asDouble(),
//                                       threadObject[n].get(5).asDouble(),
//                                       threadObject[n].get(2).asDouble());
//         }
//         else if (threadObject[n].get(1).asString() == "Cyl") {
//             simTarget[n] = new SimCyl(n, 0.5, -4,
//                                       threadTable.get(4).asDouble()-0.05,
//                                       threadTable.get(5).asDouble()+0.2,
//                                       threadTable.get(6).asDouble()-0.01,
//                                       0, 0, 0,
//                                       threadObject[n].get(4).asDouble(),
//                                       threadObject[n].get(5).asDouble(),
//                                       threadObject[n].get(6).asDouble(),
//                                       threadObject[n].get(2).asDouble(),
//                                       threadObject[n].get(3).asDouble());
//         }
//         else if (threadObject[n].get(1).asString() == "Model") {
//             simTarget[n] = new SimModel(n, 0.5, -4,
//                                         threadTable.get(4).asDouble()-0.05,
//                                         threadTable.get(5).asDouble()+((threadTable.get(2).asDouble())/2)+0.1,
//                                         threadTable.get(6).asDouble()-0.01,
//                                         0, 0, 0,
//                                         threadObject[n].get(2).asString(),
//                                         threadObject[n].get(3).asString());
//         }
//         simTarget[n]->objSubIndex=0;
//     }

//     for ( int n=0 ; n<threadObject.size() ; n++ ) {
//         if      (threadObject[n].get(1).asString() == "Box") {
//             simTool[n] = new SimBox(n, 0.5, -5,
//                                       threadTable.get(4).asDouble()-0.05,
//                                       threadTable.get(5).asDouble()+((threadTable.get(2).asDouble())/2)+0.1,
//                                       threadTable.get(6).asDouble()-0.01,
//                                       0, 0, 0,
//                                       threadObject[n].get(5).asDouble(),
//                                       threadObject[n].get(6).asDouble(),
//                                       threadObject[n].get(7).asDouble(),
//                                       threadObject[n].get(2).asDouble(),
//                                       threadObject[n].get(3).asDouble(),
//                                       threadObject[n].get(4).asDouble());
//         }
//         else if (threadObject[n].get(1).asString() == "Sph") {
//             simTool[n] = new SimSph(n, 0.5, -5,
//                                       threadTable.get(4).asDouble()-0.05,
//                                       threadTable.get(5).asDouble()+((threadTable.get(2).asDouble())/2)+0.1,
//                                       threadTable.get(6).asDouble()-0.01,
//                                       0, 0, 0,
//                                       threadObject[n].get(3).asDouble(),
//                                       threadObject[n].get(4).asDouble(),
//                                       threadObject[n].get(5).asDouble(),
//                                       threadObject[n].get(2).asDouble());
//         }
//         else if (threadObject[n].get(1).asString() == "Cyl") {
//             simTool[n] = new SimCyl(n, 0.5, -5,
//                                       threadTable.get(4).asDouble()-0.05,
//                                       threadTable.get(5).asDouble()+0.2,
//                                       threadTable.get(6).asDouble()-0.01,
//                                       0, 0, 0,
//                                       threadObject[n].get(4).asDouble(),
//                                       threadObject[n].get(5).asDouble(),
//                                       threadObject[n].get(6).asDouble(),
//                                       threadObject[n].get(2).asDouble(),
//                                       threadObject[n].get(3).asDouble());
//         }
//         else if (threadObject[n].get(1).asString() == "Model") {
//             simTool[n] = new SimModel(n, 0.5, -5,
//                                         threadTable.get(4).asDouble()-0.05,
//                                         threadTable.get(5).asDouble()+((threadTable.get(2).asDouble())/2)+0.1,
//                                         threadTable.get(6).asDouble()-0.01,
//                                         0, 0, 0,
//                                         threadObject[n].get(2).asString(),
//                                         threadObject[n].get(3).asString());
//         }
//         simTool[n]->objSubIndex=0;
//     }

// }

// Bottle SimWorld::deleteAll() {

//     Bottle cmd;
//     cmd.addString("world");
//     cmd.addString("del");
//     cmd.addString("all");
//     resetSimObjectIndex();
//     return cmd;
// }

// void SimWorld::resetSimObjectIndex() {
//     simTable->objSubIndex=0;
//     for ( int n=0 ; n<simTarget.size() ; n++ ) {
//         simTarget[n]->objSubIndex=0;
//     }
//     for ( int n=0 ; n<8 ; n++ ) {
//         objSubIndex[n]=0;
//     }
// }
