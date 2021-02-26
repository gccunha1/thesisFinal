#include <yarp/os/all.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include "worldclient.hpp"
#include "simobjloader.h"
using namespace yarp::os;
int main(int argc, char *argv[]) {
    iCubWorldClient client;
    SimSBox marker(0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.05,0.005,0.05);
    std::vector<int> ind(7,0);
    double x = 0, y = 0, z = 0;
    client.sendCommand(marker.makeObjectBottle(ind, false));
    while (true) {
        marker.setObjectPosition(x,y,z);
        client.sendCommand(marker.moveObjectBottle());
        x += 0.05;
        Time::delay(1);
    }
}