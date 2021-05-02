#ifndef CLIENT_HPP
#define CLIENT_HPP

#include <yarp/os/all.h>
#include <stdio.h>
#include <string>
#include <iostream>
using namespace yarp::os;


class YarpServer {
public:
	YarpServer(std::string serverName){
		port.open(serverName);
	}

private:
	Network yarp;
	RpcClient port;
};

class YarpClient {
public:
	YarpClient(std::string clientName, std::string serverName){
		port.open(clientName);
		std::cout << "Trying to connect to " << serverName << std::endl;
	    yarp::os::Network::connect(clientName, serverName);

	}

	bool sendMessage(){}

private:
	Network yarp;
	RpcClient port;
};


class iCubWorldClient {
public:
	Network yarp;
	RpcClient port;
	iCubWorldClient() {
		port.open("/client");

		if (port.getOutputCount()==0) {
			std::cout << "Trying to connect to /icubSim/world" << std::endl;
            yarp.connect("/client", "/icubSim/world");
        }

        Bottle setDir("world set mdir /mnt/d/models");
        sendCommand(setDir);
        Bottle delAll("world del all");
		sendCommand(delAll);
	}

	~iCubWorldClient() {
		Bottle delAll("world del all");
		sendCommand(delAll);
	}

	bool sendCommand(Bottle cmd) {
		Bottle response;
		printf("Sending message... %s\n", cmd.toString().c_str());
        port.write(cmd,response);
        printf("Got response: %s\n", response.toString().c_str());
	}
private:

};

#endif
