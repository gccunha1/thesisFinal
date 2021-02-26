#include "iCubLimb.hpp"

void GenericRightArm::updateArm(std::vector<double> dhParams) {
	printf("%d",this->getDOF());
	for (int i = 0; i < 7; ++i) {
		popLink();
	}
	pushLink(new iCub::iKin::iKinLink( dhParams[0],  dhParams[1],  dhParams[2],     dhParams[3],  -95.5*M_PI/180,  9.45*M_PI/180));
	pushLink(new iCub::iKin::iKinLink( dhParams[4],  dhParams[5],  dhParams[6],     dhParams[7],             0.0, 160.8*M_PI/180));
	pushLink(new iCub::iKin::iKinLink( dhParams[8],  dhParams[9], dhParams[10],    dhParams[11],  -37.0*M_PI/180,  79.5*M_PI/180));
	pushLink(new iCub::iKin::iKinLink(dhParams[12], dhParams[13], dhParams[14],    dhParams[15], 15.385*M_PI/180,105.88*M_PI/180));
	pushLink(new iCub::iKin::iKinLink(dhParams[16], dhParams[17], dhParams[18],    dhParams[19],  -90.0*M_PI/180,  90.0*M_PI/180));
	pushLink(new iCub::iKin::iKinLink(dhParams[20], dhParams[21], dhParams[22],    dhParams[23],  -90.0*M_PI/180,   0.0*M_PI/180));
	pushLink(new iCub::iKin::iKinLink(dhParams[24], dhParams[25], dhParams[26],    dhParams[27],  -19.8*M_PI/180,  39.6*M_PI/180));
}

