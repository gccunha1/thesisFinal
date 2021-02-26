#ifndef ICUBLIMB_HPP
#define ICUBLIMB_HPP
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
 
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>

#include <vector>
#include <math.h>

//#define 	M_PI   (3.14159265358979323846)


class GenericRightArm : public iCub::iKin::iKinLimb {
public:
    GenericRightArm(std::vector<double> dhParams) : iKinLimb() {
        dhParamsLimb = dhParams;
        allocate("don't care");
    }

    void updateArm(std::vector<double> dhParams);
    
 
protected:
	std::vector<double> dhParamsLimb;

    virtual void allocate(const std::string &_type) {
        // the type is used to discriminate between left and right limb
 
        // you have to specify the rototranslational matrix H0 from the origin
        // to the root reference so as from iCub specs.
        yarp::sig::Matrix H0(4,4);
        H0.zero();
        H0(0,1)=-1.0;
        H0(1,2)=-1.0;
        H0(2,0)=1.0;
        H0(3,3)=1.0;
        setH0(H0);
          // define the links in standard D-H convention
         //                             A,        D,     alpha,           offset(*),          min theta,          max theta
        pushLink(new iCub::iKin::iKinLink(            0.32,              0.0,         M_PI/2.0,                 0.0, -22.0*M_PI/180,  84.0*M_PI/180));
        pushLink(new iCub::iKin::iKinLink(             0.0,           -0.055,         M_PI/2.0,           -M_PI/2.0, -39.0*M_PI/180,  39.0*M_PI/180));
        pushLink(new iCub::iKin::iKinLink(       -0.233647,           -1.433,         M_PI/2.0, -105.0*M_PI/180, -59.0*M_PI/180,  59.0*M_PI/180));
        pushLink(new iCub::iKin::iKinLink( dhParamsLimb[0],  dhParamsLimb[1],  dhParamsLimb[2],     dhParamsLimb[3], -95.5*M_PI/180,   9.45*M_PI/180));
        pushLink(new iCub::iKin::iKinLink( dhParamsLimb[4],  dhParamsLimb[5],  dhParamsLimb[6],     dhParamsLimb[7],                0.0, 160.8*M_PI/180));
        pushLink(new iCub::iKin::iKinLink( dhParamsLimb[8],  dhParamsLimb[9], dhParamsLimb[10],    dhParamsLimb[11], -37.0*M_PI/180,  79.5*M_PI/180));
        pushLink(new iCub::iKin::iKinLink(dhParamsLimb[12], dhParamsLimb[13], dhParamsLimb[14],    dhParamsLimb[15],   15.385*M_PI/180, 105.88*M_PI/180));
        pushLink(new iCub::iKin::iKinLink(dhParamsLimb[16], dhParamsLimb[17], dhParamsLimb[18],    dhParamsLimb[19], -90.0*M_PI/180,  90.0*M_PI/180));
        pushLink(new iCub::iKin::iKinLink(dhParamsLimb[20], dhParamsLimb[21], dhParamsLimb[22],    dhParamsLimb[23], -90.0*M_PI/180,   0.0*M_PI/180));
        pushLink(new iCub::iKin::iKinLink(dhParamsLimb[24], dhParamsLimb[25], dhParamsLimb[26],    dhParamsLimb[27], -19.8*M_PI/180,  39.6*M_PI/180));
        // (*) remind that offset is added to theta before computing the rototranslational matrix    
 
        // usually the first three links which describes the torso kinematic come
        // as blocked, i.e. they do not belong to the set of arm's dof.
        blockLink(0,0.0);
        blockLink(1,0.0);
        blockLink(2,0.0);
    }
};


#endif
