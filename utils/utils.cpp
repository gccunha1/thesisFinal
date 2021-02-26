#include "utils.hpp"
#include <time.h>
#include <random>
#include <unsupported/Eigen/MatrixFunctions>
#define N 2000
#define M_PI 3.14159265358979323846  /* pi */



Eigen::Matrix4d evalDHMatrix(double a, double d, double alpha, double theta) {
	Eigen::Matrix4d T;
	T << cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), cos(theta)*a,
        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), sin(theta)*a,
        0, sin(alpha), cos(alpha), d,
        0, 0, 0, 1;
    return T;
}

Eigen::Matrix4d invertTransformMatrix(Eigen::Matrix4d T) {
	Eigen::Matrix4d ret;
	Eigen::Matrix3d rot;
	Eigen::Vector3d aux;
	ret.setIdentity();

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			ret(i,j) = T(j,i);
			rot(i,j) = T(j,i);
		}
	}



	for (int i = 0; i < 3; ++i) {
		aux[i] = T(i,3);
	}

	aux = -rot*aux;

	for (int i = 0; i < 3; ++i) {
		ret(i,3) = aux[i];
	}

	return ret;

}

int threedTo1dTransform(std::vector<int> v) {
	return v[0] + N*v[1] + N*N*v[2];
}

std::vector<int> onedTo3dTransform(int n) {
	std::vector<int> ret;
	int n2 = round(n);
	int aux = n2 / N;
	ret[0] = n2 % N;
	ret[1] = aux % N;
	aux /= N;
	ret[2] = aux;

	return ret;
}


void printStdVector(std::vector<double> v) {
	for (int i = 0; i < v.size(); ++i) {
		std::cout << v[i] << " ";
	}
	std::cout << std::endl;
}

void randomVector(std::vector<double> &v, std::vector<double> lb, std::vector<double> ub) {
	for (int i = 0; i < v.size(); ++i) {
		v[i] = randomValueUniDistr(lb[i], ub[i]);
	}
}

double randomValueUniDistr(double lb, double ub) {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(lb, ub);//uniform distribution between 0 and 1
	return dis(gen);
}

void computeKinematicsError(double &posError, double &oriError, Eigen::Matrix4d T1, Eigen::Matrix4d T2) {
	std::vector<double> pos1, pos2, ori1, ori2;
	Eigen::Matrix3d rot1, rot2;
	int i, j;
	
	inverseKinematics(pos1, ori1, T1);
	inverseKinematics(pos2, ori2, T2);
	posError = vectorDistance(pos1, pos2);
	
	for (i = 0; i < 3; ++i) {
		for(j = 0; j < 3; ++j) {
			rot1(i, j) = T1(i, j);
			rot2(i, j) = T2(i, j);
		}
	}

	oriError = rotMatrixError(rot1, rot2);
	//std::cout << rot1 << "\n" << rot2 << std::endl;
}

double rotMatrixError(Eigen::Matrix3d R1, Eigen::Matrix3d R2) {
	return sqrt(((R1.transpose()*R2).log()).squaredNorm()/2);
}

double vectorDistance(std::vector<double> v1, std::vector<double> v2) {
	int i;
	double ret = 0;
	for (i = 0; i < v1.size(); ++i) {
		ret += (v1[i] - v2[i])*(v1[i] - v2[i]);
	}
	ret = sqrt(ret);
	return ret;
}

void inverseKinematics(std::vector<double> &pos, std::vector<double> &ori, Eigen::Matrix4d T) {
	double rotAngle, sum = 0.0;
	
	int i;
	
	if(pos.size() != 3) pos.resize(3);
	if(ori.size() != 4) ori.resize(4);

	for (i = 0; i < 3; i++) {
		pos[i] = T(i, 3);
		sum += T(i, i);
	}
	rotAngle = acos((sum-1)/2.0);

	if(abs(sin(rotAngle)) > 0.0001) {
		ori[0] = (1/(2*sin(rotAngle)))*(T(2,1)-T(1,2));
		ori[1] = (1/(2*sin(rotAngle)))*(T(0,2)-T(2,0));
		ori[2] = (1/(2*sin(rotAngle)))*(T(1,0)-T(0,1));
	} else {
		if (abs(rotAngle) > abs(rotAngle-M_PI)){
			ori[0] = -2;
			ori[1] = -2;
			ori[2] = -2;
			if (abs(T(0,0)+1) < 0.0002) { //rx = 0
				ori[0] = 0;
				if (abs(T(1,1)+1) < 0.0002) { //ry = 0
					ori[1] = 0;
					ori[2] = 1;
				} else if(abs(T(1,1)-1) < 0.0002) { //ry = 1
					ori[1] = 1;
					ori[2] = 0;
				} else {
					ori[1] = sqrt(0.5*(T(1,1)+1));
					ori[2] = T(1,2)/(2*ori[1]);
				}
			} else if(abs(T(0,0)-1) < 0.0002) { //rx = 1
				ori[0] = 1;
				ori[1] = 0;
				ori[2] = 0;
			} else {
				ori[0] = sqrt(0.5*(T(0,0)+1));
				
				if (abs(T(1,1)+1) < 0.0002) { //ry = 0
					ori[1] = 0;
					ori[2] = T(0,2)/(2*ori[0]);
				} else {
					ori[1] = T(0,1)/(2*ori[0]);

					if (abs(T(2,2)+1) < 0.0002) {
						ori[2] = 0;
					} else {
						ori[2] = T(0,2)/(2*ori[0]);
					}
				}
			}
		} else {
			ori[0] = 1;
			ori[1] = 0;
			ori[2] = 0;
		}
	}
	ori[3] = rotAngle;

	//printStdVector(pos);

	//printStdVector(ori);

	//std::cout << T << std::endl;
	//std::cout << (1/(2*sin(rotAngle))) << std::endl;

}

double angularDifference(double x, double y) {
	return atan2(sin(x-y), cos(x-y));
}

bool vector2file(std::vector<double> v, std::ofstream &f, bool vecPosition) {
	int i;
	if(f.is_open()) {
		if(vecPosition) {
			for (i = 0; i < v.size(); ++i) {
				f << v[i] << std::endl;
			} 	
		} else {
			for (i = 0; i < v.size(); ++i) {
				f << v[i] << " ";
			} 
			f << std::endl;
		}
		return true;
	} else return false;
}


bool vectorIsInBounds(std::vector<double> v, std::vector<double> lb, std::vector<double> ub) {
	int i;
	for (i = 0; i < v.size(); ++i) {
		if(v[i] > ub[i] || v[i] <= lb[i])
			return false;
	} 
	return true;
}

double randomValueNormal(double mean, double stdDev) {
	std::default_random_engine generator;
  	std::normal_distribution<double> distribution(mean,stdDev);
  	return distribution(generator);
}

std::vector<double> randomVectorNormal(std::vector<double> means, std::vector<double> stdDevs) {
	std::vector<double> ret(means.size());
	int i;

	for (int i = 0; i < ret.size(); ++i) {
		ret[i] = randomValueNormal(means[i], stdDevs[i]);
	}

	return ret;
}

std::vector<double> randomVectorCircumference(int size, double radius) {
	std::vector<double> ret(size), lb(size, -1), ub(size, 1);
	randomVector(ret, lb, ub);
	double sum = 0;
	for (int i = 0; i < size; ++i) {
		sum += ret[i] * ret[i];
	}
	sum = sqrt(sum);
	for (int i = 0; i < size; ++i) {
		ret[i] = radius*ret[i]/sum;
	}
	return ret;
}

Eigen::Matrix3d eulerXYZ(double g, double b, double a) {
	Eigen::Matrix3d ret;
	ret << cos(a)*cos(b),                      -cos(b)*sin(a),                     sin(b),
		   cos(g)*sin(a)+cos(a)*sin(b)*sin(g),  cos(a)*cos(g)-sin(a)*sin(b)*sin(g), -cos(b)*sin(g),
		   sin(a)*sin(g)-cos(a)*cos(g)*sin(b),  cos(a)*sin(g)+cos(g)*sin(a)*sin(b), cos(b)*cos(g);

	return ret;
}

Eigen::Matrix4d combineRotTrans(Eigen::Matrix3d rot, double x, double y, double z) {
	Eigen::Matrix4d ret;
	ret.setIdentity();
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			ret(i,j) = rot(i,j);
		}
	}
	ret(0,3) = x;
	ret(1,3) = y;
	ret(2,3) = z;

	return ret;
}


Eigen::Matrix3d axisAngle2matrix(double vx, double vy, double vz, double angle) {
	Eigen::Matrix3d ret;
	Eigen::Vector4d o;
	ret.setZero();

	o << vx, vy, vz, angle;

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			ret(i,j) = o[i]*o[j]*(1-cos(o[3]));
		}
	}


	for (int i = 0; i < 3; ++i) {
		ret(i,i) += cos(o[3]);
	}

	ret(0,1) += -o[2]*sin(o[3]);
	ret(0,2) += o[1]*sin(o[3]);
	ret(1,0) += o[2]*sin(o[3]);
	ret(1,2) += -o[0]*sin(o[3]);
	ret(2,0) += -o[1]*sin(o[3]);
	ret(2,1) += o[0]*sin(o[3]);

	return ret;
}


double predictSampleReliability(Eigen::Matrix4d pose) {
	double distance, angle;
	Eigen::Vector4d zVec;

	zVec << 0, 0, 1.0, 0.0;

	distance = sqrt(pose(0,3)*pose(0,3) + pose(1,3)*pose(1,3) + pose(2,3)*pose(2,3));

	angle = acos(-zVec.dot(pose*zVec)/(pose*zVec).norm())*180/M_PI;

	return predictSampleReliability(distance, angle); 	
}

double predictSampleReliability(double distance, double angle) {
	double ret;

	if(distance < 5) { //best
		ret = 1;
	} else if(distance >= 5 && distance < 7) {
		ret = 3;
	} else if(distance >= 7 && distance < 10) {
		ret = 4;
	} else {
		ret = 100;
		return ret;
	}

	if(angle > 30 && angle <= 50) { //best

	} else if(angle <= 30 && angle >= 0) {
		ret += 2;
	} else if(angle > 50 && angle <= 90) {
		ret += 1;
	} else {
		ret = 100;
		return ret;
	}
	ret = (distance-2)*(distance-2)/10.0 + 0.001*(angle-45)*(angle-45);
	
	return ret;
}

