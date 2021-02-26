#include "ActiveSelectionEKF.hpp"

#include <string>



ActiveSelectionEKF::ActiveSelectionEKF(ExtendedKalmanFilterBodySchema *filter, nlopt::opt *optimizer, iCubRobot *rbt) : 
	ekf{filter}, robot{rbt}, 
	opt{optimizer} {
	
	this->readFileParameters();
	this->dtOpt.ekf = this->ekf;
	this->dtOpt.boundsOffset = this->boundsOffset;
	this->dtOpt.costSensitive = this->costSensitive;
	this->dtOpt.costCoef = this->costCoef;
	this->opt->set_min_objective(this->f, &(this->dtOpt));

	this->opt->set_lower_bounds(this->lowerBounds);
	this->opt->set_upper_bounds(this->upperBounds);
	this->opt->set_maxtime(this->maxTime);

	this->outputfile << std::endl;
	this->outputfile << std::endl;
	this->outputfile << "New execution" << std::endl;
	this->outputfile << "Iterations: " << this->maxIter << std::endl;
	this->outputfile << "nJoints: " << this->inputDim << std::endl;
	this->outputfile << "Active Learning: " << this->activeLearning << std::endl;
	this->outputfile << "Cost Sensitive: " << this->costSensitive << std::endl;
	this->outputfile << "costCoef: " << this->costCoef << std::endl;
	this->outputfile << "[EKF]"<< std::endl;
	this->outputfile << "p: " << this->ekf->getCoef("p1") << std::endl;
	this->outputfile << "q: " << this->ekf->getCoef("q") << std::endl;
	this->outputfile << "r: " << this->ekf->getCoef("r") << std::endl;
}

ActiveSelectionEKF::~ActiveSelectionEKF() {
	this->outputfile.close();
}

bool ActiveSelectionEKF::readFileParameters() {
	std::ifstream myfile;
	std::string line;
	std::string delimiter = ":";
	std::string comma = ",";
	std::string id, value, token;
	size_t pos = 0;
	int i;

	myfile.open("parameters.txt");
	
	if(!myfile.is_open()) {
		return false;
	}
	while(std::getline(myfile, line)) {
		//std::cout << line << std::endl;
		if((pos = line.find(delimiter)) == std::string::npos) {
			continue;
		}
		id = line.substr(0, pos);
		value = line.substr(pos + delimiter.length(), line.length());
		value.erase(value.length()-1); //apagar \n
		if (id == "use") {
			if (value == "no") {
				this->activeLearning = false;
			}
		} else if(id == "cost") {
			if(value == "yes")
				this->costSensitive = true;
		} else if(id == "costCoef") {
			this->costCoef = std::stoi(value);
		} else if(id == "iterations") {
			this->maxIter = std::stoi(value);
		} else if(id == "lower_bounds") {
			i = 0;
			while ((pos = value.find(comma)) != std::string::npos) {
			    token = value.substr(0, pos);
			    this->lowerBounds[i] = 0;
			    this->boundsOffset[i] = std::stof(token); 
			    value.erase(0, pos + comma.length());
				i++;
			}
			token = value.substr(0, pos);
			this->lowerBounds[i] = 0;
			this->boundsOffset[i] = std::stof(token);
		} else if(id == "upper_bounds") {
			i = 0;
			while ((pos = value.find(comma)) != std::string::npos) {
			    token = value.substr(0, pos);
			    this->upperBounds[i] = std::stof(token) - this->boundsOffset[i];
			    value.erase(0, pos + comma.length());
			    i++;
			}
			token = value.substr(0, pos);
			this->upperBounds[i] = std::stof(token) - this->boundsOffset[i];
		} else if(id == "max_time") {
			this->maxTime = std::stoi(value);
		} else if(id == "input") {
			this->inputDim = std::stoi(value);
			this->lowerBounds.resize(this->inputDim);
			this->upperBounds.resize(this->inputDim);
			this->boundsOffset.resize(this->inputDim);
		} else if(id == "outputFile") {
			this->outputfile.open(value, std::ios::out|std::ios::app);
		}
	}


	myfile.close();

	return true;
}

double ActiveSelectionEKF::f(const std::vector<double> &x, std::vector<double> &grad, void* f_data) {
	Eigen::VectorXd thetas, prevThetas;
	dataOpt *dataekf = (dataOpt *) f_data;
	int i;
	thetas.resize(x.size());
	prevThetas.resize(x.size());

	for (i = 0; i < x.size(); ++i) {
		thetas[i] = x[i] + dataekf->boundsOffset[i];
		prevThetas[i] = dataekf->prevTheta[i];
	}

	if(dataekf->costSensitive)
		return dataekf->ekf->getPredictedCovTrace(thetas) + dataekf->costCoef*(thetas - prevThetas).squaredNorm();
	else
		return dataekf->ekf->getPredictedCovTrace(thetas);
}

void ActiveSelectionEKF::optimize(std::vector<double> &dhParameters){
	std::vector<double> nextJointValues(this->inputDim, 0.0);
	std::vector<double> trueNextValues(this->inputDim, 0.0);	
	int i, j;
	double f_min, error, totalTravel = 0;
	std::vector<double> z, o;
	std::vector<double> jointsAux(7);
	std::fill(jointsAux.begin(), jointsAux.end(), 0);
	

	this->outputfile << "[Error values] \n" << std::endl;

	for (i = 0; i < this->maxIter; ++i) {
		for (j = 0; j < nextJointValues.size(); ++j) {
			trueNextValues[j] = nextJointValues[j] + this->boundsOffset[j];
		}
		//printStdVector(nextJointValues);
		//printStdVector(trueNextValues);
		//Sample point
		robot->move(trueNextValues, "right_arm");
		if(robot->getRightArmPose(z, o, this->inputDim)){
			//Give point to EKF
			ekf->feedSample(trueNextValues, z);
			error = this->computeError(0.0, this->inputDim, jointsAux);
			
			//Select next best point
			dtOpt.prevTheta = nextJointValues;
			if(this->activeLearning) {
				opt->optimize(nextJointValues, f_min);
			} else {
				//random
				randomVector(nextJointValues, this->lowerBounds, this->upperBounds);
			}
		} else { //robot couldn't find hand
			//Select random joint config next
		}
		totalTravel += vectorDistance(nextJointValues, dtOpt.prevTheta);
		std::cout << "ERROR = " << error << " Travel: " << totalTravel << std::endl;
		std::cout << i << std::endl;
		this->outputfile << error << " " << totalTravel << std::endl;
	}
	dhParameters = this->ekf->getState();

	std::cout << "The end." << std::endl;
}

double ActiveSelectionEKF::computeError(double e, int n, std::vector<double> joints) {
	
	int i;
	int count = 6;
	
	if(n == 1) {
		std::vector<double> z(3), o(4);
		for (i = 0; i < count; ++i) {
			joints[n-1] = ((double) i/count)*(this->upperBounds[n-1] - this->lowerBounds[n-1]) + this->boundsOffset[n-1];
			this->robot->getRightArmPose(z, o, joints, this->inputDim);
			e += vectorDistance(ekf->predict(joints), z);
		}
	} else {
		for (i = 0; i < count; ++i) {
			joints[n-1] = ((double) i/count)*(this->upperBounds[n-1] - this->lowerBounds[n-1]) + this->boundsOffset[n-1];
			e = computeError(e, n-1, joints);
		}
	}
	return e;

	//getRightArmPose(z, o, jointAngles);
}

