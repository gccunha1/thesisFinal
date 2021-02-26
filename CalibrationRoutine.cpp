#include <vector>
#include <fstream>
#include <iostream>
#include "CalibrationRoutine.hpp"
#include <time.h>
#include <ctime>
#include <chrono> 
#include <signal.h>
#include <unistd.h>
#include "utils/utils.hpp"

using namespace std::chrono; 


boost::numeric::ublas::vector<double> maxVectorElementWise(boost::numeric::ublas::vector<double> v1, boost::numeric::ublas::vector<double> v2); 
boost::numeric::ublas::vector<double> minVectorElementWise(boost::numeric::ublas::vector<double> v1, boost::numeric::ublas::vector<double> v2); 



CalibrationRoutine::CalibrationRoutine(){
	readFileParameters();
	srand(time(NULL));
	this->outputfile << std::endl;
	this->outputfile << std::endl;
	this->outputfile << "New execution" << std::endl;
	this->outputfile << "Iterations: " << this->maxIter << std::endl;
	this->outputfile << "nJoints: " << this->inputDim << std::endl;
	this->outputfile << "Active Learning: " << this->activeLearning << std::endl;
	this->outputfile << "Criteria: " << this->criteria << std::endl;
	this->outputfile << "Cost Weight: " << this->costCoef << std::endl;
	this->outputfile << "[EKF]"<< std::endl;
	this->outputfile << "p: " << this->ekf.getCoef("p1") << std::endl;
	this->outputfile << "q: " << this->ekf.getCoef("q") << std::endl;
	this->outputfile << "r: " << this->ekf.getCoef("r") << std::endl;
	this->outputfile << "Repetitions: " << this->reps << std::endl;
	time_t now = time(0);
	char* dt = ctime(&now);
	this->outputfile << "\nDate/Time: " << dt << std::endl;
	this->averagesfile << "\nDate/Time: " << dt << std::endl;
	this->trajectoryfile << "\nDate/Time: " << dt << std::endl;
	this->ekfdhfile << "\nDate/Time: " << dt << std::endl;
	this->traceFile.open("trace.txt", std::ios::out|std::ios::app);
	this->traceFile << "\nDate/Time: " << dt << std::endl;

	dtOpt.costCoef = this->costCoef;
	dtOpt.prevTheta.resize(7);
	nextJointValues.resize(7);
	prevTheta.resize(7);
	nextJointsStd.resize(7);
	
	posErrorStats.resize(maxIter+1);
	rotErrorStats.resize(maxIter+1);
	travelDistanceStats.resize(maxIter+1);
	optTime.resize(1);
	discardedSamples.resize(1);

	posErrorTravel.resize(startBinLen);
	rotErrorTravel.resize(startBinLen);
}
CalibrationRoutine::~CalibrationRoutine(){
	this->outputfile.close();
	this->trajectoryfile.close();
	this->averagesfile.close();
	this->outputTrajectoryFile.close();
	delete this->arm;

}

double CalibrationRoutine::f(const std::vector<double> &x, std::vector<double> &grad, void* f_data) {
	Eigen::VectorXd thetas, prevThetas;
	Eigen::Vector4d headPos, pos, dir, dirPret, zVec;
	Eigen::Matrix4d T;
	std::vector<double> thetasStd(7);
	boost::numeric::ublas::vector<double> query(7);
	double poseCost, betaCost = 1;
	dataOpt *dataekf = (dataOpt *) f_data;
	int i;
	bool predictErrorAL = true;

	thetas.resize(x.size());
	prevThetas.resize(x.size());

	headPos << 0, 0, 3.0, 1.0;
	zVec << 0, 0, 1.0, 1.0;

	for (i = 0; i < x.size(); ++i) {
		thetas[i] = x[i] + dataekf->boundsOffset[i];
		query[i] = thetas[i];
	}
	copyVector(thetasStd, thetas);

	T = dataekf->ekf->predictTmatrix(thetasStd);
	for (int i = 0; i < 3; ++i) {
		pos[i] = T(i,3);
	}
	pos[3] = 1.0;

	dir = T*zVec - pos;
	dirPret = headPos - pos;

	double angle = acos(abs(dir.dot(dirPret))/(dirPret.norm()*dir.norm()))*180/M_PI;
	
	poseCost = predictSampleReliability(dirPret.norm(), angle);
	
	if (predictErrorAL)
		dataekf->ekf->updateRmatrix(poseCost);
	else
		dataekf->ekf->updateRmatrix(1);
	
	return (dataekf->ekf->getPredictedCovTrace(thetas) + 10*(atan(10*pos[0]) + M_PI/2)) / (betaCost*dataekf->betaProcess->getSuccessLikelihood(thetasStd))  + dataekf->costCoef*pow(boost::numeric::ublas::norm_2(query - dataekf->prevTheta), 2);
}

void CalibrationRoutine::start(bool test){
	boost::numeric::ublas::vector<double> potLbounds(this->inputDim, 0.0), potUbounds(this->inputDim, 0.0);
	int i, j;
	double f_min, error = 0, orientationError = 0;
	std::vector<double> z, o, sample(7);
	std::vector<double> jointsAux(7, 0.0);
	std::vector<double> nextJointsStd(this->inputDim, 0.0);
	bool rBackup = false;

	binLen = startBinLen;

	printf("Ler Backup? [y/n] ");
    char c = getchar();
    if (c == 'y' || c == 'Y') {
        rBackup = true;
    }

    nlopt::opt *opt = new nlopt::opt(nlopt::GN_DIRECT, 7);
	lb.resize(7);
	ub.resize(7);
	this->dtOpt.boundsOffset.resize(7);
	copyVector(lb, this->lowerBounds);
	copyVector(ub, this->upperBounds);
	this->dtOpt.betaProcess = &this->betaProcess;
	
	for (int i = 0; i < 7; ++i) {
		this->dtOpt.boundsOffset[i] = lb[i];
		lb[i] = 0;
		ub[i] = ub[i] - this->dtOpt.boundsOffset[i];
	}
	 
	opt->set_min_objective(this->f, &this->dtOpt);
	opt->set_lower_bounds(lb);
	opt->set_upper_bounds(ub);
	opt->set_maxtime(1.0);
	this->dtOpt.ekf = &this->ekf;

	for (j = this->currentRep; j < this->reps; ++j) {
		if(!rBackup) {
			currentRep = j;
			/* code */
			this->outputfile << "Repetition: " << j << std::endl;
			this->outputTrajectoryFile << "Repetition: " << j << std::endl;
			this->trajectoryfile << "Repetition: " << j << std::endl;
			this->ekfdhfile << "Repetition: " << j << std::endl;
			this->traceFile << "Repetition: " << j << std::endl;
			this->outputfile << "[Error values] \n" << std::endl;
			std::fill(nextJointsStd.begin(), nextJointsStd.end(), 0);
			std::fill(prevTheta.begin(), prevTheta.end(), -5);
			std::fill(nextJointValues.begin(), nextJointValues.end(), 0);
			this->ekf.reset();
			betaProcess.reset();
			vector2file(this->ekf.getState(), this->ekfdhfile, TRANSPOSED);
			totalTravel = 0;
			binCnt = 0;
			iter = 0;
			discSamp = 0;
		
			this->computeRandomSpacialError(error, orientationError, totalTravel);
			binCnt++;
			this->outputfile << error << " " << orientationError << " " << totalTravel << std::endl;
			
			travelDistanceStats[0].addValue(totalTravel);
			jointsAux = generateRandomJointConfiguration();
			prevTheta[0] = jointsAux[0];   
			prevTheta[1] = jointsAux[1];
			prevTheta[2] = jointsAux[2];
			prevTheta[3] = jointsAux[3];
			prevTheta[4] = jointsAux[4];
			prevTheta[5] = jointsAux[5];
			prevTheta[6] = jointsAux[6];

			if(this->activeLearning) {
				try{
					std::vector<double> xsel(7);
				    nlopt::result result = opt->optimize(xsel, f_min);
					for (int k = 0; k < nextJointsStd.size(); ++k) {
						nextJointsStd[k] = nextJointsStd[k] + dtOpt.boundsOffset[k];
					}
				}
				catch(std::exception &e) {
				    std::cout << "nlopt failed: " << e.what() << std::endl;
				    exit(0);
				}
				nextJointValues = std2boostVector(nextJointsStd);
				std::cout << "Next joints: " << nextJointValues << std::endl;	
				std::cout << "Probability = " << betaProcess.getSuccessLikelihood(nextJointsStd) << std::endl;
			
				//random
				randomVector(nextJointsStd, boost2stdVector(this->lowerBounds), boost2stdVector(this->upperBounds));
				nextJointValues = std2boostVector(nextJointsStd);
			} else {
				//random
				randomVector(nextJointsStd, boost2stdVector(this->lowerBounds), boost2stdVector(this->upperBounds));
				nextJointValues = std2boostVector(nextJointsStd);
			}
		} else {
			readBackup();
			j = currentRep;
			rBackup = false;
		}
		

		for (i = iter; i < this->maxIter; ++i) {
			//Sample point
			iter = i+1;
			
			robot.move(boost2stdVector(nextJointValues), "right_arm");
			for (int k = 0; k < this->inputDim; ++k) {
				this->trajectoryfile << nextJointValues[k] << " ";
			}
			this->trajectoryfile << std::endl;
			

			if (!robot.getRightArmJointValues(nextJointsStd)) {
				printf("iCub_SIM crashou!\n");
				backupEverything();
				exit(0);
			}

			z = this->ekf.predict(nextJointsStd);
			
			
				

			
			if(getSample(boost2stdVector(nextJointValues), z)) {
				std::cout << "Medição: " << std::endl;
				printStdVector(z);
				vector2file(z, this->outputTrajectoryFile, TRANSPOSED);
				ekf.feedSample(nextJointsStd, z);
				error = 0;
				orientationError = 0;
				
				nextJointsStd = boost2stdVector(nextJointValues);
				betaProcess.addSuccess(nextJointsStd);
			
			} else { //robot couldn't find hand
				if (robot.getRightArmJointValues(jointsAux)) {
					/* code */
					printf("[main]: Marker not found.\n");
					discSamp++;
					betaProcess.addFailure(jointsAux);
				} else {
					printf("iCub_SIM crashou!\n");
					backupEverything();
					exit(0);
				}
			}
			if (!robot.getRightArmJointValues(jointsAux)) {
				printf("iCub_SIM crashou!\n");
				backupEverything();
				exit(0);
			}	

			copyVector(nextJointValues, nextJointsStd);

			totalTravel += boost::numeric::ublas::norm_1(nextJointValues - prevTheta);

			copyVector(prevTheta, jointsAux);

			//Select next best point
			if(this->activeLearning) {
				std::vector<double> xsel(7);
				dtOpt.prevTheta = nextJointValues;
				if(constrained) {
					potLbounds = prevTheta-this->delta*(this->upperBounds - this->lowerBounds);
					potUbounds = prevTheta+this->delta*(this->upperBounds - this->lowerBounds);

					boost::numeric::ublas::vector<double> tempLowerBounds = maxVectorElementWise(potLbounds, this->lowerBounds);
					boost::numeric::ublas::vector<double> tempUpperBounds = minVectorElementWise(potUbounds, this->upperBounds);
					std::vector<double> lb2(7);
					std::vector<double> ub2(7);
					for (int k = 0; k < 7; ++k) {
						dtOpt.boundsOffset[k] = tempLowerBounds[k];
						lb2[k] = 0;
						ub2[k] = tempUpperBounds[k] - dtOpt.boundsOffset[k];
					}
					//add movement constraints
					delete opt;
					opt = new nlopt::opt(nlopt::GN_DIRECT, 7);
					opt->set_min_objective(this->f, &this->dtOpt);
					opt->set_lower_bounds(lb2);
					opt->set_upper_bounds(ub2);
					opt->set_maxtime(1.0);
				}
				try{
					nlopt::result result = opt->optimize(xsel, f_min);
				}
				catch(std::exception &e) {
				    std::cout << "nlopt failed: " << e.what() << std::endl;
				    exit(0);
				}
				for (int k = 0; k < nextJointsStd.size(); ++k) {
					nextJointsStd[k] = xsel[k] + dtOpt.boundsOffset[k];
				}
				nextJointValues = std2boostVector(nextJointsStd);
				std::cout << "Next joints: " << nextJointValues << std::endl;	
				std::cout << "Probability = " << betaProcess.getSuccessLikelihood(nextJointsStd) << std::endl;
			} else {
				//random
				randomVector(nextJointsStd, boost2stdVector(this->lowerBounds), boost2stdVector(this->upperBounds));
				nextJointValues = std2boostVector(nextJointsStd);
			}
			
			this->computeRandomSpacialError(error, orientationError, totalTravel);
			std::cout << "ERROR = " << error << " ORIENTATION ERROR = " << orientationError << " Travel: " << totalTravel << std::endl;
			std::cout << i << std::endl;
			this->outputfile << error << " " << orientationError << " " << totalTravel << std::endl;
			vector2file(this->ekf.getState(), this->ekfdhfile, TRANSPOSED);
			

			
			travelDistanceStats[i+1].addValue(totalTravel);
			

			while(totalTravel >= binCnt*binStep) {
				binCnt++;
			}
			
		}
		

		std::cout << "The end. \n" << std::endl;
		discardedSamples[0].addValue(discSamp);
	}

	
	if (constrained){
		writeFiles("optTimeMarkersNaiveAL", "constrained", optTime, 0);
		writeFiles("positerationsMarkersNaiveAL", "constrained", posErrorStats, maxIter);
		writeFiles("rotiterationsMarkersNaiveAL", "constrained", rotErrorStats, maxIter);
		writeFiles("traveliterationsMarkersNaiveAL", "constrained", travelDistanceStats, maxIter);
		writeFiles("postravelMarkersNaiveAL", "constrained", posErrorTravel, binCnt);
		writeFiles("rottravelMarkersNaiveAL", "constrained", rotErrorTravel, binCnt);
		writeFiles("discardedSamplesNaiveAL", "constrained", discardedSamples, 0);
	} else {
		writeFiles("optTimeMarkersNaiveAL", "unconstrained", optTime, 0);
		writeFiles("positerationsMarkersNaiveAL", "unconstrained", posErrorStats, maxIter);
		writeFiles("rotiterationsMarkersNaiveAL", "unconstrained", rotErrorStats, maxIter);
		writeFiles("traveliterationsMarkersNaiveAL", "unconstrained", travelDistanceStats, maxIter);
		writeFiles("postravelMarkersNaiveAL", "unconstrained", posErrorTravel, binCnt);
		writeFiles("rottravelMarkersNaiveAL", "unconstrained", rotErrorTravel, binCnt);
		writeFiles("discardedSamplesNaiveAL", "unconstrained", discardedSamples, 0);
	} 	

	delete opt;

}

void CalibrationRoutine::writeFiles(std::string name, std::string method, std::vector<StatValue<double>> stats, int size) {
	std::ofstream statsfile;
	std::ofstream olfile;
	std::string filename, prefix;

	if(method == "constrained")
		filename = "./dist/" + method + "/" + name + this->criteria + std::to_string(this->delta) + ".txt";
	else if(method == "unconstrained")
		filename = "./dist/" + method + "/" + name + this->criteria + std::to_string(this->costCoef) + ".txt";
	else {
		filename = "./dist/" + method + "/" + name + "goal" + std::to_string(this->delta) + ".txt";
	}
	statsfile.open(filename, std::ios::out);
	statsfile << "\n\nNEW:\n\n";

	if (sampleMarker) {
		prefix = "./overleafgraphs/markers/";
	} else{
		prefix = "./overleafgraphs/";
	}

	if(this->criteria == "random") {
		filename = prefix + method + "/" + name + this->criteria + ".txt";
	} else {
		if(method == "constrained")
			filename = prefix + method + "/" + name + this->criteria + std::to_string(this->delta) + ".txt";
		else if(method == "unconstrained")
			filename = prefix + method + "/" + name + this->criteria + std::to_string(this->costCoef) + ".txt";
		else {
			filename = prefix + method + "/" + name + "goal" + std::to_string(this->delta) + ".txt";
		}
	}


	olfile.open(filename, std::ios::out);
	olfile << "n mean stddev min max median quart1 quart3\n";
	for (int i = 0; i <= size; ++i) {
		stats[i].toFile(olfile);
		stats[i].all2file(statsfile);
	}
	statsfile.close();
	olfile.close();
}




std::vector<double> CalibrationRoutine::generateRandomJointConfiguration() {
	std::vector<double> ret(this->inputDim), lb(this->lowerBounds.size()), ub(this->upperBounds.size());
	copyVector(lb, this->lowerBounds);
	copyVector(ub, this->upperBounds);
	randomVector(ret, lb, ub);
	return ret;
}


bool CalibrationRoutine::readFileParameters(){
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
		
		if(id == "iterations") {
			this->maxIter = std::stoi(value);
			avPosError.resize(this->maxIter);
			avRotError.resize(this->maxIter);
			avDistanceTraveled.resize(this->maxIter);
			std::fill(avPosError.begin(), avPosError.end(), 0);
			std::fill(avRotError.begin(), avRotError.end(), 0);
			std::fill(avDistanceTraveled.begin(), avDistanceTraveled.end(), 0);
		} else if(id == "lower_bounds") {
			i = 0;
			while ((pos = value.find(comma)) != std::string::npos) {
			    token = value.substr(0, pos);
			    this->lowerBounds[i] = std::stof(token); 
			    value.erase(0, pos + comma.length());
				i++;
			}
			token = value.substr(0, pos);
			this->lowerBounds[i] = std::stof(token);
		} else if(id == "upper_bounds") {
			i = 0;
			while ((pos = value.find(comma)) != std::string::npos) {
			    token = value.substr(0, pos);
			    this->upperBounds[i] = std::stof(token);
			    value.erase(0, pos + comma.length());
			    i++;
			}
			token = value.substr(0, pos);
			this->upperBounds[i] = std::stof(token);
		}  else if(id == "input") {
			this->inputDim = stoi(value);
			this->lowerBounds.resize(stoi(value));
			this->upperBounds.resize(stoi(value));
		} else if(id == "outputFile") {
			this->outputfile.open(value, std::ios::out|std::ios::app);
		} else if(id == "crit_name") {
			this->criteria = value;
			if (value == "random") {
				this->activeLearning = false;
			} else {
				this->activeLearning = true;
			}
		} else if(id == "costCoef") {
			this->costCoef = std::stod(value);
			std::cout << this->costCoef << std::endl;
		} else if(id == "trajectoryFile") {
			this->trajectoryfile.open(value, std::ios::out|std::ios::app);
		} else if(id == "averageFile") {
			this->averagesfile.open(value, std::ios::out|std::ios::app);
		} else if(id=="outputTrajectory") {
			this->outputTrajectoryFile.open(value, std::ios::out|std::ios::app);
		} else if(id == "reps") {
			this->reps = stoi(value);
		} else if(id == "DHfile") {
			this->ekfdhfile.open(value, std::ios::out|std::ios::app);
		} else if(id == "delta"){
			this->delta = std::stod(value);
		} else if(id == "nSamples") {
			this->nSamples = stoi(value);
		}
	}

	
	myfile.close();

	return true;
}

void CalibrationRoutine::computePredictionError(std::vector<double> joints, double &posError, double &orientationError) {
	Eigen::Matrix4d ekfT, robotT;
	robotT = this->robot.getTmatrix(joints, this->inputDim);
	ekfT = this->ekf.predictTmatrix(joints);
	computeKinematicsError(posError, orientationError, ekfT, robotT);
}



void CalibrationRoutine::computeRandomSpacialError(double &pe, double &oe, double travel) {
	int N = 1000, binCopy;
	Eigen::Matrix4d ekfT, robotT;
	double newPosError, newOriError;
	std::vector<double> joints(this->inputDim);

	oe = 0; pe = 0;
	for (int i = 0; i < N; ++i) {
		binCopy = binCnt;

		joints = generateRandomJointConfiguration();
		
		robotT = this->robot.getTmatrix(joints, this->inputDim);
		ekfT = this->ekf.predictTmatrix(joints);
		
		computeKinematicsError(newPosError, newOriError, ekfT, robotT);
		
		oe += newOriError;
		pe += newPosError;

		posErrorStats[iter].addValue(newPosError*100);
		rotErrorStats[iter].addValue(newOriError);
		
		
		while(travel >= binCopy*binStep) {
			posErrorTravel[binCopy].addValue(newPosError*100);
			rotErrorTravel[binCopy].addValue(newOriError);
			binCopy++;
			if(binCopy >= binLen) {
				posErrorTravel.resize(posErrorTravel.size()+10);
				rotErrorTravel.resize(posErrorTravel.size()+10);
				binLen += 10;
			}
		}
	}
	oe /= N;
	pe /= N;
}


std::vector<double> CalibrationRoutine::boost2stdVector(boost::numeric::ublas::vector<double> v) {
	std::vector<double> ret(v.size());
	int i;
	for (i = 0; i < v.size(); ++i) {
		ret[i] = v[i];
	}
	return ret;
}

boost::numeric::ublas::vector<double> CalibrationRoutine::std2boostVector(std::vector<double> v) {
	boost::numeric::ublas::vector<double>  ret(v.size());
	int i;
	for (i = 0; i < v.size(); ++i) {
		ret[i] = v[i];
	}
	return ret;
}

bool CalibrationRoutine::getSample(std::vector<double> q, std::vector<double> &z) {
	std::vector<double> o;
	std::vector<double> realPose, diffPoses(7);
	double max = 0, sampleReliability;

	robot.getRightArmPose(z, o, realPose, sampleReliability, this->inputDim, true);
	robot.look(z[0], z[1], z[2]);

	if(robot.getRightArmPose(z, o, realPose, sampleReliability, this->inputDim, !sampleMarker)) {
		z.insert(z.end(), o.begin(), o.end());
		std::cout << "Pose medida: " << std::endl;
		printStdVector(z);
		std::cout << "Pose real: " << std::endl;
		printStdVector(realPose);
		
		for (int i = 0; i < 7; ++i) {
			if(abs(z[i]-realPose[i]) > max) {
				max = abs(z[i]-realPose[i]);
			}
		}
		if(predictError)
			ekf.updateRmatrix(sampleReliability);

		if(this->addError && !this->sampleMarker) {
			double theta = atan2(z[4], z[3]), phi = asin(z[5]);
			std::vector<double> erro(6, 0);
			std::vector<double> stdDevs(6, 0);
			stdDevs[0] = 0.02; stdDevs[1] = 0.02; stdDevs[2] = 0.02; //dm
			stdDevs[3] = 0.08; stdDevs[4] = 0.08; stdDevs[5] = 0.08; //radians
			//adicionar erro com dist normal
			erro = randomVectorNormal(erro, stdDevs);
			z[0] += erro[0]; z[1] += erro[1]; z[2] += erro[2];
			z[6] += erro[5]; theta += erro[3]; phi += erro[4];
			z[5] = sin(phi); z[4] = sin(theta)*sqrt(1 - z[5]*z[5]); z[3] = cos(theta)*sqrt(1 - z[5]*z[5]); 
		}

		return true;
	} else return false;
}

void CalibrationRoutine::backupEverything() {
	std::ofstream f;
	f.open("execbackup.dat", std::ios::out);

	f << "CurRep:" << currentRep << std::endl;
	f << "CurIter:" << iter-1 << std::endl;
	f << "discSamp:" << discSamp << std::endl;
	f << "totalTravel:" << totalTravel << std::endl;
	f << "prevTheta:";
	for (int i = 0; i < prevTheta.size(); ++i) {
		f << prevTheta[i] << ",";
	}

	f << std::endl << "nextJointValues:";
	for (int i = 0; i < nextJointValues.size(); ++i) {
		f << nextJointValues[i] << ",";
	}

	f << std::endl << ekf << std::endl;

	
	for (int i = 0; i < posErrorStats.size(); ++i) {
		f << "posErrorStats:" << i << ",";
		f << posErrorStats[i];
	}

	for (int i = 0; i < rotErrorStats.size(); ++i) {
		f << "rotErrorStats:" << i << ",";
		f << rotErrorStats[i];
	}

	for (int i = 0; i < travelDistanceStats.size(); ++i) {
		f << "travelDistanceStats:" << i << ",";
		f << travelDistanceStats[i];
	}

	f << "optTime:";
	f << optTime[0];
	

	f << "discardedSamples:";
	f << discardedSamples[0];

	f << "binCnt:" << this->binCnt << std::endl;
	f << "binLen:" << this->binLen << std::endl;

	for (int i = 0; i < binLen; ++i) {
		f << "posErrorTravel:" << i << ",";
		f << posErrorTravel[i];
	}

	for (int i = 0; i < binLen; ++i) {
		f << "rotErrorTravel:" << i << ",";
		f << rotErrorTravel[i];
	}
}

void CalibrationRoutine::readBackup() {
	std::ifstream myfile;
	std::string line;
	std::string delimiter = ":";
	std::string comma = ",";
	std::string id, value, token;
	size_t pos = 0;
	int i;
	std::vector<double> ekfState(4*7);
	Eigen::MatrixXd ekfPmatrix(4*7,4*7);

	myfile.open("execbackup.dat");

	if(!myfile.is_open()) {
		return;
	}
	while(std::getline(myfile, line)) {
		if((pos = line.find(delimiter)) == std::string::npos) {
			continue;
		}
		id = line.substr(0, pos);
		value = line.substr(pos + delimiter.length(), line.length());
		//value.erase(value.length()-1); //apagar \n
		//std::cout << id << ":" << value << std::endl;
		if(id == "CurRep") {
			this->currentRep = stoi(value);
		} else if(id == "CurIter") {
			this->iter = stoi(value);
		} else if(id == "discSamp") {
			this->discSamp = stoi(value);
		} else if(id == "totalTravel") {
			this->totalTravel = stod(value);
		} else if(id == "discSamp") {
			this->discSamp = stoi(value);
		} else if(id == "prevTheta") {
			i = 0;
			while ((pos = value.find(comma)) != std::string::npos) {
			    token = value.substr(0, pos);
			    prevTheta[i] = std::stod(token);
			    value.erase(0, pos + comma.length());
			    i++;
			}
		} else if(id == "nextJointValues") {
			i = 0;
			while ((pos = value.find(comma)) != std::string::npos) {
			    token = value.substr(0, pos);
			    nextJointValues[i] = std::stod(token);
			    value.erase(0, pos + comma.length());
			    i++;
			}
		} else if(id == "x") {
			i = 0;
			while ((pos = value.find(comma)) != std::string::npos) {
			    token = value.substr(0, pos);
			    ekfState[i] = std::stod(token);
			    value.erase(0, pos + comma.length());
			    i++;
			}
			ekf.setState(ekfState);
		} else if(id == "P") {
			i = 0;
			while ((pos = value.find(comma)) != std::string::npos) {
			    token = value.substr(0, pos);
			    ekfPmatrix(i/28, i%28) = std::stod(token);
			    value.erase(0, pos + comma.length());
			    i++;
			}
			ekf.setPmatrix(ekfPmatrix);
		} else if(id == "posErrorStats") {
			pos = value.find(comma);
			token = value.substr(0, pos);
			i = stoi(token);
			value.erase(0, pos + comma.length());

			while ((pos = value.find(comma)) != std::string::npos) {
			    token = value.substr(0, pos);
			    posErrorStats[i].addValue(std::stod(token));
			    value.erase(0, pos + comma.length());
			}
		} else if(id == "rotErrorStats") {
			pos = value.find(comma);
			token = value.substr(0, pos);
			i = stoi(token);
			value.erase(0, pos + comma.length());

			while ((pos = value.find(comma)) != std::string::npos) {
			    token = value.substr(0, pos);
			    rotErrorStats[i].addValue(std::stod(token));
			    value.erase(0, pos + comma.length());
			}
		} else if(id == "travelDistanceStats") {
			pos = value.find(comma);
			token = value.substr(0, pos);
			i = stoi(token);
			value.erase(0, pos + comma.length());

			while ((pos = value.find(comma)) != std::string::npos) {
			    token = value.substr(0, pos);
			    travelDistanceStats[i].addValue(std::stod(token));
			    value.erase(0, pos + comma.length());
			}
		} else if(id == "optTime") {
			while ((pos = value.find(comma)) != std::string::npos) {
			    token = value.substr(0, pos);
			    optTime[0].addValue(std::stod(token));
			    value.erase(0, pos + comma.length());
			}
		} else if(id == "discardedSamples") {
			while ((pos = value.find(comma)) != std::string::npos) {
			    token = value.substr(0, pos);
			    discardedSamples[0].addValue(std::stod(token));
			    value.erase(0, pos + comma.length());
			}
		} else if(id == "binCnt") {
			this->binCnt = std::stoi(value);
		} else if(id == "binLen") {
			this->binLen = std::stoi(value);
			posErrorTravel.resize(this->binLen);
			rotErrorTravel.resize(this->binLen);
		} else if(id == "posErrorTravel") {
			pos = value.find(comma);
			token = value.substr(0, pos);
			i = stoi(token);
			value.erase(0, pos + comma.length());
			while ((pos = value.find(comma)) != std::string::npos) {
			    token = value.substr(0, pos);
			    posErrorTravel[i].addValue(std::stod(token));
			    value.erase(0, pos + comma.length());
			}
		} else if(id == "rotErrorTravel") {
			pos = value.find(comma);
			token = value.substr(0, pos);
			i = stoi(token);
			value.erase(0, pos + comma.length());
			while ((pos = value.find(comma)) != std::string::npos) {
			    token = value.substr(0, pos);
			    rotErrorTravel[i].addValue(std::stod(token));
			    value.erase(0, pos + comma.length());
			}
		} 
	}
}

boost::numeric::ublas::vector<double> maxVectorElementWise(boost::numeric::ublas::vector<double> v1, boost::numeric::ublas::vector<double> v2) {
	int i;
	boost::numeric::ublas::vector<double> ret(v1.size());
	for (int i = 0; i < v1.size(); ++i) {
		ret[i] = std::max(v1[i], v2[i]);
	}

	return ret;
}

boost::numeric::ublas::vector<double> minVectorElementWise(boost::numeric::ublas::vector<double> v1, boost::numeric::ublas::vector<double> v2) {
	int i;
	boost::numeric::ublas::vector<double> ret(v1.size());
	for (int i = 0; i < v1.size(); ++i) {
		ret[i] = std::min(v1[i], v2[i]);
	}

	return ret;
}



