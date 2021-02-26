#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>

#define TRANSPOSED 0
#define VERTICAL 1

Eigen::Matrix4d evalDHMatrix(double a, double d, double alpha, double theta);
int threedTo1dTransform(std::vector<int> v);
std::vector<int> onedTo3dTransform(int n);
void printStdVector(std::vector<double> v);
void randomVector(std::vector<double> &v, std::vector<double> lb, std::vector<double> ub);
double vectorDistance(std::vector<double> v1, std::vector<double> v2);
void inverseKinematics(std::vector<double> &pos, std::vector<double> &ori, Eigen::Matrix4d T);
double angularDifference(double a, double b);
void computeKinematicsError(double &posError, double &oriError, Eigen::Matrix4d T1, Eigen::Matrix4d T2);
double rotMatrixError(Eigen::Matrix3d R1, Eigen::Matrix3d R2);
bool vector2file(std::vector<double> v, std::ofstream &f, bool vecPosition = VERTICAL);
double randomValueUniDistr(double lb, double ub);
bool vectorIsInBounds(std::vector<double> v, std::vector<double> lb, std::vector<double> ub);
double randomValueNormal(double mean, double stdDev);
std::vector<double> randomVectorNormal(std::vector<double> means, std::vector<double> stdDevs);
std::vector<double> randomVectorCircumference(int size, double radius);
Eigen::Matrix4d invertTransformMatrix(Eigen::Matrix4d T);
Eigen::Matrix3d eulerXYZ(double x, double y, double z);
Eigen::Matrix4d combineRotTrans(Eigen::Matrix3d rot, double x, double y, double z);
Eigen::Matrix3d axisAngle2matrix(double vx, double vy, double vz, double angle);
double predictSampleReliability(Eigen::Matrix4d pose);
double predictSampleReliability(double distance, double angle);

template<class T, class E>
void copyVector(T &dest, E &orig) {
	int i;
	dest.resize(orig.size());
	for (i = 0; i < dest.size(); ++i) {
		dest[i] = orig[i];
	}
}

template<class T>
class StatValue {
public:
	StatValue() {
		this->samples.resize(0);
		nSamples = 0;
	}
	void addValue(T newValue) {
		nSamples++;
		if(nSamples > samples.size()) {
			this->samples.resize(samples.size() + 1000);
		}
		this->samples[nSamples - 1] = newValue;
		
		if((double) newValue >= max) max = (double) newValue;
		if((double) newValue <= min) min = (double) newValue;
	};
	double getAverage() {
		return average;
	};
	double getVariance() {
		return variance;
	};
	double getStdDeviation() {
		return stdDev;
	};
	void toFile(std::ostream &f) {
		updateAverage();
		updateVariance();
		computeStats();
		f << nSamples<< ' '<< this->average << ' '<< this->stdDev << ' ' << this->min << ' ' << this->max << ' ' << this->median << ' ' << this->quart1 << ' ' << this->quart3 << std::endl;
	};

	void all2file(std::ostream &f) {
		for (int i = 0; i < nSamples; ++i) {
			f << samples[i] << " ";
		}
		f << "\n" << std::endl;
	}

	friend std::ostream& operator<<(std::ostream& f, const StatValue& dt) {
		for (int i = 0; i < dt.nSamples; ++i) {
			f << dt.samples[i] << ",";
		}
		f << std::endl;
	}

private:
	int nSamples = 0;
	double average = 0;
	double stdDev = 0;
	double variance = 0;
	double median = 0;
	double min = 100000;
	double max = -1;
	double quart1 = 0;
	double quart3 = 0;
	std::vector<T> samples;
	void updateVariance() {
		double sum = 0;
		for (int i = 0; i < nSamples; ++i) {
			sum += pow((double) samples[i] - average, 2); 
		}
		variance = sum / (nSamples - 1);
		stdDev = sqrt(variance);
	};
	void updateAverage() {
		double sum = 0;
		for (int i = 0; i < nSamples; ++i) {
			sum += (double) samples[i]; 
		}
		average = sum / nSamples;
	};
	
	void computeStats() {
		samples.resize(nSamples);
		sort(samples.begin(), samples.end()); 
		computeMedianQuartiles();
	}

	void computeMedianQuartiles() {
		if(nSamples == 0) return;
		int idx = 0;

		if (nSamples % 2 == 0) {
			idx = nSamples/2;
			median = ((double)samples[idx - 1] + (double)samples[idx]) / 2; 
			
			if (idx % 2 == 0) {
				quart1 = ((double)samples[idx/2 - 1] + (double)samples[idx/2]) / 2; 
				quart3 = ((double)samples[3*idx/2 - 1] + (double)samples[3*idx/2]) / 2;
			} else {
				quart1 = (double)samples[idx/2];
				quart3 = (double)samples[3*idx/2];
			}

			return;
		} else {
			idx = nSamples/2;
			median = (double)samples[idx];

			if (idx % 2 == 0) {
				quart1 = ((double)samples[idx/2 - 1] + (double)samples[idx/2]) / 2; 
				quart3 = ((double)samples[3*idx/2] + (double)samples[3*idx/2 + 1]) / 2;
			} else {
				quart1 = (double)samples[idx/2];
				quart3 = (double)samples[3*idx/2 + 1];
			}
			return;
		}
	}
};

class BetaProcess {
public:
	void addSuccess(std::vector<double> &newInput) {
		int idx = searchInput(newInput);
		if(idx >= 0) {
			successes[idx]++;
			return;
		}

		newEntry(newInput);
		successes[successes.size() - 1]++;
	}
	void addFailure(std::vector<double> &newInput) {
		int idx = searchInput(newInput);
		if(idx >= 0) {
			failures[idx]++;
			return;
		}

		newEntry(newInput);
		failures[failures.size() - 1]++;
		std::cout << "Added failure at: " << std::endl;
		printStdVector(newInput);
	}
	double getSuccessLikelihood(std::vector<double> &testInput) {
		double alpha0 = 1.0;
		double beta0 = 0.0;
		double alpha = alpha0, beta = beta0;
		
		for(int i = 0; i < pastInputs.size(); i++) {
			alpha += computeKernel(pastInputs[i], testInput) * successes[i];
			beta += computeKernel(pastInputs[i], testInput) * failures[i];
			//std::cout << "Successes: " << successes[i] << " Failures: " << failures[i] << std::endl;
		}

		return alpha/(alpha + beta);
	}

	void reset() {
		pastInputs.resize(0);
		successes.resize(0);
		failures.resize(0);
	}
		
private:
	int inputDim;
	std::vector<std::vector<double>> pastInputs;
	std::vector<int> successes, failures;
	std::string kernel;

	int searchInput(std::vector<double> input) {
		for (int i = 0; i < pastInputs.size(); ++i) {
			if(input == pastInputs[i])
				return i;
		}
		return -1;
	}

	void newEntry(std::vector<double> &input) {
		pastInputs.resize(pastInputs.size()+1);
		successes.resize(successes.size()+1);
		failures.resize(failures.size()+1);

		pastInputs[pastInputs.size() - 1] = input;
		successes[successes.size() - 1] = 1;
		failures[failures.size() - 1] = 0;
	}
	double computeKernel(std::vector<double> &x1, std::vector<double> &x2) {
		return sqexp(x1,x2);
	}
	double sqexp(std::vector<double> &x1, std::vector<double> &x2) {
		double ell = 10;
		double sigma = 100;
		double sum = 0;

		for (int i = 0; i < x1.size(); ++i) {
			sum += (x1[i] - x2[i])*(x1[i] - x2[i]);	
		}

		return sigma*sigma*exp(-0.5*sum/(ell*ell));
	}
};

#endif
