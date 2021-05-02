#ifndef EFK_HPP
#define EFK_HPP

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <vector>
#include "../iCubLimb/iCubLimb.hpp"

#include "../utils/utils.hpp"

#define PARAMERROR 0.30

class ExtendedKalmanFilterBodySchema {
	public:
		ExtendedKalmanFilterBodySchema() {
			if(!readFileParameters()) {
				return;
			}
			
			this->joints.resize(this->nJoints);

			this->pMatrix.resize(this->stateDimensions, this->stateDimensions);
			this->pMatrixDest.resize(this->stateDimensions, this->stateDimensions);
			this->qMatrix.resize(this->stateDimensions, this->stateDimensions);
			this->hMatrix.resize(7, this->stateDimensions);
			this->kalmanGain.resize(this->stateDimensions, 7);

			this->rMatrix.resize(7,7);
			this->rMatrix.setIdentity();
			this->rMatrix *= this->rMatrixCoef;

			this->qMatrix.setIdentity();
			this->qMatrix *= this->qMatrixCoef;

			this->joints.setZero();
			this->pMatrix.setIdentity();
			this->pMatrix *= this->pMatrixCoef1;
			this->pMatrixDest.setIdentity();
			this->pMatrixDest /= this->stateDimensions;
			this->hMatrix.setZero();
			this->kalmanGain.setZero();
			this->arm = new GenericRightArm(this->getState());
		}

		ExtendedKalmanFilterBodySchema(double rMatrixCoef, double qMatrixCoef) {
			this->state.resize(this->stateDimensions);
			this->joints.resize(this->nJoints);

			this->pMatrix.resize(this->stateDimensions, this->stateDimensions);
			this->pMatrixDest.resize(this->stateDimensions, this->stateDimensions);
			this->qMatrix.resize(this->stateDimensions, this->stateDimensions);
			this->hMatrix.resize(7, this->stateDimensions);
			this->kalmanGain.resize(this->stateDimensions, 7);

			this->rMatrix.setIdentity();
			this->rMatrix *= rMatrixCoef;

			this->qMatrix.setIdentity();
			this->qMatrix *= qMatrixCoef;

			this->state.setZero();
			this->joints.setZero();
			this->pMatrixDest.setIdentity();
			//this->pMatrixDest /= this->stateDimensions;
			//std::cout << "Pdest: \n"<< this->pMatrixDest << std::endl;
			this->pMatrix.setIdentity();
			this->pMatrix *= pMatrixCoef1;
			this->hMatrix.setZero();
			this->kalmanGain.setZero();

			this->arm = new GenericRightArm(this->getState());

		}

		~ExtendedKalmanFilterBodySchema() {
			std::cout << "DH parameters: \n" << this->state << std::endl;
		}

		void reset() {
			readFileParameters();
			this->rMatrix.setIdentity();
			this->rMatrix *= rMatrixCoef;

			this->qMatrix.setIdentity();
			this->qMatrix *= qMatrixCoef;

			this->joints.setZero();
			this->pMatrix.setIdentity();
			this->pMatrix *= pMatrixCoef1;
			this->hMatrix.setZero();
			this->kalmanGain.setZero();
			this->arm->updateArm(this->getState());
		}

		void updateRmatrix(double val) {
			this->rMatrix.setIdentity();
			this->rMatrix *= val;

			//printf("[EKF]: Updating R to %f\n", val);
		}

		std::vector<double> feedSample(std::vector<double> jointValues, std::vector<double> measurement) {
			Eigen::VectorXd jnts(jointValues.size());
			Eigen::VectorXd meas(7);
			Eigen::MatrixXd sMatrix(7,7);
			Eigen::VectorXd error(7);
			std::vector<double> ret(7);

			int i;
			for (i = 0; i < jointValues.size(); ++i) {
				jnts[i] = jointValues[i];
			}
			for (i = 0; i < 7; ++i)	{
				meas[i] = measurement[i];
			}

			//Predict step
			Eigen::VectorXd zpred = h(jnts);
			this->hMatrix = getJacobianH(jnts);
			
			//this->qMatrix = this->hMatrix*(this->pMatrixDest*this->hMatrix.transpose()*this->hMatrix*this->pMatrixDest)*this->hMatrix.transpose();
			
			//this->qMatrix = this->qMatrix*(this->rMatrix + this->hMatrix*this->pMatrixDest*this->hMatrix.transpose()).inverse(); 
			this->pMatrix += this->qMatrix;

			error = meas-zpred;
			error[6] = angularDifference(meas[6], zpred[6]);
			//std::cout << "error1: \n"<< error << std::endl;
			//Update step
			
			//std::cout << "H: \n"<< this->hMatrix << std::endl;
			sMatrix = this->hMatrix*this->pMatrix*this->hMatrix.transpose() + this->rMatrix;
			this->kalmanGain = this->pMatrix*this->hMatrix.transpose()*sMatrix.inverse();
			//std::cout << "K: \n"<< this->kalmanGain << std::endl;
			this->state = this->state + this->kalmanGain*error;
			this->arm->updateArm(this->getState()); //update kin chain
			//std::cout << "delta: \n"<< this->kalmanGain*error << std::endl;
			//std::cout << "meas: \n"<< meas << std::endl;
			std::cout << "zpred: \n"<< zpred << std::endl;
			
			//this->pMatrix = (Eigen::MatrixXd::Identity(this->stateDimensions, this->stateDimensions) - this->kalmanGain*this->hMatrix) * this->pMatrix;
			this->pMatrix = this->pMatrix - this->kalmanGain*sMatrix*this->kalmanGain.transpose();

			//std::cout << "P: \n"<< this->pMatrix << std::endl;
			//std::cout << "state: \n"<< this->state << std::endl;
			for (i = 0; i < 7; ++i)	{
				ret[i] = zpred[i];
			}
			error = meas-h(jnts);
			//std::cout << "error2: \n"<< error << std::endl;
			return ret;
		}

		std::vector<double> predict(std::vector<double> jointValues) {
			//check vector size
			int i;
			std::vector<double> ret(7);
			Eigen::VectorXd jnts(jointValues.size());
			
			for (i = 0; i < jointValues.size(); ++i) {
				jnts[i] = jointValues[i];
			}
			Eigen::VectorXd z = h(jnts);
			for (i = 0; i < z.size(); ++i) {
				ret[i] = z[i];
			}
			return ret;
		}

		void predictStep() {
			this->pMatrix += this->qMatrix;
		}

		Eigen::Matrix4d predictTmatrix(std::vector<double> jointValues) {
			int i;
			Eigen::Matrix4d T;
			T.setZero();
	        T(0,1)=-1.0;
	        T(1,2)=-1.0;
	        T(2,0)=1.0;
	        T(3,3)=1.0;
			//T.setIdentity();
			T = T*evalDHMatrix(      0.32,       0,     M_PI/2,    0);
			T = T*evalDHMatrix(       0,    -0.055,     M_PI/2,    -M_PI/2);
			T = T*evalDHMatrix(-0.233647,  -1.433,     M_PI/2,    -15*M_PI/180 - M_PI/2 );
			for (i = 0; i < this->nJoints; ++i) {
				T = T*evalDHMatrix(this->state[4*i], this->state[4*i + 1], this->state[4*i + 2], this->state[4*i + 3] + jointValues[i]*M_PI/180);
				//std::cout << this->state[4*i] << ", " << this->state[4*i+1] << ", " << this->state[4*i+2] << ", " << jointValues[i]*M_PI/180 + this->state[4*i+3] << std::endl;
			}
			return T;
		}

		double getPredictedCovTrace(Eigen::VectorXd jointValues) {
			Eigen::MatrixXd pred(this->stateDimensions, this->stateDimensions);
			this->hMatrix = getJacobianH(jointValues);
			
			this->kalmanGain = this->pMatrix*(this->hMatrix.transpose())*(this->hMatrix*(this->pMatrix + this->qMatrix)*this->hMatrix.transpose() + this->rMatrix).inverse();
			pred = (Eigen::MatrixXd::Identity(this->stateDimensions, this->stateDimensions) - this->kalmanGain*this->hMatrix)*(this->pMatrix + this->qMatrix);
			//std::cout << "Traço P: " << pred.trace() << std::endl;
			return pred.trace();
		}

		Eigen::MatrixXd getPredictedCov(Eigen::MatrixXd lastP, Eigen::VectorXd jointValues) {
			Eigen::MatrixXd ret(this->stateDimensions, this->stateDimensions);
			this->hMatrix = getJacobianH(jointValues);
			
			this->kalmanGain = lastP*(this->hMatrix.transpose())*(this->hMatrix*(lastP + this->qMatrix)*this->hMatrix.transpose() + this->rMatrix).inverse();
			ret = (Eigen::MatrixXd::Identity(this->stateDimensions, this->stateDimensions) - this->kalmanGain*this->hMatrix)*(lastP + this->qMatrix);

			return ret;
		}

		Eigen::MatrixXd getCov() {
			return this->pMatrix;
		}

		int getJointNr() {
			return this->nJoints;
		}

		std::vector<double> getState() {
			std::vector<double> ret(this->stateDimensions);
			int i;
			for (i = 0; i < this->stateDimensions; ++i) {
				ret[i] = this->state[i];
			}

			return ret;
		}

		bool setState(std::vector<double> &newState) {
			if (newState.size() != this->stateDimensions) {
				printf("[EKF]: stateDimensions disagree.\n");
				exit(0);
				return false;
			}
			for (int i = 0; i < this->stateDimensions; ++i) {
				this->state[i] = newState[i];
			}

			return true;
		}

		bool setPmatrix(Eigen::MatrixXd newP) {
			for (int i = 0; i < this->stateDimensions; ++i) {
				for (int j = 0; j < this->stateDimensions; ++j) {
					this->pMatrix(i,j) = newP(i,j);
				}
			}
			return true;
		}

		double getCoef(std::string coef) {
			if(coef == "r") return this->rMatrixCoef;
			if(coef == "q") return this->qMatrixCoef;
			if(coef == "p1") return this->pMatrixCoef1;
			if(coef == "p2") return this->pMatrixCoef2;
		}

		friend std::ostream& operator<<(std::ostream& f, const ExtendedKalmanFilterBodySchema& ekf) {
			f << "x:";
			for (int i = 0; i < ekf.stateDimensions; ++i) {
				f << ekf.state[i] << ",";
			}
			f << std::endl;
			f << "P:";
			for (int i = 0; i < ekf.stateDimensions; ++i) 
				for (int j = 0; j < ekf.stateDimensions; ++j)
					f << ekf.pMatrix(i,j) << ",";

			f << std::endl;
		}

	private:
		int stateDimensions = 12;
		int nJoints = 3;
		double rMatrixCoef, qMatrixCoef, pMatrixCoef1, pMatrixCoef2 = M_PI;
		Eigen::VectorXd state;
		Eigen::VectorXd joints;
		Eigen::MatrixXd pMatrix;
		Eigen::MatrixXd pMatrixDest;
		Eigen::MatrixXd qMatrix;
		Eigen::MatrixXd hMatrix;
		Eigen::MatrixXd rMatrix;
		Eigen::MatrixXd kalmanGain;
		GenericRightArm *arm = NULL;
		
		Eigen::VectorXd h(Eigen::VectorXd jointValues, bool predict = false) {
			int i;
			Eigen::Matrix4d T;
			Eigen::VectorXd ret(7);
			yarp::sig::Vector q, pose;
			copyVector(q, jointValues);
			std::vector<double> pos(3);
			std::vector<double> ori(4);
			T.setZero();
	        T(0,1)=-1.0;
	        T(1,2)=-1.0;
	        T(2,0)=1.0;
	        T(3,3)=1.0;
			//T.setIdentity();
			T = T*evalDHMatrix(      0.32,       0,     M_PI/2,    0);
			T = T*evalDHMatrix(       0,    -0.055,     M_PI/2,    -M_PI/2);
			T = T*evalDHMatrix(-0.233647,  -1.433,     M_PI/2,    -15*M_PI/180 - M_PI/2 );
			for (i = 0; i < this->nJoints; ++i) {
				T = T*evalDHMatrix(this->state[4*i], this->state[4*i + 1], this->state[4*i + 2], this->state[4*i + 3] + jointValues[i]*M_PI/180);
				//std::cout << this->state[4*i] << ", " << this->state[4*i+1] << ", " << this->state[4*i+2] << ", " << jointValues[i]*M_PI/180 + this->state[4*i+3] << std::endl;
			}
			//std::cout << jointValues << std::endl;
			inverseKinematics(pos, ori, T);

			if(predict){
				ret[0] = pos[0];
				ret[1] = pos[1];
				ret[2] = pos[2];
				ret[3] = ori[0];
				ret[4] = ori[1];
				ret[5] = ori[2];
				ret[6] = ori[3];

				return ret;
			}

			for (i = 0; i < 7; ++i)
			{
				q[i] *= M_PI/180;
			}

			iCub::iKin::iKinChain *chain;
			chain = this->arm->asChain();
			chain->setAng(q);
			pose = chain->Pose(9, true);
			copyVector(ret,pose);

			//ret[0] = pos[0];
			//ret[1] = pos[1];
			//ret[2] = pos[2];
			//ret[3] = ori[0];
			//ret[4] = ori[1];
			//ret[5] = ori[2];
			//ret[6] = ori[3];
			

			return ret;
		}
		Eigen::MatrixXd getJacobianH(Eigen::VectorXd jointValues) {
			int i, j;
			Eigen::MatrixXd jacobian(7, this->stateDimensions);
			Eigen::VectorXd z1(7), z2(7);
			double delta = 0.0001;
			for (i = 0; i < this->stateDimensions; i++) {
				this->state[i] += delta/2.0;
				z1 = h(jointValues, true);
				//std::cout << "z1: \n"<< z1 << std::endl;
				this->state[i] -= delta;
				z2 = h(jointValues, true);
				//std::cout << "z2: \n"<< z2 << std::endl;
				this->state[i] += delta/2.0;
				for (j = 0; j < 6; j++) {
					jacobian(j, i) = (z1[j] - z2[j])/delta; 
				}
				jacobian(6, i) = angularDifference(z1[6], z2[6])/delta; //z[6] é um ângulo
			}

			return jacobian;
		}
		bool readFileParameters() {
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
				std::cout << id << std::endl;
				if (id == "joints") {
					this->nJoints = std::stoi(value);
					this->stateDimensions = 4*this->nJoints;
					this->state.resize(this->stateDimensions);
				} else if(id == "r") {
					this->rMatrixCoef = std::stod(value);
				} else if(id == "q") {
					this->qMatrixCoef = std::stod(value);
				} else if(id == "p") {
					this->pMatrixCoef1 = std::stod(value);
				} else if(id == "x") {
					i = 0;
					while ((pos = value.find(comma)) != std::string::npos) {
					    token = value.substr(0, pos);
					    this->state[i] = std::stod(token) + randomValueUniDistr(-PARAMERROR*std::stod(token), PARAMERROR*std::stod(token));
					    value.erase(0, pos + comma.length());
						i++;
					}
					token = value.substr(0, pos);
					this->state[i] = std::stod(token) + randomValueUniDistr(-PARAMERROR*std::stod(token), PARAMERROR*std::stod(token));
				} 
			}

			myfile.close();
			return true;
		}
};	

#endif
