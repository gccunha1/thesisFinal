#include "MarkerNoiseEstimator.hpp"
#include "../utils/utils.hpp"
#include <Eigen/Dense>


MarkerNoiseEstimator::MarkerNoiseEstimator(int ndim, bayesopt::Parameters param) : ContinuousModel(ndim, param) {
	this->ndim = ndim;
	this->param = param;
};

MarkerNoiseEstimator::~MarkerNoiseEstimator() {

}


double MarkerNoiseEstimator::evaluateSample( const boost::numeric::ublas::vector<double> &query ) {
	
};
bool MarkerNoiseEstimator::checkReachability( const boost::numeric::ublas::vector<double> &query ) {
	//Colocar restrições
	/*std::cout << "Bound Checking." << std::endl;
	std::vector<double> qry(this->ndim);

	int i;
	for (i = 0; i < query.size(); i++) {
		qry[i] = query[i];
	}

	bool ok = this->robot.move(qry, "right_arm"); 
	std::cout << ok << std::endl;
	return ok;*/

	return true;
};


