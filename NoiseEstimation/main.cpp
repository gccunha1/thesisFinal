#include "MarkerNoiseEstimator.hpp"

int main(int argc, char** argv)
{
	bayesopt::Parameters params;
	int dim = 2;
	params.l_type = L_MCMC;
	MarkerNoiseEstimator optimizer(dim, params);
	 //Define bounds and prepare result.
	boost::numeric::ublas::vector<double> bestPoint(dim);
	boost::numeric::ublas::vector<double> lowerBound(dim);
	boost::numeric::ublas::vector<double> upperBound(dim);
	//Set the bounds. This is optional. Default is [0,1]
	//Only required because we are doing continuous optimization
	
	//optimizer.setBoundingBox(lowerBounds,upperBounds);
	
	//Collect the result in bestPoint
	optimizer.optimize(bestPoint);
}