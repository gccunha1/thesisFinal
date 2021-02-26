#ifndef  MY_PROBLEM
#define  MY_PROBLEM


#include <bayesopt/bayesopt.hpp>
#include <iostream>
#include <fstream>



class MarkerNoiseEstimator: public bayesopt::ContinuousModel
{
 	private:
 		int ndim;
 		bayesopt::Parameters param;
 	public:
  		MarkerNoiseEstimator(int ndim, bayesopt::Parameters param);
  		~MarkerNoiseEstimator();
  		bool readFileParameters();
  		double evaluateSample( const boost::numeric::ublas::vector<double> &query );
  		bool checkReachability( const boost::numeric::ublas::vector<double> &query );
};

class MarkerController {
public:
	
private:

};

#endif