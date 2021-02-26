#include "EKF/ExtendedKalmanFilter.hpp"
#include "iCub/Robot.hpp"
#include "iCubLimb/iCubLimb.hpp"
#include <nlopt.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

typedef struct _dataOpt {
	ExtendedKalmanFilterBodySchema *ekf;
	std::vector<double> boundsOffset;
	bool costSensitive;
	double costCoef;
	double delta;
	boost::numeric::ublas::vector<double> prevTheta;
	BetaProcess *betaProcess;
	boost::numeric::ublas::vector<double> lowerBounds;
	boost::numeric::ublas::vector<double> upperBounds;
} dataOpt;

class CalibrationRoutine
{
public:
	CalibrationRoutine();
	~CalibrationRoutine();
	void start(bool test = false);
private:
	GenericRightArm *arm = NULL;
	dataOpt dtOpt;
	ExtendedKalmanFilterBodySchema ekf;
	iCubRobot robot;
	BetaProcess betaProcess;
	std::ofstream outputfile;
	std::ofstream trajectoryfile;
	std::ofstream outputTrajectoryFile;
	std::ofstream averagesfile;
	std::ofstream ekfdhfile;
	std::ofstream traceFile;
	bool activeLearning = true;
	bool goalDriven = true;
	bool addError = false;
	bool sampleMarker = true;
	bool constrained = true;
	bool predictError = true;
	double costCoef = 0;
	double goalCoef = 0;
	double delta = 0.1;
	int nSamples = 10;
	std::string criteria;
	int maxIter;
	int inputDim;
	int reps = 1;
	int currentRep = 0;

	std::vector<double> avPosError;
	std::vector<double> avRotError;
	std::vector<double> avDistanceTraveled;
	std::vector<double> goal;
	std::vector<double> lb; 
	std::vector<double> ub;
	boost::numeric::ublas::vector<double> lowerBounds;
	boost::numeric::ublas::vector<double> upperBounds;

	int startBinLen = 50;
	int binCnt = 0;
	int binStep = 200;
	int binLen;
	int iter = 0;
	std::vector<StatValue<double>> posErrorStats;
	std::vector<StatValue<double>> rotErrorStats;
	std::vector<StatValue<double>> travelDistanceStats; 
	std::vector<StatValue<double>> posErrorTravel;
	std::vector<StatValue<double>> rotErrorTravel;
	std::vector<StatValue<double>> optTime;
	std::vector<StatValue<double>> discardedSamples;
	boost::numeric::ublas::vector<double> nextJointValues, prevTheta;
	double totalTravel = 0;	
	int discSamp = 0;
	std::vector<double> nextJointsStd;

	std::vector<double> optimiseGreedy(std::vector<double> ideal);
	bool readFileParameters();
	void computeRandomSpacialError(double &pe, double &oe, double travel);
	void computePredictionError(std::vector<double> joints, double &posError, double &orientationError);
	std::vector<double> boost2stdVector(boost::numeric::ublas::vector<double> v);
	boost::numeric::ublas::vector<double> std2boostVector(std::vector<double> v);
	std::vector<double> generateRandomJointConfiguration();
	bool getSample(std::vector<double> q, std::vector<double> &z);
	void writeFiles(std::string name, std::string method, std::vector<StatValue<double>> stats, int size);
	void backupEverything();
	void readBackup();

	static double f(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
	static void constraint(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data);
};