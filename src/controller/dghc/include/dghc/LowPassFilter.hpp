
#ifndef _LowPassFilter_hpp_
#define _LowPassFilter_hpp_

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Sparse>
class LowPassFilter{
public:
	//constructors
	LowPassFilter();
	LowPassFilter(float iCutOffFrequency, float iDeltaTime);
	//functions
	Eigen::VectorXd update(Eigen::VectorXd input);
	Eigen::VectorXd update(Eigen::VectorXd input, float deltaTime, float cutoffFrequency);
	//get and configure funtions
	Eigen::VectorXd getOutput() const{return output;}
	void reconfigureFilter(float deltaTime, float cutoffFrequency,int dof);
private:
	Eigen::VectorXd output;
	float ePow;
	int dof;
};

#endif //_LowPassFilter_hpp_
