#ifndef HATARAKI_BASICMOTION_MOTIONCONTROLLER_HPP_
#define HATARAKI_BASICMOTION_MOTIONCONTROLLER_HPP_

#define IKFAST_NO_MAIN
#define IKFAST_HAS_LIBRARY
#define IKFAST_NAMESPACE ikfast2
#include "ikfast.h"

#include <unistd.h>

#include <iomanip>
#include <cmath>
#include <vector>
#include <mutex>
#include <functional>
#include <queue>
#include <memory>
#include <algorithm> 

#define VMATH_NAMESPACE vmath
#include "vmath.h"

#include "MathUtils.hpp"
#include "PredictiveJointController.hpp"
#include "MathUtils.hpp"
#include "AS5048.hpp"
#include "MotionPlanner.hpp"
#include "TimeUtil.hpp"



class MotionController {

private:
	std::vector<PredictiveJointController*> joints;
	int numSteps, planStepCount;
	std::vector<std::shared_ptr<MotionPlan> > currentPlan;
	std::queue<std::function<void()> > taskQueue;
	std::mutex taskQueueMutex;
	double samplePeriod;
	long updatePeriod;
	
	MotionPlanner * motionPlanner;
	
public:

	std::vector<double> getJointAnglesRadians();
	std::vector<double> getJointAnglesSteps();
	void getTransform(std::vector<double> & angles, vmath::Vector3d & translation, vmath::Matrix3d & rotation);
	
	void executeMotionPlan(std::vector<std::shared_ptr<MotionPlan> > newPlan);
	
	MotionController(std::vector<PredictiveJointController*> & joints, double samplePeriod, int planStepCount);

	PredictiveJointController * getJointByIndex(int jointIndex);
		
	void moveToPosition(vmath::Vector3d position, vmath::Matrix3d rotation, int pathDivisionCount, bool interactive);
	
	std::vector<std::shared_ptr<MotionPlan> >  planForPosition(vmath::Vector3d position, vmath::Matrix3d rotation, int pathDivisionCount);
	
	
	void setJointPosition(int jointIndex, double angle);
	bool confirmMotionPlan(std::vector<std::shared_ptr<MotionPlan> > & newPlan);

	void updateController();

	void postTask(std::function<void()> task);
	
	void shutdown();
	
	void zeroAllJoints();
	void prepareAllJoints();
	void enableAllJoints();

};

#endif