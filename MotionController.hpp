#ifndef HATARAKI_BASICMOTION_MOTIONCONTROLLER_HPP_
#define HATARAKI_BASICMOTION_MOTIONCONTROLLER_HPP_

#define IKFAST_NO_MAIN
#define IKFAST_HAS_LIBRARY
#define IKFAST_NAMESPACE ikfast2

#define InvertPitchJoints false

#include <unistd.h>

#include <vector>
#include <mutex>
#include <functional>
#include <queue>
#include <memory>
#include <algorithm> 

#define VMATH_NAMESPACE vmath
#include "vmath.h"

#include "ikfast.h"
#include "MathUtils.hpp"
#include "PredictiveJointController.hpp"
#include "MathUtils.hpp"
#include "AS5048.hpp"
#include "PathPlanner.hpp"
#include "MotionPlanner.hpp"

namespace MotionControllerState {

	static int Waiting = 0;
	static int Stepping = 1;
}


class MotionController {

private:
	std::vector<PredictiveJointController*> joints;
	int numSteps, state, planStepCount;
	std::vector<std::shared_ptr<MotionPlan> > currentPlan;
	std::queue<std::function<void()> > taskQueue;
	std::mutex taskQueueMutex;
	double samplePeriod;
	long updatePeriod;
	
	MotionPlanner * motionPlanner;
	
public:

	std::vector<double> getJointAngles();
	void executeMotionPlan(std::vector<MotionPlan*> & newPlan);

public:

	MotionController(std::vector<PredictiveJointController*> & joints, double samplePeriod, int planStepCount);

	PredictiveJointController * getJointByIndex(int jointIndex);
		
	void moveToPosition(vmath::Vector3d position, vmath::Matrix3d rotation, int pathDivisionCount, bool interactive);	
	void setJointPosition(int jointIndex, double angle, double velocity, double accel);
	

	void updateController();

	void postTask(std::function<void()> task);
	
	void shutdown();
	
	void zeroAllJoints();
	void prepareAllJoints();
	void enableAllJoints();

};

#endif