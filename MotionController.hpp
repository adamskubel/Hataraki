#ifndef HATARAKI_BASICMOTION_MOTIONCONTROLLER_HPP_
#define HATARAKI_BASICMOTION_MOTIONCONTROLLER_HPP_

#define IKFAST_NO_MAIN
#define IKFAST_HAS_LIBRARY
#define IKFAST_NAMESPACE ikfast2

#define InvertPitchJoints true

#include <unistd.h>

#include <vector>

#include "ikfast.h"

#include "JointLoop.hpp"
#include "MathUtils.hpp"
#include "AS5048.hpp"
#include <mutex>
#include <functional>
#include <queue>


namespace MotionControllerState {

	static int Waiting = 0;
	static int Stepping = 1;
}

class MotionController {
	
	struct MotionStep {

		double targetJointAngles[6];
		double maxJointVelocities[6];
		double targetPosition[3];

	};

private:
	std::vector<JointLoop*> joints;
	int numSteps, state, currentStep;
	std::vector<MotionStep*> currentPlan;
	std::queue<std::function<void()> > taskQueue;
	std::mutex taskQueueMutex;

	long updatePeriod;

	void getJointAngles(double * angles);

	bool getEasiestSolution(const double * currentAngles, const double * targetPosition, double * result);
	bool checkSolutionValid(const double * solution);
	double calculateMotionEffort(const double * solution0, const double * solution1);

	bool buildMotionSteps(double * position, std::vector<MotionStep*> & steps);

	void commandMotionStep(MotionStep * step);

public:
	static int PlanStepCount;

	MotionController(std::vector<JointLoop*> & joints, long updatePeriod);
		
	void moveToPosition(double * position);

	void updateController();

	void postTask(std::function<void()> task);

};

#endif