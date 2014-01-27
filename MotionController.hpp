#ifndef HATARAKI_BASICMOTION_MOTIONCONTROLLER_HPP_
#define HATARAKI_BASICMOTION_MOTIONCONTROLLER_HPP_

#define IKFAST_NO_MAIN
#define IKFAST_HAS_LIBRARY
#define IKFAST_NAMESPACE ikfast2

#define InvertPitchJoints true

#include <unistd.h>

#include <vector>
#include <mutex>
#include <functional>
#include <queue>
#include <memory>

#include "ikfast.h"
#include "MathUtils.hpp"
#include "PredictiveJointController.hpp"
#include "MathUtils.hpp"
#include "AS5048.hpp"


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
	std::vector<PredictiveJointController*> joints;
	int numSteps, state;
	std::vector<std::shared_ptr<JointMotionPlan> > currentPlan;
	std::queue<std::function<void()> > taskQueue;
	std::mutex taskQueueMutex;

	long updatePeriod;

	void getJointAngles(double * angles);

	bool getEasiestSolution(const double * currentAngles, const double * targetPosition, double * result);
	bool checkSolutionValid(const double * solution);
	double calculateMotionEffort(const double * solution0, const double * solution1);

	bool buildMotionSteps(double * position, std::vector<MotionStep*> & steps);

	void executeMotionPlan(std::vector<JointMotionPlan*> & newPlan);

	std::vector<std::shared_ptr<JointMotionPlan> > createMotionPlans(std::vector<MotionStep*> & steps);

public:
	static int PlanStepCount;

	MotionController(std::vector<PredictiveJointController*> & joints, long updatePeriod);
		
	void moveToPosition(double * position);

	void updateController();

	void postTask(std::function<void()> task);

};

#endif