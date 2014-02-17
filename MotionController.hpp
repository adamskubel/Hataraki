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


namespace MotionControllerState {

	static int Waiting = 0;
	static int Stepping = 1;
}


struct PlanSolution {
	
	double t0,t1,t2,t3;
	double travelVelocity;
	double accel0,accel1;
	bool valid;
};

class MotionController {
	
	struct MotionStep {

		double targetJointAngles[6];
		double jointAngleDelta[6];
		double targetPosition[3];

	};

private:
	std::vector<PredictiveJointController*> joints;
	int numSteps, state, planStepCount;
	std::vector<std::shared_ptr<MotionPlan> > currentPlan;
	std::queue<std::function<void()> > taskQueue;
	std::mutex taskQueueMutex;
	double samplePeriod;
	long updatePeriod;

	void getJointAngles(double * angles);

	bool getEasiestSolution(const double * currentAngles, vmath::Vector3d targetPosition, vmath::Matrix3d targetRotation, double * result);
	bool checkSolutionValid(const double * solution);
	double calculateMotionEffort(const double * solution0, const double * solution1);

	bool buildMotionSteps(double * position, vmath::Matrix3d targetRotation, std::vector<MotionStep*> & steps);

	void executeMotionPlan(std::vector<MotionPlan*> & newPlan);

	std::vector<std::shared_ptr<MotionPlan> > createMotionPlans(std::vector<MotionStep*> & steps, double maxAccel, double maxDeccel);

public:
	

	MotionController(std::vector<PredictiveJointController*> & joints, double samplePeriod, int planStepCount);

	PredictiveJointController * getJointByIndex(int jointIndex);
		
	void moveToPosition(vmath::Vector3d position, vmath::Matrix3d rotation, double accel, double deccel, bool interactive);	
	void setJointPosition(int jointIndex, double angle, double velocity, double accel);
	
	static std::shared_ptr<MotionPlan> buildMotionPlan(const double startPosition,const double endPosition, const double totalTime, const double approachVelocity, const double maxAccel);

	static double optimalSpeed(const double a0, const double d3, const double dTotal, const double v0, const double v2, const double maxSpeed, double & speed);
	static void calculatePlan(double absAccel, double d3, double tTotal, double dTotal, double v0, double v2, PlanSolution & result);

	static double optimalSpeed2Part(const double a0, const double dTotal, const double v0, const double maxSpeed, double & speed);
	static void calculatePlan2Part(double absAccel, double tTotal, double dTotal, double v0, PlanSolution & result);

	void updateController();

	void postTask(std::function<void()> task);
	
	void shutdown();
	
	void zeroAllJoints();
	void prepareAllJoints();
	void enableAllJoints();

};

#endif