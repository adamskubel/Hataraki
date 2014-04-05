#ifndef HATARAKI_BASICMOTION_MOTIONCONTROLLER_HPP_
#define HATARAKI_BASICMOTION_MOTIONCONTROLLER_HPP_

#include "IKFast.hpp"

#include <unistd.h>

#include <iomanip>
#include <cmath>
#include <vector>
#include <mutex>
#include <functional>
#include <queue>
#include <memory>
#include <algorithm>

#include "MathUtils.hpp"
#include "PredictiveJointController.hpp"
#include "MathUtils.hpp"
#include "AS5048.hpp"
#include "MotionPlanner.hpp"
#include "TimeUtil.hpp"
#include "TrajectoryPlanner.hpp"
#include "ALog.hpp"


class DirectControlProvider
{
public:
//	DirectControlProvider() { }
	
	virtual IKGoal nextGoal() = 0;
	virtual void motionComplete() = 0;
	virtual void motionOutOfRange() = 0;

	virtual void grantControl() = 0;
	virtual void revokeControl() = 0;

};


class MotionController {

public:
	enum class State
	{
		Waiting,
		FinitePlan,
		StreamingPlan,
		Shutdown
	};
	
public:
	MotionController(std::vector<PredictiveJointController*> joints);

private:
	State state;
	long updatePeriod;
	long updateCount;

	ArmState cArmState;

	std::function<void(State)> stateWatcher;

	std::vector<PredictiveJointController*> joints;

	std::queue<std::function<void()> > taskQueue;
	std::mutex taskQueueMutex;
	
	std::stringstream ikLogStream;
	std::map<std::string,SimpleMovingAverage*> timeSMA_map;
	
	timespec planStartTime;
	double planLogTimeOffset;

	std::vector<std::shared_ptr<MotionPlan> > currentPlan;
		
	MotionPlanner * motionPlanner;
	DirectControlProvider * controlProvider;
	TrajectoryPlanner * trajectoryPlanner;

	void executeControlTasks();
	void updateControllerState(bool jointHasActivePlan);
	void updateChildState();
	void updateStreamingMotionPlans();		
	bool confirmMotionPlan(std::vector<std::shared_ptr<MotionPlan> > & newPlan);
	
public:
	
	void updateController();
	void requestDirectControl(IKGoal initialGoal, DirectControlProvider * controlProvider);
	void printAverageTime();
	
	void executeMotionPlan(std::vector<std::shared_ptr<MotionPlan> > newPlan);
	void setJointVelocity(int jointIndex, double velocity, double runTime);
	void setJointPosition(int jointIndex, double angle);
				
	void postTask(std::function<void()> task);	
	
	void shutdown();	
	void zeroAllJoints();
	void prepareAllJoints();
	void enableAllJoints();
	
	MotionPlanner * getMotionPlanner();
	TrajectoryPlanner * getTrajectoryPlanner();
	void getTransform(std::vector<double> & angles, vmath::Vector3d & translation, vmath::Matrix3d & rotation);	
	PredictiveJointController * getJointByIndex(int jointIndex);

	void setStateWatcher(std::function<void(State)> watcher);

};

#endif