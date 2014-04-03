#include "TrajectoryController.hpp"


using namespace std;

TrajectoryController::TrajectoryController(MotionController * motionController)
{
	this->state = State::Waiting;
	this->motionController = motionController;
	motionController->setStateWatcher([this](MotionController::State mcState){ 
		if (mcState == MotionController::State::Waiting)
		{

		}
	});
}

void TrajectoryController::nextGoal()
{
	if (state != State::ExecutingGoalSequence) return;

	auto trajectory = motionController->getTrajectoryPlanner()->buildTrajectory(sequentialGoals.front());
	auto motionPlan = motionController->getMotionPlanner()->buildPlan(trajectory);

	sequentialGoals.pop_front();
	
	if (sequentialGoals.empty())
	{
		state = State::Waiting;
	}
	
	motionController->executeMotionPlan(motionPlan);
}

void TrajectoryController::executeSequentialGoals(vector<IKGoal> goals)
{
	if (state != State::Waiting) throw std::runtime_error("TrajectoryController::executeSequentialGoals - Must be in waiting state to continue");

	sequentialGoals.clear();
	std::copy(goals.begin(), goals.end(), back_inserter(sequentialGoals));
		
	motionController->setStateWatcher([this](MotionController::State mcState){ 
		if (mcState == MotionController::State::Waiting)
		{
			this->nextGoal();
		}
	});

	state = State::ExecutingGoalSequence;
	nextGoal();
}