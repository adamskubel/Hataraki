#ifndef HATARAKI_BASICMOTION_MOTIONPLANNING_TRAJECTORY_CONTROLLER_HPP_
#define HATARAKI_BASICMOTION_MOTIONPLANNING_TRAJECTORY_CONTROLLER_HPP_

#include <vector>
#include <list>
#include <iterator>

#include "MotionController.hpp"
#include "TrajectoryPlanner.hpp"
#include "MotionPlan.hpp"
#include "MotionPlanner.hpp"


class TrajectoryController {

	enum class State {

		Waiting,
		ExecutingGoalSequence
	};

private:
	MotionController * motionController;
	State state;
	std::list<IKGoal> sequentialGoals;

	void nextGoal();

public:
	TrajectoryController(MotionController * motionController);

	void executeSequentialGoals(std::vector<IKGoal> goals);


};

#endif