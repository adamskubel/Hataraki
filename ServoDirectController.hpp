#ifndef HATARAKI_BASICMOTION_SERVO_DEBUG_CONTROLLER_HPP_
#define HATARAKI_BASICMOTION_SERVO_DEBUG_CONTROLLER_HPP_

#include <vector>

#include "PredictiveJointController.hpp"
#include "MotionPlanner.hpp"

class ServoDirectController {

private:
	std::vector<PredictiveJointController*> controlJoints;
	MotionPlanner * motionPlanner;

public:
	ServoDirectController(MotionPlanner * motionPlanner, std::vector<PredictiveJointController*> controlJoints);

	void setJointVelocity(int jointId, double targetVelocity, double runTime);
	void setJointPosition(int jointIndex, double targetAngle);
	void zeroAllJoints();
	void prepareAllJoints();
	void enableAllJoints();

	void smoothStopJoint(PredictiveJointController * joint);


};


#endif