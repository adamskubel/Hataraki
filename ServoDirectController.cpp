#include "ServoDirectController.hpp"

using namespace std;


ServoDirectController::ServoDirectController(MotionPlanner * motionPlanner, vector<PredictiveJointController*> controlJoints)
{
	this->motionPlanner = motionPlanner;
	this->controlJoints = controlJoints;
}


void ServoDirectController::setJointVelocity(int jointIndex, double targetVelocity, double runTime)
{
	if (jointIndex >= 0 && jointIndex < controlJoints.size()){		
			
		auto joint = controlJoints.at(jointIndex);
		auto plan = std::shared_ptr<MotionPlan>(new MotionPlan());

		double v0 = joint->getCurrentVelocity();
		double vF = targetVelocity;
		double accelTime = abs((vF - v0)/joint->getMaxAcceleration());

		plan->motionIntervals.push_back(MotionInterval(v0,vF,accelTime));
		plan->motionIntervals.push_back(MotionInterval(vF,runTime));

		plan->startAngle = joint->getCurrentAngle();
		plan->finalAngle = plan->x(plan->getPlanDuration());

		joint->validateMotionPlan(plan);		
		plan->startNow();		
		joint->executeMotionPlan(plan);
	}
	else {
		throw std::runtime_error("Error! Joint index is not valid.");
	}
}


void ServoDirectController::setJointPosition(int jointIndex, double targetAngle)
{
	if (jointIndex >= 0 && jointIndex < controlJoints.size()){		
			
		auto joint = controlJoints.at(jointIndex);
		auto plan = motionPlanner->buildOptimalMotionPlan(jointIndex,targetAngle);
		
		plan->startNow();
		joint->validateMotionPlan(plan);		
		plan->startNow();		
		joint->executeMotionPlan(plan);
	}
	else {
		throw std::runtime_error("Error! Joint index is not valid.");
	}
}


void ServoDirectController::zeroAllJoints()
{
	for (int i=0;i<controlJoints.size();i++)
	{
		setJointPosition(i,0);
	}
}

void ServoDirectController::prepareAllJoints()
{
	for (auto it = controlJoints.begin(); it != controlJoints.end(); it++)
	{
		(*it)->prepare();
	}
}

void ServoDirectController::enableAllJoints()
{
	for (auto it = controlJoints.begin(); it != controlJoints.end(); it++)
	{
		(*it)->enable();
	}
}

void ServoDirectController::smoothStopJoint(PredictiveJointController * joint)
{
	double v0 = joint->getCurrentVelocity();
	double accelTime = abs((v0)/joint->getMaxAcceleration());
	
	auto plan = shared_ptr<MotionPlan>(new MotionPlan());
	plan->motionIntervals.push_back(MotionInterval(v0,0,accelTime));
	plan->motionIntervals.push_back(MotionInterval(0,0.5));
	plan->startAngle = joint->getCurrentAngle();
	plan->finalAngle = plan->x(plan->getPlanDuration());

	cout << "Stopping joint" << endl;
	joint->joinMotionPlan(plan);
}