#include "WheelMotionController.hpp"

using namespace std;

WheelMotionController::WheelMotionController(vector<PredictiveJointController*> wheels,DriveModel _driveModel, ServoDirectController * directController) :
driveModel(_driveModel)
{
	this->directController = directController;
	this->wheels = wheels;
	setState(State::Waiting);
}


void WheelMotionController::translateBy(double distance)
{
	if (state != State::Waiting)
		throw std::runtime_error("Must be in waiting state to invoke");
	
	double totalRotation = driveModel.metersToSteps(distance);
	
	//cout << "Meters per step = " << driveModel.metersPerStep << endl;
	//cout << "Offsetting by " << totalRotation << " steps" << endl;
	
	for (int i=0;i<wheels.size();i++)
	{
		double finalPosition;
		if (driveModel.wheels[i].reverse)
			finalPosition = wheels[i]->getCurrentAngle() - totalRotation;
		else
			finalPosition = wheels[i]->getCurrentAngle() + totalRotation;
		
		directController->setJointPosition(i, finalPosition);
	}
	
	cout << "Translation in progress" << endl;
	
	setState(State::Active);
}

void WheelMotionController::rotateBy(double angleDegrees)
{
	if (state != State::Waiting)
	{
		throw std::runtime_error("Must be in waiting state to invoke");
	}
	
	vector<double> wheelTranslations(2);

	double leftCircle = (driveModel.wheels[0].distanceFromCenter)*(angleDegrees/360.0)*(MathUtil::PI*2.0);
	double rightCircle = -(driveModel.wheels[1].distanceFromCenter)*(angleDegrees/360.0)*(MathUtil::PI*2.0);

	wheelTranslations[0] = driveModel.metersToSteps(leftCircle);
	wheelTranslations[1] = driveModel.metersToSteps(rightCircle);

	for (int i=0;i<wheels.size();i++)
	{
		double finalPosition;
		if (driveModel.wheels[i].reverse)
			finalPosition = wheels[i]->getCurrentAngle() - wheelTranslations[i];
		else
			finalPosition = wheels[i]->getCurrentAngle() + wheelTranslations[i];
		
		directController->setJointPosition(i, finalPosition);
	}
	
	cout << "Rotation in progress" << endl;
	
	setState(State::Active);
}

void WheelMotionController::addStateWatcher(function<void(State)> watcher)
{
	watchers.push_back(watcher);
}

void WheelMotionController::update()
{
	if (state == State::Active)
	{
		bool jointActive = false;
		for (auto it = wheels.begin(); it!= wheels.end(); it++)
		{
			jointActive = jointActive || (*it)->isDynamicMode();
		}
		if (!jointActive)
		{
			setState(State::Waiting);
		}
	}
}

void WheelMotionController::shutdown()
{
	
}

void WheelMotionController::setState(State newState)
{
	if (state != newState)
	{
		state = newState;
		for (auto it=watchers.begin(); it != watchers.end(); it++)
		{
			(*it)(newState);
		}
	}
}

void WheelMotionController::stopMotion()
{
	for (auto it=wheels.begin(); it != wheels.end(); it++)
	{
		(*it)->cancelMotionPlan(true);
		//		directController->smoothStopJoint(*it);
	}
}














