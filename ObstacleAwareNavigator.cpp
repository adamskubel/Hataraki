#include "ObstacleAwareNavigator.hpp"

using namespace std;

ObstacleAwareNavigator::ObstacleAwareNavigator(vector<AntennaDeflectionSensor*> antennaSensors, vector<PredictiveJointController*> wheelControllers, DriveModel driveModel, ServoDirectController * directController) :
	WheelMotionController(wheelControllers,driveModel,directController)
{
	this->antennaSensors = antennaSensors;
	colliding = false;

	for (auto it=antennaSensors.begin(); it!=antennaSensors.end(); it++)
	{
		(*it)->addDeflectionEventWatcher([this](){this->deflectionEvent();});
	}
	
	addStateWatcher([this](State s){
		
		if (s == State::Active)
		{
			for (auto it=this->antennaSensors.begin(); it!=this->antennaSensors.end(); it++)
			{
				(*it)->resetSensor();
				cout << "Resetting sensor" << endl;
			}
		}		
	});
}


void ObstacleAwareNavigator::deflectionEvent()
{
	if (state == State::Active)
	{
		colliding = true;

		wheels[0]->getCurrentAngle();

		//cout << "Obstacle detected, stopping" << endl;
		//this->stopMotion();
	}
	else if (state == State::Waiting)
		cout << "State is 'Waiting', but deflection event received" << endl;
}


void ObstacleAwareNavigator::update()
{
	WheelMotionController::update();

	if (colliding)
	{

	}
}