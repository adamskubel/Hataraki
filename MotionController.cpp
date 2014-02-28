#include "MotionController.hpp"

#include "TimeUtil.hpp"

#include <iomanip>
#include <cmath>

using namespace std;
using namespace ikfast2;
using namespace ikfast;
using namespace vmath;

#define MathDebug true

MotionController::MotionController(vector<PredictiveJointController*> & _joints, double _samplePeriod, int _planStepCount) {
	this->joints = _joints;
	this->updatePeriod = (long)(_samplePeriod*1000000.0); //seconds to microseconds
	this->planStepCount = _planStepCount;
	this->samplePeriod = _samplePeriod;
	
	this->motionPlanner = new MotionPlanner(joints);
}

PredictiveJointController * MotionController::getJointByIndex(int jointIndex)
{
	if (jointIndex < 0 || jointIndex > joints.size())
		throw new std::runtime_error("Invalid joint index");

	return joints[jointIndex];
}

void MotionController::updateController(){

	try 
	{
		timespec start, step;
		TimeUtil::setNow(start);

		std::vector<double> jointAngles;

		for (auto it = joints.begin(); it != joints.end(); it++)
		{
			TimeUtil::setNow(step);
			(*it)->run();
			jointAngles.push_back(AS5048::stepsToRadians((*it)->getCurrentAngle()));
			TimeUtil::assertTime(step,"Joint " + (*it)->getJointModel()->name + " update");
		}

		TimeUtil::setNow(step);
		PoseDynamics::getInstance().setJointAngles(jointAngles);
		PoseDynamics::getInstance().update();
		TimeUtil::assertTime(step,"Pose dynamics update");
				
		TimeUtil::setNow(step);
		if (taskQueueMutex.try_lock()) {		
			while (!taskQueue.empty()) {
				try 
				{
					taskQueue.front()();
				}
				catch (std::runtime_error & e)
				{
					cout << "Exception executing scheduled task: " << e.what() << endl;
				}
				taskQueue.pop();
			}
			taskQueueMutex.unlock();
		}
		TimeUtil::assertTime(step,"Task execution");
		
		double totalTime = TimeUtil::timeSince(start);
		long adjustedSleep = updatePeriod - (totalTime*1000000);
		if (adjustedSleep > 0 && adjustedSleep <= updatePeriod)
			usleep(static_cast<unsigned int>(adjustedSleep));
	}
	catch (std::runtime_error & e)
	{
		cout << "Exception thrown during control loop execution: " << e.what() << endl;
		cout << "Commencing emergency shutdown." << endl;
		shutdown();
	}
}

double getAccelDistFromTime(double initialSpeed, double accel, double time)
{
	return time*initialSpeed + (std::pow(time,2) * accel)/2.0;
}

double getAccelDistFromSpeed(double initialSpeed, double endSpeed, double accel)
{
	double time = (endSpeed - initialSpeed)/accel;

	return time*initialSpeed + (std::pow(time,2) * accel)/2.0;
}



void MotionController::setJointPosition(int jointIndex, double targetAngle, double travelTime, double accel)
{
	if (jointIndex >= 0 && jointIndex < joints.size()){		
			
		auto pjc = joints.at(jointIndex);
		auto plan = motionPlanner->buildMotionPlan(pjc->getCurrentAngle(),targetAngle,travelTime,pjc->getJointModel()->servoModel.controllerConfig.approachVelocity, accel);
		
		pjc->validateMotionPlan(plan);
		
		plan->startNow();		
		cout << "Time until complete=" << TimeUtil::timeUntil(plan->endTime) << endl;
		plan->startNow();
		pjc->executeMotionPlan(plan);
	}
	else {
		throw std::runtime_error("Error! Joint index is not valid.");
	}
}

void MotionController::shutdown()
{
	for (auto it = joints.begin(); it != joints.end(); it++)
	{
		try
		{
			(*it)->emergencyHalt("Shutdown commanded.");
		}
		catch (std::runtime_error & e)
		{
			cout << "Error while shutting down joint: " << e.what() << endl;
		}
	}
}

void MotionController::zeroAllJoints()
{
	for (int i=0;i<joints.size();i++)
	{
		setJointPosition(i,0,3,AS5048::degreesToSteps(80));
	}
}

void MotionController::prepareAllJoints()
{
	for (auto it = joints.begin(); it != joints.end(); it++)
	{
		(*it)->prepare();
	}
}

void MotionController::enableAllJoints()
{
	for (auto it = joints.begin(); it != joints.end(); it++)
	{
		(*it)->enable();
	}
}


void MotionController::moveToPosition(Vector3d targetPosition, Matrix3d targetRotation, int pathDivisionCount, bool interactive)
{
	motionPlanner->setPathDivisions(pathDivisionCount);
	vector<Step> steps = motionPlanner->buildMotionSteps( getJointAnglesRadians(), targetPosition, targetRotation);
	
	bool executePlan = false;

	currentPlan.clear();

	if (pathDivisionCount > 1)
		currentPlan = motionPlanner->buildPlan(steps);	
	else
		currentPlan = motionPlanner->createClosedSolutionMotionPlanFromSteps(steps);


	if (interactive)
	{
		cout << "Angles    = ";
		for (auto it = currentPlan.begin(); it != currentPlan.end(); it++)
		{
			cout << setprecision(2) << std::round(AS5048::stepsToDegrees((*it)->finalAngle)/0.01)*0.01 << "  ";
		}
		cout << endl;
		cout << "Times = ";
		for (auto it = currentPlan.begin(); it != currentPlan.end(); it++)
		{
			double time = (*it)->getPlanDuration();
			cout << std::round(time/0.01)*0.01 << "  ";
		}
		cout << endl;
		cout << std::fixed;
		cout << "Commence motion? [y]es/[a]bort :" << endl;
		string strIn;
		getline(cin,strIn);

		if (strIn.compare("y") != 0)
			currentPlan.clear();
		else
			executePlan = true;
	}
	else
		executePlan = true;

	if (executePlan)
	{
		this->postTask([this](){

			for (int i=0;i<6;i++)
			{
				joints.at(i)->validateMotionPlan(currentPlan.at(i));
			}

			//Simultaneous start
			for (int i=0;i<6;i++)
			{					
				currentPlan.at(i)->startNow();
			}

			for (int i=0;i<6;i++)
			{
				joints.at(i)->executeMotionPlan(currentPlan.at(i));
			}
		});
	}
}


void MotionController::postTask(std::function<void()> task)
{
	std::lock_guard<std::mutex> locks(taskQueueMutex);
	taskQueue.push(task);
}

vector<double> MotionController::getJointAnglesRadians()
{
	vector<double> angles;
	for (auto it = joints.begin(); it != joints.end(); it++)
	{					
		angles.push_back(AS5048::stepsToRadians((*it)->getCurrentAngle()));
	}
	return angles;
}

vector<double> MotionController::getJointAnglesSteps()
{
	vector<double> angles;
	for (auto it = joints.begin(); it != joints.end(); it++)
	{					
		angles.push_back((*it)->getCurrentAngle());
	}
	return angles;
}
