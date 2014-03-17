#include "MotionController.hpp"

using namespace std;
using namespace ikfast2;
using namespace ikfast;
using namespace vmath;

#define MathDebug true

MotionController::MotionController(vector<PredictiveJointController*> & _joints, int _planStepCount) {
	this->joints = _joints;
	this->updatePeriod = (long)(Configuration::SamplePeriod*1000000.0); //seconds to microseconds
	this->planStepCount = _planStepCount;	
	this->motionPlanner = new MotionPlanner(joints);

	updateCount = 0;
	
	state = State::Waiting;
}

PredictiveJointController * MotionController::getJointByIndex(int jointIndex)
{
	if (jointIndex < 0 || jointIndex > joints.size())
		throw new std::runtime_error("Invalid joint index");

	return joints[jointIndex];
}

void MotionController::updateController(){

	if (state == State::Shutdown) return;

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

		//TimeUtil::setNow(step);
		if (updateCount % 100 == 0)
		{
			PoseDynamics::getInstance().setJointAngles(jointAngles);
			PoseDynamics::getInstance().update();
		}
		//TimeUtil::assertTime(step,"Pose dynamics update",10.0/1000.0);
		
		updateStreamingMotionPlans();

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
		
		updateCount++;
	}
	catch (std::runtime_error & e)
	{
		cout << "Exception thrown during control loop execution: " << e.what() << endl;
		cout << "Commencing emergency shutdown." << endl;
		shutdown();
		state = State::Shutdown;
	}
}


void MotionController::updateStreamingMotionPlans()
{
	if (state == State::StreamingPlan)
	{
		bool complete = false;
		for (auto it=currentPlan.begin(); it != currentPlan.end(); it++)
		{
			if (TimeUtil::timeUntil((*it)->endTime) < 0)
			{
				complete = true;
				break;
			}
		}
		
		if (complete)
		{
			currentPlan.clear();
			IKGoal goal = controlProvider->nextGoal();
			currentPlan = motionPlanner->buildPlan(goal);
			
			std::for_each(currentPlan.begin(),currentPlan.end(),[](shared_ptr<MotionPlan> p){p->startNow();});
			for (int i=0;i<6;i++) joints.at(i)->joinMotionPlan(currentPlan.at(i));			
		}
	}
}

void MotionController::requestDirectControl(IKGoal initialGoal, DirectControlProvider * controlProvider)
{
	auto plan = motionPlanner->buildPlan(initialGoal);
	this->controlProvider = controlProvider;	
	executeMotionPlan(plan);		
	state = State::StreamingPlan;
	controlProvider->grantControl();
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

void MotionController::setJointVelocity(int jointIndex, double targetVelocity, double runTime)
{
	if (jointIndex >= 0 && jointIndex < joints.size()){		
			
		auto joint = joints.at(jointIndex);
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


void MotionController::setJointPosition(int jointIndex, double targetAngle)
{
	if (jointIndex >= 0 && jointIndex < joints.size()){		
			
		auto joint = joints.at(jointIndex);
		auto plan = motionPlanner->buildOptimalMotionPlan(jointIndex,targetAngle);
		
		joint->validateMotionPlan(plan);		
		plan->startNow();		
		joint->executeMotionPlan(plan);
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
		setJointPosition(i,0);
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

void MotionController::executeMotionPlan(vector<shared_ptr<MotionPlan> > newPlan)
{
	if (state != State::Waiting)
		throw std::runtime_error("Must be in waiting state to execute plans");

	this->postTask([this,newPlan](){
		
		this->currentPlan.clear();
		this->currentPlan = newPlan;
		
		
		//Simultaneous start
		for (int i=0;i<6;i++)
		{
			currentPlan.at(i)->startNow();
		}
		for (int i=0;i<6;i++)
		{
			joints.at(i)->validateMotionPlan(currentPlan.at(i));
		}
		for (int i=0;i<6;i++)
		{
			joints.at(i)->executeMotionPlan(currentPlan.at(i));
		}
	});
}

bool MotionController::confirmMotionPlan(vector<shared_ptr<MotionPlan> > & newPlan)
{
	cout << "Angles    = ";
	for (auto it = newPlan.begin(); it != newPlan.end(); it++)
	{
		//cout << setprecision(2) << std::round(AS5048::stepsToDegrees((*it)->finalAngle)/0.01)*0.01 << "  ";
		cout << setprecision(2) << std::round(AS5048::stepsToDegrees((*it)->x(1000))/0.01)*0.01 << "  ";
	}
	cout << endl;
	cout << "Angles2    = ";
	for (auto it = newPlan.begin(); it != newPlan.end(); it++)
	{
		cout << setprecision(2) << std::round(AS5048::stepsToDegrees((*it)->finalAngle)/0.01)*0.01 << "  ";
	}
	cout << endl;
	cout << "Times = ";
	for (auto it = newPlan.begin(); it != newPlan.end(); it++)
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
	{
		newPlan.clear();
		return false;
	}
	else
	{
		return true;
	}
}

void MotionController::moveToPosition(Vector3d targetPosition, Matrix3d targetRotation, int pathDivisionCount, bool interactive)
{
	motionPlanner->setPathDivisions(pathDivisionCount);
	auto newPlan = motionPlanner->buildPlan(IKGoal(targetPosition,targetRotation,false));

	if (!interactive || confirmMotionPlan(newPlan))
	{
		executeMotionPlan(newPlan);
	}
}


void MotionController::postTask(std::function<void()> task)
{
	std::lock_guard<std::mutex> locks(taskQueueMutex);
	taskQueue.push(task);
}

void MotionController::getTransform(std::vector<double> & anglesDegrees, vmath::Vector3d & translation, vmath::Matrix3d & rotation)
{
	int i =0;
	double anglesRad[6];
	for (auto it = joints.begin(); it != joints.end(); it++)
	{
		double angleSteps = (*it)->getCurrentAngle();
		anglesRad[i++] = AS5048::stepsToRadians(angleSteps);
		anglesDegrees.push_back(AS5048::stepsToDegrees(angleSteps));
	}
	
	double r[9];
	ikfast2::ComputeFk(anglesRad,translation,r);
	rotation = Matrix3d::fromRowMajorArray(r);	
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

MotionPlanner * MotionController::getMotionPlanner()
{
	return motionPlanner;
}




















