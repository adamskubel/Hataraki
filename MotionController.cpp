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

	planLogTimeOffset = 0;
	updateCount = 0;
	
	for (auto it = joints.begin(); it != joints.end(); it++)
		timeSMA_map.insert(make_pair((*it)->getJointModel()->name,new SimpleMovingAverage(30)));
	
	
	timeSMA_map.insert(make_pair("All",new SimpleMovingAverage(30)));
	timeSMA_map.insert(make_pair("PoseDynamics",new SimpleMovingAverage(30)));
	
	AsyncLogger::getInstance().postLogTask("ik_tracking.csv", "Count,x,y,z,xt,yt,zt,xd,yd,zd\n");
	AsyncLogger::getInstance().postLogTask("joint_tracking.csv", "Time,Roll0,xRoll0,Pitch0,xPitch0,Roll1,xRoll1,Pitch1,xPitch1,Pitch2,xPitch2,Roll2,xRoll2\n");
	state = State::Waiting;
}

PredictiveJointController * MotionController::getJointByIndex(int jointIndex)
{
	if (jointIndex < 0 || jointIndex > joints.size())
		throw new std::runtime_error("Invalid joint index");

	return joints[jointIndex];
}

void MotionController::executeControlTasks()
{
	timespec step;
	TimeUtil::setNow(step);
	if (taskQueueMutex.try_lock()) {		
		while (!taskQueue.empty()) {
			try 
			{
				taskQueue.front()();
			}
			catch (std::runtime_error & e)
			{
				stringstream ss;
				ss << "Exception executing scheduled task: " << e.what();
				AsyncLogger::log(ss);
			}
			taskQueue.pop();
		}
		taskQueueMutex.unlock();
	}
	TimeUtil::assertTime(step,"Task execution");
}


void MotionController::updateController(){

	if (state == State::Shutdown) return;
	
	try 
	{
		bool jointHasActivePlan = false;
		bool logIk = false;

		timespec start, step;
		TimeUtil::setNow(start);

		std::vector<double> jointAngles, jointTargetAngles;

		for (auto it = joints.begin(); it != joints.end(); it++)
		{
			TimeUtil::setNow(step);
			auto joint = *it;

			joint->run();
			
			jointAngles.push_back(AS5048::stepsToRadians(joint->getCurrentAngle()));
			jointTargetAngles.push_back(AS5048::stepsToRadians(joint->getAngleSetpoint()));

			jointHasActivePlan = jointHasActivePlan|| joint->isDynamicMode();

			timeSMA_map[joint->getJointModel()->name]->add(TimeUtil::timeSince(step)*1000.0);
		}
		timeSMA_map["All"]->add(TimeUtil::timeSince(start)*1000.0);
		
//		if (updateCount % 10 == 0)
		{
			TimeUtil::setNow(step);
			PoseDynamics::getInstance().setJointAngles(jointAngles);
			PoseDynamics::getInstance().update();
			timeSMA_map["PoseDynamics"]->add(TimeUtil::timeSince(step)*1000.0);
		}
		
		//Update state
		switch (state)
		{
		case State::FinitePlan:
			if (!jointHasActivePlan)
			{
				state = State::Waiting;
				//On transition, mark the time
				planLogTimeOffset += TimeUtil::timeSince(planStartTime) + 0.1;
			}
			break;
		case State::StreamingPlan:
			updateStreamingMotionPlans();
			break;
		default:
			break;
		}
				
		if (state == State::FinitePlan)
		{
			if (logIk)
			{
				stringstream ss;
				double r[9];
				Vector3d t,t2,td;
				ikfast2::ComputeFk(jointAngles.data(),t,r);
				ss << updateCount << "," << t.x << "," << t.y << "," << t.z;
				ikfast2::ComputeFk(jointTargetAngles.data(),t2,r);
				td = t - t2;
				ss << "," << t2.x << "," << t2.y << "," << t2.z <<
					"," << td.x << "," << td.y << "," << td.z << endl;
			
				AsyncLogger::getInstance().postLogTask("ik_tracking.csv", ss.str());
			}

			//stringstream jointStream;
			//jointStream << TimeUtil::timeSince(planStartTime) + planLogTimeOffset << ",";
			//for (auto it = joints.begin(); it != joints.end(); it++)
			//{
			//	auto joint = *it;
			//	jointStream << joint->getCurrentAngle() << "," << joint->getAngleSetpoint << "," << joint->getCurrentVelocity() << ","
			//}
			//for (int i=0;i<jointAngles.size();i++)
			//{
			//	jointStream << 
			//	AS5048::radiansToSteps(jointAngles[i]) << "," << AS5048::radiansToSteps(jointTargetAngles[i]) << ",";
			//}
			//jointStream << endl;
			//AsyncLogger::getInstance().postLogTask("joint_tracking.csv", jointStream.str());
		}
		
		
		executeControlTasks();
		
		double totalTime = TimeUtil::timeSince(start);
		long adjustedSleep = updatePeriod - (totalTime*1000000);
		if (adjustedSleep > 0 && adjustedSleep <= updatePeriod)
			usleep(static_cast<unsigned int>(adjustedSleep));
		
		updateCount++;
	}
	catch (std::runtime_error & e)
	{
		stringstream ss;
		ss << "Exception thrown during control loop execution: " << e.what();
		AsyncLogger::log(ss);
		shutdown();
		state = State::Shutdown;
	}
}


void MotionController::updateStreamingMotionPlans()
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
		AsyncLogger::log("Plan complete, reading next goal");
		currentPlan.clear();
		IKGoal goal = controlProvider->nextGoal();
		try
		{
			currentPlan = motionPlanner->buildPlan(goal);
		}
		catch (std::runtime_error & e)
		{
			stringstream ss;
			ss << "Exception building plan for streaming goal." << e.what();
			AsyncLogger::log(ss.str());
			controlProvider->motionOutOfRange();
			controlProvider->revokeControl();
			currentPlan = motionPlanner->buildPlan(IKGoal::stopGoal());
		}

		std::for_each(currentPlan.begin(),currentPlan.end(),[](shared_ptr<MotionPlan> p){p->startNow();});
		for (int i=0;i<6;i++) joints.at(i)->joinMotionPlan(currentPlan.at(i));			
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
		
		plan->startNow();
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
	
	for (auto it = joints.begin(); it != joints.end(); it++)
		(*it)->writeHistoryToLog();
}

void MotionController::printAverageTime()
{
	this->postTask([this](){
		for (auto it = timeSMA_map.begin(); it != timeSMA_map.end(); it++)
		{
			cout << it->first << " = " << it->second->avg() << endl;
		}
	});
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
	AsyncLogger::getInstance().postLogTask("motion.log", "Executing motion plan\n");
	
	if (state != State::Waiting)
		throw std::runtime_error("Must be in waiting state to execute plans");

	this->postTask([this,newPlan](){
		
		if (state != State::Waiting)
			throw std::runtime_error("ASYNC: Must be in waiting state to execute plans");			

		state = State::FinitePlan;

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

		//Simultaneous start
		for (int i=0;i<6;i++)
		{
			currentPlan.at(i)->startNow();
		}
		planStartTime = TimeUtil::getNow();

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




















