#include "MotionController.hpp"

using namespace std;
using namespace ikfast;
using namespace vmath;

#define MathDebug true

MotionController::MotionController(vector<PredictiveJointController*> joints) {
	this->joints = joints;
	this->updatePeriod = (long)(Configuration::SamplePeriod*1000000.0); //seconds to microseconds
	
	motionPlanner = new MotionPlanner(joints);
	trajectoryPlanner = new TrajectoryPlanner();

	planLogTimeOffset = 0;
	updateCount = 0;
	
	for (auto it = joints.begin(); it != joints.end(); it++)
		timeSMA_map.insert(make_pair((*it)->getJointModel()->name,new SimpleMovingAverage(30)));
	
	
	timeSMA_map.insert(make_pair("All",new SimpleMovingAverage(30)));
	timeSMA_map.insert(make_pair("PoseDynamics",new SimpleMovingAverage(30)));
	
	ikLogStream << "Count,x,y,z,xt,yt,zt,xd,yd,zd" << endl;
	
	//AsyncLogger::getInstance().postLogTask("ik_tracking.csv", "Count,x,y,z,xt,yt,zt,xd,yd,zd\n");
	//AsyncLogger::getInstance().postLogTask("joint_tracking.csv", "Time,Roll0,xRoll0,Pitch0,xPitch0,Roll1,xRoll1,Pitch1,xPitch1,Pitch2,xPitch2,Roll2,xRoll2\n");
	state = State::Waiting;
	
	clog << "Time,Roll0,xRoll0,Pitch0,xPitch0,Roll1,xRoll1,Pitch1,xPitch1,Pitch2,xPitch2,Roll2,xRoll2" << endl;
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

		timespec start, step;
		TimeUtil::setNow(start);

		vector<double> jointAngles;
		for (auto it = joints.begin(); it != joints.end(); it++)
		{
			TimeUtil::setNow(step);

			auto joint = *it;

			joint->run();			
			jointHasActivePlan = jointHasActivePlan|| joint->isDynamicMode();
			jointAngles.push_back(joint->getCurrentAngle());

			timeSMA_map[joint->getJointModel()->name]->add(TimeUtil::timeSince(step)*1000.0);
		}
		timeSMA_map["All"]->add(TimeUtil::timeSince(start)*1000.0);
		
		cArmState.setJointAngles(jointAngles);
		
		updateControllerState(jointHasActivePlan);				
		updateChildState();
		executeControlTasks();
		
//		stringstream ss;
//		ss << TimeUtil::timeSince(planStartTime) + planLogTimeOffset << ",";
//		for (auto it = joints.begin(); it != joints.end(); it++)
//		{
//			ss << round((*it)->getCurrentAngle()/0.001)*0.001 << "," << round((*it)->getAngleSetpoint()/0.001)*0.001 << ",";
//		}
//		
//		clog << ss.str() << endl;
			
		double totalTime = TimeUtil::timeSince(start);
		long adjustedSleep = updatePeriod - (totalTime*1000000);
		if (adjustedSleep > 0 && adjustedSleep <= updatePeriod)
		{
			usleep(static_cast<unsigned int>(adjustedSleep));
		}		
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

void MotionController::updateChildState()
{	
	PoseDynamics::getInstance().setArmState(cArmState);
	motionPlanner->setArmState(cArmState);
	trajectoryPlanner->setArmState(cArmState);
}

void MotionController::updateControllerState(bool jointHasActivePlan)
{
	bool logIk = false;
	switch (state)
	{
	case State::FinitePlan:
		if (!jointHasActivePlan)
		{
			state = State::Waiting;
			stateWatcher(state);
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
//		stringstream ss;
//		ss << TimeUtil::timeSince(planStartTime) + planLogTimeOffset << ",";
//		for (auto it = joints.begin(); it != joints.end(); it++)
//		{
//			ss << (*it)->getCurrentAngle() << "," << (*it)->getAngleSetpoint() << ",";
//		}
//		
//		clog << ss.str() << endl;
		
		if (logIk)
		{
			
			
//			stringstream ss;
//			double r[9];
//			Vector3d t,t2,td;
//			ComputeFk(cArmState.JointAngles.data(),t,r);
//			ss << updateCount << "," << t.x << "," << t.y << "," << t.z;
//			ComputeFk(jointTargetAngles.data(),t2,r);
//			td = t - t2;
//			ss << "," << t2.x << "," << t2.y << "," << t2.z <<
//				"," << td.x << "," << td.y << "," << td.z << endl;
//			
//			ikLogStream << ss.str();
		}
	}
}

void MotionController::updateStreamingMotionPlans()
{
//	bool complete = false;
//	for (auto it=currentPlan.begin(); it != currentPlan.end(); it++)
//	{
//		if (TimeUtil::timeUntil((*it)->endTime) < 0)
//		{
//			complete = true;
//			break;
//		}
//	}
//
//	if (complete)
//	{
//		AsyncLogger::log("Plan complete, reading next goal");
//		currentPlan.clear();
//		IKGoal goal = controlProvider->nextGoal();
//		try
//		{
//			currentPlan = motionPlanner->buildPlan(goal);
//		}
//		catch (std::runtime_error & e)
//		{
//			stringstream ss;
//			ss << "Exception building plan for streaming goal." << e.what();
//			AsyncLogger::log(ss.str());
//			controlProvider->motionOutOfRange();
//			controlProvider->revokeControl();
//			currentPlan = motionPlanner->buildPlan(IKGoal::stopGoal());
//		}
//
//		std::for_each(currentPlan.begin(),currentPlan.end(),[](shared_ptr<MotionPlan> p){p->startNow();});
//		for (int i=0;i<6;i++) joints.at(i)->joinMotionPlan(currentPlan.at(i));			
//	}
}

void MotionController::requestDirectControl(IKGoal initialGoal, DirectControlProvider * controlProvider)
{
	//auto plan = motionPlanner->buildPlan(initialGoal);
	//this->controlProvider = controlProvider;	
	//executeMotionPlan(plan);		
	//state = State::StreamingPlan;
	//controlProvider->grantControl();
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
	cout << "Writing ik2 file " << endl;
	ofstream logIK("ik_tracking.csv",std::ofstream::out);
	logIK << ikLogStream.str();
	logIK.close();
	
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
	AsyncLogger::getInstance().log("Executing motion plan with duration " + to_string(newPlan.front()->getPlanDuration()));
	
	if (state != State::Waiting)
	{
		throw std::runtime_error("Must be in waiting state to execute plans");
	}


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
	
	state = State::FinitePlan;
	stateWatcher(state);

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
	ComputeFk(anglesRad,translation,r);
	rotation = Matrix3d::fromRowMajorArray(r);	
}

TrajectoryPlanner * MotionController::getTrajectoryPlanner()
{
	return trajectoryPlanner;
}

MotionPlanner * MotionController::getMotionPlanner()
{
	return motionPlanner;
}

void MotionController::setStateWatcher(std::function<void(MotionController::State)> watcher)
{
	this->stateWatcher = watcher;
}


















