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
	timeSMA_map.insert(make_pair("PoseDynamics",new SimpleMovingAverage(30)));
	
	ikLogStream << "Count,x,y,z,xt,yt,zt,xd,yd,zd" << endl;
	
	//AsyncLogger::getInstance().postLogTask("ik_tracking.csv", "Count,x,y,z,xt,yt,zt,xd,yd,zd\n");
	//AsyncLogger::getInstance().postLogTask("joint_tracking.csv", "Time,Roll0,xRoll0,Pitch0,xPitch0,Roll1,xRoll1,Pitch1,xPitch1,Pitch2,xPitch2,Roll2,xRoll2\n");
	state = State::Waiting;
	
	//clog << "Time,Roll0,xRoll0,Pitch0,xPitch0,Roll1,xRoll1,Pitch1,xPitch1,Pitch2,xPitch2,Roll2,xRoll2" << endl;
}

PredictiveJointController * MotionController::getJointByIndex(int jointIndex)
{
	if (jointIndex < 0 || jointIndex > joints.size())
		throw new std::runtime_error("Invalid joint index");

	return joints[jointIndex];
}


void MotionController::update(){

	if (state == State::Shutdown) return;
	
	try 
	{
		bool jointHasActivePlan = false;

		vector<double> jointAngles;
		for (auto it = joints.begin(); it != joints.end(); it++)
		{
			auto joint = *it;
			jointHasActivePlan = jointHasActivePlan|| joint->isDynamicMode();
			jointAngles.push_back(joint->getCurrentAngle());
		}
		
		cArmState.setJointAngles(jointAngles);
		
		updateControllerState(jointHasActivePlan);				
		updateChildState();
		
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
	default:
		break;
	}
				
	if (state == State::FinitePlan)
	{		
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


















