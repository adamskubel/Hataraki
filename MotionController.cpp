#include "MotionController.hpp"

#include "TimeUtil.hpp"

#include <iomanip>
#include <cmath>

using namespace std;
using namespace ikfast2;
using namespace ikfast;
using namespace vmath;

MotionController::MotionController(vector<PredictiveJointController*> & _joints, double _samplePeriod, int _planStepCount) {
	this->joints = _joints;
	this->updatePeriod = (long)(_samplePeriod*1000000.0); //seconds to microseconds
	this->planStepCount = _planStepCount;
	state = MotionControllerState::Waiting;
	this->samplePeriod = _samplePeriod;
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
		
		double totalTime = MathUtil::timeSince(start);
		long adjustedSleep = updatePeriod - (totalTime*1000000);
		if (adjustedSleep > 0 && adjustedSleep <= updatePeriod)
			usleep(adjustedSleep);
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


double MotionController::optimalSpeed(const double accel, const double d3, const double dTotal, const double v0, const double v2, const double maxSpeed, double & v1)
{
	double a0 = std::abs(accel);
	v1 = sqrt(2.0)*sqrt(a0*d3*-2.0+a0*dTotal*2.0+v0*v0+v2*v2)*(1.0/2.0);

	if (std::isnan(v1))
	{
		a0 = -a0;
		v1 = -sqrt(2.0)*sqrt(a0*d3*-2.0+a0*dTotal*2.0+v0*v0+v2*v2)*(1.0/2.0);
	}

	if (std::abs(v1) > std::abs(maxSpeed))
		v1 = std::abs(maxSpeed)*MathUtils::sgn<double>(v1);

	return ((v0*v0)*v2*(1.0/2.0)-v1*(v2*v2)+(v1*v1)*v2+(v2*v2*v2)*(1.0/2.0)+a0*d3*v1-a0*d3*v2+a0*dTotal*v2-v0*v1*v2)/(a0*v1*v2);
}

double MotionController::optimalSpeed2Part(const double accel, const double dTotal, const double v0, const double maxSpeed, double & v1)
{
	double a0 = std::abs(accel);
	v1 = sqrt(a0*dTotal*2.0+v0*v0);

	if (std::isnan(v1))
	{
		a0 = -a0;
		v1 = -sqrt(a0*dTotal*2.0+v0*v0);
	}

	if (std::abs(v1) > std::abs(maxSpeed))
		v1 = std::abs(maxSpeed)*MathUtils::sgn<double>(v1);

	return (a0*dTotal-v0*v1+(v0*v0)*(1.0/2.0)+(v1*v1)*(1.0/2.0))/(a0*v1);
}


void MotionController::calculatePlan2Part(double absAccel, double tTotal, double dTotal, double v0, PlanSolution & result)
{
	double v1,t0,t1;
	result.valid = false;

	double a0 = absAccel;
	
	
	double t0_sign = -(sqrt(a0*(dTotal*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))-a0*tTotal)/a0;
	
	if (std::isnan(t0_sign)) return;

	if (t0_sign < 0)
	{
		a0 = -a0;	
		v1 = v0+sqrt(a0*(dTotal*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))+a0*tTotal;
		t0 = (sqrt(a0*(dTotal*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))+a0*tTotal)/a0;
		t1 = -sqrt(a0*(dTotal*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))/a0;
	}
	else
	{
		v1 = v0-sqrt(a0*(dTotal*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))+a0*tTotal;
		t0 = -(sqrt(a0*(dTotal*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))-a0*tTotal)/a0;
		t1 = sqrt(a0*(dTotal*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))/a0;
	}
	
	result.t0 = t0;
	result.t1 = t1;
	result.accel0 = a0;
	result.travelVelocity = v1;
	result.valid = true;
}

void MotionController::calculatePlan(double absAccel, double d3, double tTotal, double dTotal, double v0, double v2, PlanSolution & result)
{
	double v1,t0,t1,t2,t3;
	result.valid = false;

	double a0 = absAccel, a1 = -absAccel;
	
	
	double t0_sign = -(v0+(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*
		(v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a0;
		
	double t2_sign = (v2+(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)
		*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a1;
	
	//Case 1: Both T0 and T2 are positive = good
	//Case 2: T0 is negative -> Flip sign of A0 and use EQ 2
	//Case 3: T2 is negative -> Flip sign of A1 and use EQ 2
	//Case 4: Both T0 and T2 are negative -> flip sign of both and use EQ 1
	//Case 5: Either T0 or T2 have non-real components (NaN). Return invalid result.

	
	if (std::isnan(t0_sign) || std::isnan(t2_sign)) return;
	if (t0_sign < 0) a0 = -a0;		
	if (t2_sign < 0) a1 = -a1;

	if (a0 == a1)
	{
		v1 = -((v0*v0)*v2-v2*v2*v2-a0*d3*v2*2.0+a0*dTotal*v2*2.0)/(a0*d3*2.0-v0*v2*2.0+(v2*v2)*2.0-a0*tTotal*v2*2.0);
		t0 = (-v0*(v2*v2)+(v0*v0)*v2*(1.0/2.0)+(v2*v2*v2)*(1.0/2.0)-a0*d3*v0+a0*d3*v2-a0*dTotal*v2+a0*tTotal*v0*v2)/(a0*(a0*d3-v0*v2+v2*v2-a0*tTotal*v2));
		t1 = -(a0*d3-v0*v2+v2*v2-a0*tTotal*v2)/(a0*v2);
		t2 = (-v0*(v2*v2)+(v0*v0)*v2*(1.0/2.0)+(v2*v2*v2)*(1.0/2.0)-a0*tTotal*(v2*v2)+a0*dTotal*v2)/(a0*(a0*d3-v0*v2+v2*v2-a0*tTotal*v2));
		t3 = d3/v2;
	}
	else
	{		
		t1 = -(a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2+(a0*v2*(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*
			(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)
			*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2))/(a0*v2-a1*v2)-(a1*v2*
			(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*
			(v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2))/(a0*v2-a1*v2))/(a0*a1*v2);

		if (t1 >= 0)
		{

			v1 = -(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)
				*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2)/(a0*v2-a1*v2);

			t0 = -(v0+(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*
				(v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a0;

			t2 = (v2+(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)
				*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a1;

			t3 = d3/v2;
		}
		else //other solution
		{
			v1 = (sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2)/(a0*v2-a1*v2);
			
			t0 = -(v0-(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a0;
			
			t1 = -(a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2-(a0*v2*(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*
				(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)
				*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2))/(a0*v2-a1*v2)+
				(a1*v2*(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)*2.0-a1*dTotal*
				(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2))/(a0*v2-a1*v2))/(a0*a1*v2);

			t2 = (v2-(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a1;
			
			t3 = d3/v2;
		}
	}
	
	result.t0 = t0;
	result.t1 = t1;
	result.t2 = t2;
	result.t3 = t3;
	result.accel0 = a0;
	result.accel1 = a1;
	result.travelVelocity = v1;
	result.valid = true;
}



shared_ptr<MotionPlan> MotionController::buildMotionPlan(const double startAngle,const  double targetAngle, const double targetTime, const  double approachVelocity, const double maxAccel)
{
	const double MinSpeedControlDistance = 150;
	auto plan = shared_ptr<MotionPlan>(new MotionPlan());
	
	if (std::abs(maxAccel) < 50) throw std::runtime_error("Acceleration must be at least 50 steps per second");

	double startVelocity = 0, direction = MathUtils::sgn<double>(targetAngle - startAngle);
	double endVelocity = std::abs(approachVelocity) * direction;
	double coastDistance = MinSpeedControlDistance * direction;
	
	double accel = std::abs(maxAccel)*direction;
	
	PlanSolution sol;
	
	calculatePlan(accel, coastDistance,targetTime,(targetAngle-startAngle),startVelocity,endVelocity,sol);

	if (!sol.valid)
	{
		stringstream ss;
		ss << "No valid solution found.";
		throw std::runtime_error(ss.str());
	}

	plan->motionIntervals.push_back(MotionInterval(startVelocity,sol.travelVelocity,sol.t0));
	
	if (sol.t1 > 0)
		plan->motionIntervals.push_back(MotionInterval(sol.travelVelocity,sol.t1));
	
	plan->motionIntervals.push_back(MotionInterval(sol.travelVelocity,endVelocity,sol.t2));
	plan->motionIntervals.push_back(MotionInterval(endVelocity,sol.t3));

	plan->startAngle = startAngle;
	plan->finalAngle = targetAngle;		

	cout << "V0=" << sol.travelVelocity << " T0=" << sol.t0 << " T1=" << sol.t1 << " T2=" << sol.t2 << " T3=" << sol.t3 <<endl;
	cout << "FinalAngle=" << AS5048::stepsToDegrees(plan->getPositionAtTime(1000)) << endl;
	
	return plan;
}


void MotionController::setJointPosition(int jointIndex, double targetAngle, double travelTime, double accel)
{
	const double MinSpeedControlDistance = 150.0;

	if (jointIndex >= 0 && jointIndex < joints.size()){		
			
		auto pjc = joints.at(jointIndex);
		auto plan = buildMotionPlan(pjc->getCurrentAngle(),targetAngle,travelTime,pjc->getJointModel()->servoModel.controllerConfig.approachVelocity, accel);
		
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


void MotionController::moveToPosition(Vector3d targetPosition, Matrix3d targetRotation, double accel, double deccel, bool interactive)
{	
	vector<MotionStep*> motionSteps;
	bool solutionFound = buildMotionSteps(targetPosition, targetRotation, motionSteps);

	if (solutionFound) {

		currentPlan.clear();
		currentPlan = createMotionPlans(motionSteps,accel,deccel);

		bool executePlan = false;

		if (interactive)
		{
			cout << "Angles    = ";
			for (auto it = currentPlan.begin(); it != currentPlan.end(); it++)
			{
				cout << setprecision(2) << std::round(AS5048::stepsToDegrees((*it)->finalAngle)/0.01)*0.01 << "  ";
			}
			cout << endl;
			cout << "Velocities = ";
			for (auto it = currentPlan.begin(); it != currentPlan.end(); it++)
			{
				double time = (*it)->getPlanDuration();
				cout << setprecision(2) << std::round(AS5048::stepsToDegrees((*it)->getSpeedAtTime(time/2.0))/0.01)*0.01 << "   ";
			}
			cout << endl;
			cout << std::fixed;
			cout << "Commence motion? [y]es/[a]bort :" << endl;
			string strIn;
			getline(cin,strIn);

			if (strIn.compare("y") != 0)
			{				
				currentPlan.clear();
			}
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
	else {
		cout << "Failed to construct motion plan for target. " << endl;
	}
	
	std::for_each( motionSteps.begin(), motionSteps.end(),[](MotionStep * step){delete step;});
}


void MotionController::postTask(std::function<void()> task)
{
	std::lock_guard<std::mutex> locks(taskQueueMutex);
	taskQueue.push(task);
}

vector<shared_ptr<MotionPlan> > MotionController::createMotionPlans(vector<MotionStep*> & steps, double maxAccel, double maxDeccel)
{
	const double CoastDistance = 150;
	
	vector<shared_ptr<MotionPlan> > motionPlan;

	for (int i=0;i<6;i++) 
	{
		motionPlan.push_back(shared_ptr<MotionPlan>(new MotionPlan()));
	}

	
	for (int stepIndex = 0;stepIndex < steps.size(); stepIndex++)
	{
		auto step = steps.at(stepIndex);
		bool lastStep = stepIndex == steps.size() - 1;

		double stepTime = 0;
		for (int i=0;i<6;i++) 
		{						
			double lastVelocity = 0;

			if (stepIndex > 0) lastVelocity = motionPlan.at(i)->getSpeedAtTime(motionPlan.at(i)->getPlanDuration());

			double delta = AS5048::radiansToSteps(step->jointAngleDelta[i]);
			double direction = MathUtils::sgn<double>(delta);
			double maxSpeed = joints.at(i)->getMaxJointVelocity() * direction;
			double coastVelocity = joints.at(i)->getJointModel()->servoModel.controllerConfig.approachVelocity*direction;

			double travelVelocity;

			double jointTime;
			
			if (lastStep)			
				jointTime = optimalSpeed(maxAccel,CoastDistance * direction,delta,lastVelocity,coastVelocity,maxSpeed,travelVelocity);			
			else
				jointTime = optimalSpeed2Part(maxAccel,delta,lastVelocity,maxSpeed,travelVelocity);							

			stepTime = std::max(stepTime,jointTime);
		}

		for (int i=0;i<6;i++) 
		{	
			double lastVelocity = 0;		
			if (stepIndex > 0) lastVelocity = motionPlan.at(i)->getSpeedAtTime(motionPlan.at(i)->getPlanDuration());

			double delta = AS5048::radiansToSteps(step->jointAngleDelta[i]);
			double direction = MathUtils::sgn<double>(delta);
			double coastVelocity = joints.at(i)->getJointModel()->servoModel.controllerConfig.approachVelocity*direction;
			
			PlanSolution sol;
			if (lastStep)
			{
			 	calculatePlan(maxAccel,CoastDistance*direction,stepTime,delta,lastVelocity,coastVelocity,sol);
				motionPlan.at(i)->motionIntervals.push_back(MotionInterval(lastVelocity,sol.travelVelocity,sol.t0));

				if (sol.t1 >= samplePeriod) 
				{
					motionPlan.at(i)->motionIntervals.push_back(MotionInterval(sol.travelVelocity,sol.t1));
					cout << "Joint[" << i << "] v1=" << sol.travelVelocity << " delta=" << delta << endl;
				}

				motionPlan.at(i)->motionIntervals.push_back(MotionInterval(sol.travelVelocity,coastVelocity,sol.t2));
				motionPlan.at(i)->motionIntervals.push_back(MotionInterval(coastVelocity,sol.t3));
			}
			else
			{
				calculatePlan2Part(maxAccel,stepTime,delta,lastVelocity,sol);				
				motionPlan.at(i)->motionIntervals.push_back(MotionInterval(lastVelocity,sol.travelVelocity,sol.t0));
				
				if (sol.t1 >= samplePeriod)
					motionPlan.at(i)->motionIntervals.push_back(MotionInterval(sol.travelVelocity,sol.t1));
			}
		}
	}

	for (int i=0;i<6;i++)
	{
		motionPlan.at(i)->startAngle = AS5048::radiansToSteps(steps.front()->targetJointAngles[i] - steps.front()->jointAngleDelta[i]);
		motionPlan.at(i)->finalAngle = AS5048::radiansToSteps(steps.back()->targetJointAngles[i]);
	}

	return motionPlan;
}

void MotionController::getJointAngles(double * angles)
{
	int i=0;
	for (auto it = joints.begin(); it != joints.end(); it++,i++)
	{					
		angles[i] = AS5048::stepsToRadians((*it)->getCurrentAngle());
	}
}


bool MotionController::checkSolutionValid(const double * solution)
{
	int j=0;
	for (auto it = joints.begin(); it != joints.end(); it++,j++)
	{
		JointModel * jointModel = (*it)->getJointModel();
		double angleSteps = AS5048::radiansToSteps(solution[j]);
		if (angleSteps < jointModel->minAngle || angleSteps > jointModel->maxAngle)
			return false;
	}
	return true;
}

double MotionController::calculateMotionEffort(const double * currentSolution, const double * targetSolution)
{
	if (checkSolutionValid(targetSolution)) {

		double maxDiff = MathUtil::PI * 6.0;
		double totalDiff = 0;
		for (int i=0;i<6;i++) {
			totalDiff += MathUtil::abs(MathUtil::subtractAngles(currentSolution[i],targetSolution[i]));
		}

		if (totalDiff > maxDiff) {
			cout << "Unexpected value for total effort: " << totalDiff << "rad . Max effort = " << maxDiff << " rad" << endl;
		}

		return maxDiff - totalDiff;
	}
	
	return 0;
}

bool MotionController::getEasiestSolution(const double * currentAngles, Vector3d targetPosition, Matrix3d rotationMatrix, double * result) {

	double targetRotation[9];
	MathUtil::getRowMajorData(rotationMatrix,targetRotation);


	IkSolutionList<IkReal> solutions;
	ComputeIk(targetPosition,targetRotation,NULL, solutions);
	
	double bestSolution[6];
	double bestSolutionScore = 0;

	for (int i=0;i<solutions.GetNumSolutions();i++)
	{		
		double solution[6];

		solutions.GetSolution(i).GetSolution(solution,NULL);

		double score = calculateMotionEffort(currentAngles,solution);

		if (score > bestSolutionScore) {
			bestSolutionScore = score;
			std::copy(std::begin(solution), std::end(solution), std::begin(bestSolution));
		}
	}
	
	if (bestSolutionScore > 0) {

		for (int i=0;i<6;i++)
			result[i] = bestSolution[i];

		return true;
	}
	return false;
}


bool MotionController::buildMotionSteps(double * targetPosition,Matrix3d targetRotation, vector<MotionStep*> & steps)
{
	int numSteps = planStepCount;

	double currentPosition[3];
	double rotationMatrix[9];

	double jointAngles[6];
	getJointAngles(jointAngles);
	
	//Store robot space angles
	double lastAngles[6];
	std::copy(std::begin(jointAngles), std::end(jointAngles), std::begin(lastAngles));

	ComputeFk(jointAngles,currentPosition,rotationMatrix);

	double delta[3];
	MathUtil::subtractVectors(targetPosition,currentPosition,delta);

	bool planningSucceeded = true;

	
	for (int i=1;i<=numSteps;i++) {

		double stepPosition[3];
		double stepSize[3];
		MathUtil::scaleVector(delta,((double)i)/((double)numSteps),stepSize);
		MathUtil::addVectors(currentPosition,stepSize,stepPosition);

		double stepAngles[6];

		Vector3d v = Vector3d(stepPosition[0],stepPosition[1],stepPosition[2]);
		bool solutionExists = getEasiestSolution(lastAngles,v,targetRotation,stepAngles);

		if (solutionExists) {	
			
			MotionStep * step = new MotionStep();
			//Determine the max joint velocities and store in the step object
			for (int j=0;j<6;j++) {
				step->jointAngleDelta[j] = MathUtil::subtractAngles(stepAngles[j],lastAngles[j],MathUtil::PI);
			}
			//Next, copy the target joint angles to the step object
			std::copy(std::begin(stepAngles), std::end(stepAngles), std::begin(step->targetJointAngles));
			std::copy(std::begin(stepAngles), std::end(stepAngles), std::begin(lastAngles));
			//Then copy the tip position
			std::copy(std::begin(stepPosition), std::end(stepPosition), std::begin(step->targetPosition));
			//Finally, store the step object in the result vector
			steps.push_back(step);
		}
		else
		{
			cout << "Solution could not be found for position: " << stepPosition[0] << "," << stepPosition[1] << "," << stepPosition[2] << endl;
			planningSucceeded = false;
			break;
		}
	}

	return planningSucceeded;	
}