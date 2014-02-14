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

shared_ptr<MotionPlan> MotionController::buildMotionPlan(const double startAngle,const  double targetAngle, const double targetTravelTime, const  double approachVelocity, const double maxVelocity, const double maxAccel)
{
	const double MinSpeedControlDistance = 150;
	auto plan = shared_ptr<MotionPlan>(new MotionPlan());
	
	if (std::abs(maxAccel) < 50) throw std::runtime_error("Acceleration must be at least 50 steps per second");

	double startVelocity = 0, travelVelocity = (targetAngle - startAngle)/targetTravelTime;
	double endVelocity = std::abs(approachVelocity) * MathUtils::sgn<double>(travelVelocity);
	double coastDistance = MinSpeedControlDistance * MathUtils::sgn<double>(travelVelocity);
	
	double accel = std::abs(maxAccel)*MathUtils::sgn<double>(travelVelocity);
	double decel = -accel;

	double travelTime = targetTravelTime;

	if (std::abs(travelVelocity) > maxVelocity)
	{
		cout << "Warning: Target velocity " << AS5048::stepsToDegrees(travelVelocity)
			<< " exceeds max joint velocity " << AS5048::stepsToDegrees(maxVelocity) << endl;

		travelVelocity = MathUtils::sgn<double>(travelVelocity) * maxVelocity;
		travelTime = (targetAngle-startAngle)/travelVelocity;
	}
			
	double accelTime = (travelVelocity - startVelocity)/accel;
	double decelTime = (endVelocity - travelVelocity)/decel;
	double coastTime = coastDistance/endVelocity;

	double equivAccelTime = getAccelDistFromSpeed(startVelocity,travelVelocity,accel)/travelVelocity;
	double equivDecelTime = getAccelDistFromSpeed(travelVelocity,endVelocity,decel)/travelVelocity;
	double coastEquivTime = coastDistance/travelVelocity;

	travelTime -= (equivAccelTime + equivDecelTime + coastEquivTime);

	plan->motionIntervals.push_back(MotionInterval(startVelocity,travelVelocity,accelTime));			
	if (travelTime > 0) plan->motionIntervals.push_back(MotionInterval(travelVelocity,travelTime));
	plan->motionIntervals.push_back(MotionInterval(travelVelocity,endVelocity,decelTime));
	plan->motionIntervals.push_back(MotionInterval(endVelocity,coastTime));

	plan->startAngle = startAngle;
	plan->finalAngle = targetAngle;		

	cout << "T0=" << accelTime << " T1=" << travelTime << " T2=" << decelTime << " T3=" << coastTime<<endl;
	cout << "FinalAngle=" << AS5048::stepsToDegrees(plan->getPositionAtTime(1000)) << endl;

	return plan;
}


void MotionController::setJointPosition(int jointIndex, double targetAngle, double travelTime, double accel)
{
	const double MinSpeedControlDistance = 150.0;

	if (jointIndex >= 0 && jointIndex < joints.size()){
		
		//PredictiveJointController * pjc = joints.at(jointIndex);
		//
		//auto plan = shared_ptr<MotionPlan>(new MotionPlan());

		//double startVelocity = 0;
		//double endVelocity = pjc->getJointModel()->servoModel.controllerConfig.approachVelocity;
		//		
		//double startAngle = pjc->getCurrentAngle();

		//double delta = targetAngle - startAngle;
		//
		//double targetVelocity = delta/travelTime;
		//
		//endVelocity = std::abs(endVelocity) * MathUtils::sgn<double>(targetVelocity);

		//double coastDistance = MinSpeedControlDistance * MathUtils::sgn<double>(targetVelocity);
		//
		//if (std::abs(accel) < 50) accel = 1000;
		//
		//accel = std::abs(accel)*MathUtils::sgn<double>(targetVelocity);
		//double decel = -accel;

		//if (std::abs(targetVelocity) > pjc->getMaxJointVelocity())
		//{
		//	cout << "Warning: Target velocity " << AS5048::stepsToDegrees(targetVelocity)
		//	<< " exceeds max joint velocity " << AS5048::stepsToDegrees(pjc->getMaxJointVelocity()) << endl;
		//	
		//	targetVelocity = MathUtils::sgn<double>(targetVelocity) * pjc->getMaxJointVelocity();
		//	travelTime = delta/targetVelocity;
		//}
		//		
		//	
		//double accelTime = (targetVelocity - startVelocity)/accel;
		//double accelDist = accelTime*startVelocity + (std::pow(accelTime,2) * accel)/2.0;
		//double equivTime = accelDist/targetVelocity;
		//
		//double decelTime = (endVelocity - targetVelocity)/decel;
		//double decelDist = decelTime*targetVelocity + (std::pow(decelTime,2) * decel)/2.0;
		//double decelEquivTime = decelDist/targetVelocity;
		//	
		//double coastTime = coastDistance/endVelocity;
		//double coastEquivTime = coastDistance/targetVelocity;
		//
		//travelTime -= (equivTime + decelEquivTime + coastEquivTime);

		//plan->motionIntervals.push_back(MotionInterval(startVelocity,targetVelocity,accelTime));			
		//if (travelTime > 0) plan->motionIntervals.push_back(MotionInterval(targetVelocity,travelTime));
		//plan->motionIntervals.push_back(MotionInterval(targetVelocity,endVelocity,decelTime));
		//plan->motionIntervals.push_back(MotionInterval(endVelocity,coastTime));
		//
		//plan->startAngle = startAngle;
		//plan->finalAngle = targetAngle;		
		//
		//cout << "T0=" << accelTime << " T1=" << travelTime << " T2=" << decelTime << " T3=" << coastTime<<endl;
		//cout << "FinalAngle=" << AS5048::stepsToDegrees(plan->getPositionAtTime(1000)) << endl;
		
		auto pjc = joints.at(jointIndex);
		auto plan = buildMotionPlan(pjc->getCurrentAngle(),targetAngle,travelTime,pjc->getJointModel()->servoModel.controllerConfig.approachVelocity, pjc->getMaxJointVelocity(),accel);
		
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
		setJointPosition(i,0,AS5048::degreesToSteps(20),0);
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
				cout << setprecision(2) << std::round(AS5048::stepsToDegrees((*it)->getSpeedAtTime(0))/0.01)*0.01 << "  ";
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
	const double MinSpeed = 400;

	vector<shared_ptr<MotionPlan> > motionPlan;

	for (int i=0;i<6;i++) 
	{
		motionPlan.push_back(shared_ptr<MotionPlan>(new MotionPlan()));
	}

	for (auto it=steps.begin();it != steps.end(); it++)
	{
		double stepTime = 0;
		for (int i=0;i<6;i++) 
		{			
			double jointTime = AS5048::radiansToSteps(std::abs((*it)->jointAngleDelta[i])) / joints.at(i)->getMaxJointVelocity();
			stepTime  = std::max(stepTime ,jointTime);
		}

		for (int i=0;i<6;i++) 
		{			
			double velocity =  AS5048::radiansToSteps((*it)->jointAngleDelta[i]/stepTime);
			double lastVelocity = 0;
			if (motionPlan.at(i)->motionIntervals.size() > 0)
			{
				lastVelocity = motionPlan.at(i)->motionIntervals.back().endSpeed;
			}
			double accelTime = (velocity - lastVelocity)/maxAccel;
			
			if (accelTime > samplePeriod*4.0 && accelTime < stepTime) 
			{
				motionPlan.at(i)->motionIntervals.push_back(MotionInterval(lastVelocity,velocity,accelTime));

				double accelDist = accelTime*lastVelocity + (std::pow(accelTime,2) * maxAccel)/2.0;
				double equivTime = accelDist/velocity;

				motionPlan.at(i)->motionIntervals.push_back(MotionInterval(velocity,stepTime-equivTime));
			}
			else
			{
				motionPlan.at(i)->motionIntervals.push_back(MotionInterval(velocity,stepTime));
			}

			motionPlan.at(i)->finalAngle = AS5048::radiansToSteps((*it)->targetJointAngles[i]);
		}
	}

	for (int i=0;i<6;i++)
	{
		if (motionPlan.at(i)->motionIntervals.size() > 0)
		{
			double lastVelocity = motionPlan.at(i)->motionIntervals.back().endSpeed;
			double accelTime = (lastVelocity - MinSpeed)/maxDeccel;
			
			if (accelTime > samplePeriod*4.0) 
			{				
				if (accelTime < motionPlan.at(i)->motionIntervals.back().duration)
				{
					double accelDist = accelTime*lastVelocity + (std::pow(accelTime,2) * -maxDeccel)/2.0;
					double equivTime = accelDist/lastVelocity;

					motionPlan.at(i)->motionIntervals.back().duration -= equivTime;
					motionPlan.at(i)->motionIntervals.push_back(MotionInterval(MinSpeed,accelTime));
				}
				//else
				//{
				//	motionPlan.at(i)->motionIntervals.back()->endSpeed = 0;
				//}
			}
		}
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
				step->jointAngleDelta[j] = MathUtil::abs(MathUtil::subtractAngles(lastAngles[j],stepAngles[j]));
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