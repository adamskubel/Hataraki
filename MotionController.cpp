#include "MotionController.hpp"

using namespace std;
using namespace ikfast2;
using namespace ikfast;

int MotionController::PlanStepCount = 1;


MotionController::MotionController(vector<PredictiveJointController*> & _joints, long _updatePeriod) {
	this->joints = _joints;
	this->updatePeriod = _updatePeriod;

	state = MotionControllerState::Waiting;
}

void MotionController::updateController(){

	try 
	{
		struct timespec start,end;
		clock_gettime(CLOCK_REALTIME, &start);

		for (auto it = joints.begin(); it != joints.end(); it++)
		{
			(*it)->run();
		}

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


void MotionController::setJointPosition(int jointIndex, double angle, double velocity)
{
	if (jointIndex >= 0 && jointIndex < joints.size()){
		
		PredictiveJointController * pjc = joints.at(jointIndex);
		
		auto plan = shared_ptr<JointMotionPlan>(new JointMotionPlan(new MotionInterval(velocity,std::numeric_limits<double>::infinity()),angle));
		
		pjc->validateMotionPlan(plan);
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
	const double zeroVelocity = MathUtil::degreesToRadians(40);
	
	for (auto it=joints.begin();it!=joints.end();it++)
	{
		auto plan = shared_ptr<JointMotionPlan>(new JointMotionPlan(new MotionInterval(zeroVelocity,std::numeric_limits<double>::infinity()),0));
		(*it)->validateMotionPlan(plan);
	}
	
	for (auto it=joints.begin();it!=joints.end();it++)
	{
		auto plan = shared_ptr<JointMotionPlan>(new JointMotionPlan(new MotionInterval(zeroVelocity,std::numeric_limits<double>::infinity()),0));
		(*it)->executeMotionPlan(plan);
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


void MotionController::moveToPosition(Vector3d targetPosition)
{	
	if (state == MotionControllerState::Stepping)
	{
		cout << "Cannot command new movement until current motion plan completes." << endl;
		return;
	}

	vector<MotionStep*> motionSteps;
	bool solutionFound = buildMotionSteps(targetPosition,motionSteps);
	std::for_each( motionSteps.begin(), motionSteps.end(),[](MotionStep * step){delete step;});

	if (solutionFound) {
	
		currentPlan.clear();
		currentPlan = createMotionPlans(motionSteps);

		cout << "Motion plan created: " << endl;

		cout << "Commence motion? [y]es/[a]bort :" << endl;

		string strIn;
		getline(cin,strIn);
		
		if (strIn.compare("y") == 0) {
			
			for (int i=0;i<6;i++)
			{
				joints.at(i)->validateMotionPlan(currentPlan.at(i));
			}
			
			for (int i=0;i<6;i++)
			{
				joints.at(i)->executeMotionPlan(currentPlan.at(i));
			}
			
		}
		else {
			currentPlan.clear();
			cout << "Aborted motion plan" << endl;
		}
	}
	else {
		cout << "Failed to construct motion plan for target. " << endl;
	}
}


void MotionController::postTask(std::function<void()> task)
{
	std::lock_guard<std::mutex> locks(taskQueueMutex);
	taskQueue.push(task);
}

vector<shared_ptr<JointMotionPlan> > MotionController::createMotionPlans(vector<MotionStep*> & steps)
{
	vector<shared_ptr<JointMotionPlan> > motionPlan;

	for (int i=0;i<6;i++) 
	{
		motionPlan.push_back(shared_ptr<JointMotionPlan>(new JointMotionPlan()));
	}

	for (auto it=steps.begin();it != steps.end(); it++)
	{
		double stepTime = 0;
		for (int i=0;i<6;i++) 
		{			
			double jointTime = std::abs((*it)->maxJointVelocities[i] / joints.at(i)->getMaxJointVelocity());
			stepTime  = std::max(stepTime ,jointTime);
		}

		for (int i=0;i<6;i++) 
		{			
			double velocity = (*it)->maxJointVelocities[i]/stepTime;
			motionPlan.at(i)->motionIntervals.push_back(new MotionInterval(velocity,stepTime));
		}
	}

	return motionPlan;
}

void MotionController::getJointAngles(double * angles)
{
	int i=0;
	for (auto it = joints.begin(); it != joints.end(); it++,i++)
	{					
		angles[i] = MathUtil::degreesToRadians((*it)->getCurrentAngle());
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

bool MotionController::getEasiestSolution(const double * currentAngles, const double * targetPosition, double * result) {

	double targetRotation[9] = {0,0,-1, 0,1,0, 1,0,0}; //-90 about Y

	IkSolutionList<IkReal> solutions;
	ComputeIk(targetPosition,targetRotation,NULL, solutions);
	
	double bestSolution[6];
	double bestSolutionScore = 0;

	for (int i=0;i<solutions.GetNumSolutions();i++)
	{		
		double solution[6];

		solutions.GetSolution(i).GetSolution(solution,NULL);

		//Convert to robot space
		if (InvertPitchJoints) {
			//Invert pitch joint angles
			solution[1] *= -1.0;
			solution[3] *= -1.0;
			solution[4] *= -1.0;
		}

		double score = calculateMotionEffort(currentAngles,solution);

		if (score > bestSolutionScore) {
			bestSolutionScore = score;
			std::copy(std::begin(solution), std::end(solution), std::begin(bestSolution));
		}
	}
	
	if (bestSolutionScore > 0) {
		//std::copy(std::begin(bestSolution), std::end(bestSolution), std::begin(result));
		for (int i=0;i<6;i++)
			result[i] = bestSolution[i];

		return true;
	}
	return false;
}


bool MotionController::buildMotionSteps(double * targetPosition,vector<MotionStep*> & steps)
{
	int numSteps = MotionController::PlanStepCount;

	double currentPosition[3];
	double rotationMatrix[9];

	double jointAngles[6];
	getJointAngles(jointAngles);
	
	//Store robot space angles
	double lastAngles[6];
	std::copy(std::begin(jointAngles), std::end(jointAngles), std::begin(lastAngles));

	//Convert to IK space for FK calculation
	if (InvertPitchJoints) {
		//Invert pitch joint angles
		jointAngles[1] *= -1.0;
		jointAngles[3] *= -1.0;
		jointAngles[4] *= -1.0;
	}

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

		bool solutionExists = getEasiestSolution(lastAngles,stepPosition,stepAngles);

		if (solutionExists) {	
			
			//double maxTime = 0;
			//for (int j=0;j<6;j++) {				
			//	double jointDelta = MathUtil::abs(MathUtil::subtractAngles(lastAngles[j],stepAngles[j]));
			//	double jointTime = jointDelta / joints.at(j)->getMaxJointVelocity();
			//	maxTime = std::max(maxTime,jointTime);
			//}
			
			MotionStep * step = new MotionStep();
			//Determine the max joint velocities and store in the step object
			for (int j=0;j<6;j++) {
				step->maxJointVelocities[j] = MathUtil::abs(MathUtil::subtractAngles(lastAngles[j],stepAngles[j]));
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