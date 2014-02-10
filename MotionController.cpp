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


void MotionController::setJointPosition(int jointIndex, double angle, double velocity, double accel)
{
	if (jointIndex >= 0 && jointIndex < joints.size()){
		
		PredictiveJointController * pjc = joints.at(jointIndex);
		
		auto plan = shared_ptr<JointMotionPlan>(new JointMotionPlan());
		
		if (accel != 0)
		{
			double accelTime = velocity / accel;
			if (accelTime > samplePeriod*4.0)
			{
				plan->motionIntervals.push_back(new MotionInterval(0,velocity,accelTime));
				plan->motionIntervals.push_back(new MotionInterval(velocity,std::numeric_limits<double>::infinity()));
			}
		}
		
		if (plan->motionIntervals.empty())
		{
			plan->motionIntervals.push_back(new MotionInterval(velocity,std::numeric_limits<double>::infinity()));
		}

		plan->finalAngle = angle;
		
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


void MotionController::moveToPosition(Vector3d targetPosition, bool interactive)
{	
	vector<MotionStep*> motionSteps;
	bool solutionFound = buildMotionSteps(targetPosition,motionSteps);

	if (solutionFound) {
	
		currentPlan.clear();
		currentPlan = createMotionPlans(motionSteps,100,100);

		if (interactive)
		{
			//cout << "Motion plan created: ";
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
		}

		string strIn;
		getline(cin,strIn);
		
		if (strIn.compare("y") == 0) {

			cout << "Executing plan" << endl;

			this->postTask([this](){

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
		else {
			currentPlan.clear();
			cout << "Aborted motion plan" << endl;
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

vector<shared_ptr<JointMotionPlan> > MotionController::createMotionPlans(vector<MotionStep*> & steps, double maxAccel, double maxDeccel)
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
			double jointTime = AS5048::radiansToSteps(std::abs((*it)->jointAngleDelta[i])) / joints.at(i)->getMaxJointVelocity();
			stepTime  = std::max(stepTime ,jointTime);
		}

		for (int i=0;i<6;i++) 
		{			
			double velocity =  AS5048::radiansToSteps((*it)->jointAngleDelta[i]/stepTime);
			double lastVelocity = 0;
			if (motionPlan.at(i)->motionIntervals.size() > 0)
			{
				lastVelocity = motionPlan.at(i)->motionIntervals.back()->endSpeed;
			}
			double accelTime = (velocity - lastVelocity)/maxAccel;
			
			if (accelTime > samplePeriod*4.0 && accelTime < stepTime) 
			{
				motionPlan.at(i)->motionIntervals.push_back(new MotionInterval(lastVelocity,velocity,accelTime));
				motionPlan.at(i)->motionIntervals.push_back(new MotionInterval(velocity,stepTime-accelTime));
			}
			else
			{
				motionPlan.at(i)->motionIntervals.push_back(new MotionInterval(velocity,stepTime));
			}

			motionPlan.at(i)->finalAngle = AS5048::radiansToSteps((*it)->targetJointAngles[i]);
		}
	}

	for (int i=0;i<6;i++)
	{
		if (motionPlan.at(i)->motionIntervals.size() > 0)
		{
			double lastVelocity = motionPlan.at(i)->motionIntervals.back()->endSpeed;
			double accelTime = (lastVelocity)/maxDeccel;
			
			if (accelTime > samplePeriod*4.0 && accelTime < motionPlan.at(i)->motionIntervals.back()->duration) 
			{				
				motionPlan.at(i)->motionIntervals.back()->duration -= accelTime;
				motionPlan.at(i)->motionIntervals.push_back(new MotionInterval(0,accelTime));
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

bool MotionController::getEasiestSolution(const double * currentAngles, const double * targetPosition, double * result) {

	double targetRotation[9] = {0,0,1, 0,1,0, -1,0,0}; //+90 about Y

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
		//std::copy(std::begin(bestSolution), std::end(bestSolution), std::begin(result));
		for (int i=0;i<6;i++)
			result[i] = bestSolution[i];

		return true;
	}
	return false;
}


bool MotionController::buildMotionSteps(double * targetPosition,vector<MotionStep*> & steps)
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

		bool solutionExists = getEasiestSolution(lastAngles,stepPosition,stepAngles);

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