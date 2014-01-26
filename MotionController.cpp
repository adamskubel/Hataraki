#include "MotionController.hpp"


using namespace std;
using namespace ikfast2;
using namespace ikfast;

int MotionController::PlanStepCount = 1;


MotionController::MotionController(vector<JointLoop*> & _joints, long _updatePeriod) {
	this->joints = _joints;
	this->updatePeriod = _updatePeriod;

	state = MotionControllerState::Waiting;
}

void MotionController::commandMotionStep(MotionStep * step) {

	for (int i=0;i<6;i++) {		
		joints.at(i)->setTargetAngle(MathUtil::radiansToDegrees(step->targetJointAngles[i]));
		joints.at(i)->setTargetJointVelocity(step->maxJointVelocities[i]);
	}
}

void MotionController::postTask(std::function<void()> task)
{
	std::lock_guard<std::mutex> locks(taskQueueMutex);
	taskQueue.push(task);
}


void MotionController::updateController(){

	struct timespec start,end;
	clock_gettime(CLOCK_REALTIME, &start);


	if (state == MotionControllerState::Stepping) {

		bool settled = true;
		for (auto it = joints.begin(); it != joints.end(); it++)
		{
			settled = settled && (*it)->isSettled();
		}

		//Ready for next step
		if (settled) {
			
			if (currentStep < currentPlan.size()) {
				commandMotionStep(currentPlan.at(currentStep));
				cout << "Step " << currentStep << " complete." << endl;
				currentStep++;
			}
			else {
				state = MotionControllerState::Waiting;

				for (int i=0;i<6;i++) {		
					joints.at(i)->setTargetJointVelocity(0);
				}
				
				cout << "Plan completed!" << endl;
			}
		}
	}


	for (auto it = joints.begin(); it != joints.end(); it++)
	{
		(*it)->run();
	}

	if (taskQueueMutex.try_lock()) {		
		while (!taskQueue.empty()) {
			taskQueue.front()();
			taskQueue.pop();
		}
		taskQueueMutex.unlock();
	}


	clock_gettime(CLOCK_REALTIME, &end);
	double totalTime = (end.tv_nsec - start.tv_nsec)/1000000000.0;
	if (totalTime < 0) totalTime += 1.0;
	//avgTotalTime = avgTotalTime*.9 + totalTime*.1;

	long adjustedSleep = updatePeriod - (totalTime*1000000);
	if (adjustedSleep > 0 && adjustedSleep <= updatePeriod)
		usleep(adjustedSleep);

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
		if (!(*it)->checkCommandValid(MathUtil::radiansToDegrees(solution[j])))
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

void MotionController::moveToPosition(double * targetPosition)
{	
	if (state == MotionControllerState::Stepping)
	{
		cout << "Cannot command new movement until current motion plan completes." << endl;
		return;
	}

	currentPlan.clear();
	if (buildMotionSteps(targetPosition,currentPlan)) {
		
		cout << "Motion plan created: " << endl;

		int stepNum = 0;
		for (auto it = currentPlan.begin(); it != currentPlan.end(); it++,stepNum++) {
			
			cout << "Step[" << stepNum << "]: Target (cm) = ";			
			for (int j=0;j<3;j++) cout << (*it)->targetPosition[j]*100.0 << " ";			
			cout << endl << "Angles (deg) = ";
			for (int j=0;j<6;j++) cout << MathUtil::radiansToDegrees((*it)->targetJointAngles[j]) << " ";
			cout << endl << "Velocities (steps/s) = ";
			for (int j=0;j<6;j++) cout << AS5048::radiansToSteps((*it)->maxJointVelocities[j]) << " ";
			cout << endl << endl;
		}

		cout << "Commence motion? [y]es/[a]bort :" << endl;

		string strIn;
		getline(cin,strIn);
		
		if (strIn.compare("y") == 0) {			
			currentStep = 0;
			state = MotionControllerState::Stepping;
			cout << "Stepping..." << endl;
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
			
			double maxTime = 0;
			for (int j=0;j<6;j++) {				
				double jointDelta = MathUtil::abs(MathUtil::subtractAngles(lastAngles[j],stepAngles[j]));
				double jointTime = jointDelta / joints.at(j)->getMaxJointVelocity();
				maxTime = std::max(maxTime,jointTime);
			}
			
			MotionStep * step = new MotionStep();
			//Determine the max joint velocities and store in the step object
			for (int j=0;j<6;j++) {
				step->maxJointVelocities[j] = MathUtil::abs(MathUtil::subtractAngles(lastAngles[j],stepAngles[j])) / maxTime;
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