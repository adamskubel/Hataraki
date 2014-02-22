#include "MotionPlanner.hpp"

using namespace std;
using namespace vmath;
using namespace ikfast2;
using namespace ikfast;

MotionPlanner::MotionPlanner(vector<PredictiveJointController*> joints)
{
	this->joints = joints;
	this->interpolationDistance = 0.5;
	this->firstStepTime = 0.1;
	this->lastStepTime = 0.1;
	this->pathDivisionCount = 20;
	
	vector<double> velocityLimits, accelLimits, jerkLimits;
	
	for (auto it=joints.begin(); it != joints.end(); it++)
	{
		velocityLimits.push_back((*it)->getMaxJointVelocity());
		accelLimits.push_back((*it)->getMaxAcceleration());
		jerkLimits.push_back(0);
	}
		
	this->pathPlanner = new PathPlanner(6,velocityLimits,accelLimits,jerkLimits);
}

bool MotionPlanner::checkSolutionValid(const double * solution)
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


vector<Step> MotionPlanner::realizeSteps(std::vector<Step> & steps)
{
	vector<Step> initial = steps;
	vector<Step> interp = pathPlanner->interpolate(initial, interpolationDistance);

	cout  << "Interpolated into " << interp.size() << " steps" << endl;
	
	vector<Step> realized;
	realized.push_back(interp.front());	
	realized.insert(realized.end(), interp.begin(), interp.end());
	realized.push_back(interp.back());

	realized.front().TimeOffset = 0;
	realized.at(1).TimeOffset = firstStepTime;
	realized.back().TimeOffset = lastStepTime;
	
	cout  << "Realized into " << interp.size() << " steps" << endl;
	
	auto res = pathPlanner->plan(realized);
	cout  << "Planned into " << res.size() << " steps" << endl;
	return res;
}


double MotionPlanner::calculateMotionEffort(const double * currentSolution, const double * targetSolution)
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

bool MotionPlanner::getEasiestSolution(const double * currentAngles, Vector3d targetPosition, Matrix3d rotationMatrix, double * result) {
	
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


vector<shared_ptr<MotionPlan> > MotionPlanner::convertStepsToPlans(std::vector<Step> & steps, vector<double> initialAngles)
{
vector<shared_ptr<MotionPlan> > plans;
	
	int numChannels = 6;
	
	for (int c=0;c<numChannels;c++)
	{
		auto plan = shared_ptr<MotionPlan>(new MotionPlan());		
		plan->startAngle = initialAngles[c];
		plans.push_back(plan);
		plan->finalAngle = steps.back().Positions[c];
	}
	
	for (int i=1;i < steps.size(); i++)
	{
		Step * s0 = &steps[i];
		Step * s1 = &steps[i-1];
		
		for (int c=0;c<numChannels;c++)
		{
			double t0 = s0->TimeOffset;
			double t1 = s1->TimeOffset;
			
			double t1_0 = 0.5*(t0+t1);
			
			double lastVelocity = 0;
			if (plans[c]->motionIntervals.size() > 0)
				lastVelocity = plans[c]->motionIntervals.back().endSpeed;
			
			plans[c]->motionIntervals.push_back(MotionInterval(lastVelocity,s0->Velocities[c],t1_0));
		}
	}
	
	return plans;
}


shared_ptr<MotionPlan> MotionPlanner::buildMotionPlan(const double startAngle,const  double targetAngle, const double targetTime, const  double approachVelocity, const double maxAccel)
{
	const double MinSpeedControlDistance = 150;
	auto plan = shared_ptr<MotionPlan>(new MotionPlan());
	
	if (std::abs(maxAccel) < 50) throw std::runtime_error("Acceleration must be at least 50 steps per second");
	
	double startVelocity = 0, direction = MathUtils::sgn<double>(targetAngle - startAngle);
	double endVelocity = std::abs(approachVelocity) * direction;
	double coastDistance = MinSpeedControlDistance * direction;
	
	double accel = std::abs(maxAccel)*direction;
	
	PlanSolution sol;
	
	KinematicSolver::calculatePlan(accel, coastDistance,targetTime,(targetAngle-startAngle),startVelocity,endVelocity,sol);
	
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



vector<Step> MotionPlanner::buildMotionSteps(vector<double> jointAngles, Vector3d targetPosition,Matrix3d targetRotation)
{
	vector<Step> steps;
		
	Vector3d currentPosition;
	double rotationMatrix[9];
	
	vector<double> lastAngles = jointAngles;
	
	ComputeFk(lastAngles.data(),currentPosition,rotationMatrix);
	
	Vector3d delta = targetPosition - currentPosition;
	
	vector<double> initialAnglesConv(6);
	for (int c=0;c<6;c++)
	{
		initialAnglesConv[c] = AS5048::radiansToSteps(jointAngles[c]);
	}
	steps.push_back(Step(initialAnglesConv));
	
	cout << "Moving from " << (currentPosition*100.0).toString() << " to " << (targetPosition*100.0).toString() << " in " << pathDivisionCount << " divisions. " << endl;
	for (int i=1;i<=pathDivisionCount;i++)
	{
		Vector3d stepPosition = currentPosition + delta*((double)i/pathDivisionCount);
		
		double stepAngles[6];
		bool solutionExists = getEasiestSolution(lastAngles.data(),stepPosition,targetRotation,stepAngles);
		
		if (!solutionExists) {
			stringstream ss;
			ss << "Solution could not be found for position: " << (stepPosition*100.0).toString() << endl;
			throw std::runtime_error(ss.str());
		}
		
		vector<double> convAngles(6);
		
		for (int c=0;c<6;c++)
		{
			convAngles[c] = AS5048::radiansToSteps(stepAngles[c]);
			lastAngles[c] = stepAngles[c];
		}
		
		
		steps.push_back(Step(convAngles));
	}
	return steps;
}

void MotionPlanner::setPathDivisions(int pathDivisionCount)
{
	this->pathDivisionCount = pathDivisionCount;
}