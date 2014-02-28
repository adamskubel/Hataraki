#include "MotionPlanner.hpp"

using namespace std;
using namespace vmath;
using namespace ikfast2;
using namespace ikfast;


const double DynamicMovementThreshold = AS5048::degreesToSteps(1.5);

MotionPlanner::MotionPlanner(vector<PredictiveJointController*> joints)
{
	this->joints = joints;
	this->interpolationDistance = 5;
	this->firstStepTime = 0.1;
	this->lastStepTime = 0.1;
	this->pathDivisionCount = 20;
	
	vector<double> velocityLimits, accelLimits, jerkLimits;
	
	for (auto it=joints.begin(); it != joints.end(); it++)
	{
		velocityLimits.push_back((*it)->getMaxVelocity());
		accelLimits.push_back((*it)->getMaxAcceleration());
		jerkLimits.push_back(0);
	}
		
	this->pathPlanner = new PathPlanner(6,velocityLimits,accelLimits,jerkLimits);
}

//TODO: Move this to IK planner
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



void MotionPlanner::setPathDivisions(int _pathDivisionCount)
{
	this->pathDivisionCount = _pathDivisionCount;
}


/*
 1. A set of final velocities is stored in the step plan
 2. Recalculate the joint motion plans for minimum time, while meeting final velocity 
 3. If not possible, recursively adjust final velocities for previous step plans
 
 */
void MotionPlanner::calculateStep(vector<StepMotionPlan> * stepPlans, std::vector<Step> & steps, int i)
{
	if (i <= 0) throw std::logic_error("It's all ogre now");

	cout << "Calculating step " << i << endl;

	Step * s0 = &(steps[i]);
	Step * s1 = &(steps[i-1]);

	double stepTime = 0;
	int phase = 0;
	for (int c=0;c<6 && phase < 4;c++)
	{
		PredictiveJointController * joint = joints.at(c);
		double v0 = stepPlans[c].at(i-1).getMotionPlanFinalVelocity();
		double delta = s0->Positions[c] - s1->Positions[c];
		double aMax = joint->getMaxAcceleration();
		double vMax = joint->getMaxVelocity();

		double vF = 0;
		bool enforce = false;
		
		if (stepPlans[c].size() > i)
		{
			vF = stepPlans[c].at(i).finalVelocity;
			enforce = stepPlans[c].at(i).isFinalVelocityEnforced;
		}

		// *** TEMPORAL ANALYSIS PHASE ***
		if (phase == 0)
		{
			double jointTime = 0;
			if (enforce)
			{
				if (!KinematicSolver::threePart_checkTimeInvariantSolutionExists(aMax,v0,vF,delta))
				{
					//No solution exists. Determine the minimum time needed to traverse the distance at max accel (implies adjusted initial speed)
					// d = vF*t + a/2 * t^2
					// solve for t

					double a0 = aMax * sgn(delta);
					
					if (sgn(delta) < 0)
						jointTime = -(vF+sqrt(a0*delta*2.0+vF*vF))/a0;
					else
						jointTime = -(vF-sqrt(a0*delta*2.0+vF*vF))/a0;
					
					if (jointTime > 10 || jointTime < 0)
					{
						stringstream ss;
						ss << "Invalid time 1: " << jointTime;
						throw std::logic_error(ss.str());
					}
				}
				else
				{
					jointTime = KinematicSolver::threePart_minimumTime(aMax,vMax, v0, vF, delta);
					
					if (jointTime > 10 || jointTime < 0)
					{
						stringstream ss;
						ss << "Invalid time 2: " << jointTime;
						throw std::logic_error(ss.str());
					}
				}
			}
			else
			{
				//A time invariant solution always exists in this case
				jointTime = KinematicSolver::twoPart_minimumTime(aMax, vMax, delta, v0);
				
				if (jointTime > 10 || jointTime < 0)
				{
					stringstream ss;
					ss << "Invalid time 3: " << jointTime;
					throw std::logic_error(ss.str());
				}
			}

			jointTime += 0.0001; //Ensure no non-real results due to imprecision errors. Better done with complex math calls.
			
		
			
			stepTime = std::max(stepTime,jointTime);
		}
		// *** VELOCITY ANALYSIS PHASE ***
		else if (phase == 1)
		{
			//If changing direction, last velocity must be zero
			if (sgn(v0) != sgn(delta) && std::abs(v0) > 0)
			{
				stepPlans[c].at(i-1).setFinalVelocity(0);
			}
			//Not changing direction, ensure last velocity is below maximum initial speed
			else
			{
				if (enforce)
				{
					double v0_new = 0;
					if (!KinematicSolver::threePart_attemptSolution(aMax,v0,vF,delta,stepTime,v0_new))
					{						
						stepPlans[c].at(i-1).setFinalVelocity(v0_new);
					}
					else if (stepPlans[c].at(i-1).isFinalVelocityEnforced)
						stepPlans[c].at(i-1).setFinalVelocity(v0);
					
				}
				else
				{
					double v0_new = 0;
					if (!KinematicSolver::twoPart_attemptSolution(aMax,v0,delta,stepTime,v0_new))
					{
						stepPlans[c].at(i-1).setFinalVelocity(v0_new);
					}
					else if (stepPlans[c].at(i-1).isFinalVelocityEnforced)
						stepPlans[c].at(i-1).setFinalVelocity(v0);
				}
			}
		}
		// *** VELOCITY ADJUSTMENT PHASE (one cycle only) ***
		else if (phase == 2)
		{
			if (c != 0) throw std::logic_error("lol");
			//This is a short phase that just calls the historical velocity adjustment

			bool enforcementNeeded = false;
			for (int x=0;x<6;x++)
			{
				enforcementNeeded |= (
									  i > 1 && 
									  stepPlans[x].at(i-1).isFinalVelocityEnforced &&
									  stepPlans[x].at(i-1).getMotionPlanFinalVelocity() != stepPlans[x].at(i-1).finalVelocity);
			}

			if (enforcementNeeded)
			{
				cout << "V0-A = ";
				for (int x=0;x<6;x++)	cout << stepPlans[x].at(i-1).getMotionPlanFinalVelocity() << " ";
				cout << endl;
				
				cout << "V0-B = ";
				for (int x=0;x<6;x++)	cout << ((stepPlans[x].at(i-1).isFinalVelocityEnforced) ? stepPlans[x].at(i-1).finalVelocity : 0.1) << " ";
				cout << endl;
				
				c = -1; phase = 0;
				calculateStep(stepPlans, steps, i-1);
				//After this call, final velocities will meet specifications (they fucking better)
				//Recalculate this step to ensure ALL non-updated channels still have valid solutions
				
				
				cout << "V0-B2 = ";
				for (int x=0;x<6;x++)	cout << stepPlans[x].at(i-1).getMotionPlanFinalVelocity() << " ";
				cout << endl;
			}
			else
			{
				c = -1; phase = 3;
			}
		}
		// *** FINAL CALCULATION PHASE ***
		else if (phase == 3)
		{
			PlanSolution sol;
			vector<MotionInterval> stepIntervals;			

			if (enforce)
			{
				KinematicSolver::threePart_calculate(aMax, v0, vF, delta, stepTime, sol);

				if (sol.t0 >= samplePeriod) stepIntervals.push_back(MotionInterval(v0,sol.v1,sol.t0));
				if (sol.t1 >= samplePeriod) stepIntervals.push_back(MotionInterval(sol.v1,sol.t1));
				if (sol.t2 >= samplePeriod) stepIntervals.push_back(MotionInterval(sol.v1,vF,sol.t2));
			}
			else
			{
				KinematicSolver::twoPart_calculate(aMax, v0, delta, stepTime, sol);

				if (sol.t0 >= samplePeriod) stepIntervals.push_back(MotionInterval(v0,sol.v1,sol.t0));
				if (sol.t1 >= samplePeriod) stepIntervals.push_back(MotionInterval(sol.v1,sol.t1));
			}
			
			if (stepPlans[c].size() > i)
				stepPlans[c].at(i).intervals = stepIntervals;
			else
				stepPlans[c].push_back(StepMotionPlan(stepIntervals));
			
		}

		if (c == 5)
		{
			phase++;
			c = -1;
		}
	}
}

vector<shared_ptr<MotionPlan> > MotionPlanner::compileStepMotionPlans(vector<StepMotionPlan> * stepPlans)
{
	vector<shared_ptr<MotionPlan> > motionPlan;

	for (int c=0;c<6;c++)
	{
		motionPlan.push_back(shared_ptr<MotionPlan>(new MotionPlan()));

		auto mi = stepPlans[c];
		for (auto it=mi.begin();it!=mi.end();it++)
			for (auto it2=it->intervals.begin(); it2 != it->intervals.end(); it2++)
				motionPlan.back()->motionIntervals.push_back(*it2);

	}


	return motionPlan;
}

vector<shared_ptr<MotionPlan> > MotionPlanner::buildPlan(vector<Step> & steps)
{
	auto stepMotionPlans = new vector<StepMotionPlan>[6];
	
	for (int c=0;c<6;c++)
	{
		stepMotionPlans[c].push_back(StepMotionPlan());
	}
	
	try
	{
		for (int i=1;i<steps.size();i++)	
		{
			calculateStep(stepMotionPlans,steps,i);
		}
	
		auto motionPlan = compileStepMotionPlans(stepMotionPlans);
	
		for (int c=0; c<6; c++)
		{
			motionPlan.at(c)->startAngle = steps.front().Positions[c];
			motionPlan.at(c)->finalAngle = steps.back().Positions[c];
		}
		return motionPlan;
	}
	catch (std::logic_error & le)
	{
		throw runtime_error("Cannot build plan. Error = " + le.what());
	}
}


//TODO: Move this to IK/Trajectory planner
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

//TODO: Move this to IK/Trajectory planner
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



shared_ptr<MotionPlan> MotionPlanner::buildMotionPlan(const double startAngle,const  double targetAngle, const double targetTime, const  double approachVelocity, const double maxAccel)
{
	const double MinSpeedControlDistance = 150;
	auto plan = shared_ptr<MotionPlan>(new MotionPlan());
	
	if (std::abs(maxAccel) < 50) throw std::runtime_error("Acceleration must be at least 50 steps per second");
	
	double startVelocity = 0, direction = sgn(targetAngle - startAngle);
	double endVelocity = std::abs(approachVelocity) * direction;
	double coastDistance = MinSpeedControlDistance * direction;
	
	double accel = std::abs(maxAccel)*direction;
	
	PlanSolution sol;
	
	KinematicSolver::fourPart_calculate(accel, coastDistance,targetTime,(targetAngle-startAngle),startVelocity,endVelocity,sol);
	
	if (sol.status != PlanSolution::SolutionStatus::OriginalSolution)
	{
		stringstream ss;
		ss << "No valid solution found.";
		throw std::runtime_error(ss.str());
	}
	
	plan->motionIntervals.push_back(MotionInterval(startVelocity,sol.v1,sol.t0));
	
	if (sol.t1 > 0)
		plan->motionIntervals.push_back(MotionInterval(sol.v1,sol.t1));
	
	plan->motionIntervals.push_back(MotionInterval(sol.v1,endVelocity,sol.t2));
	plan->motionIntervals.push_back(MotionInterval(endVelocity,sol.t3));
	
	plan->startAngle = startAngle;
	plan->finalAngle = targetAngle;
	
	cout << "V0=" << sol.v1 << " T0=" << sol.t0 << " T1=" << sol.t1 << " T2=" << sol.t2 << " T3=" << sol.t3 <<endl;
	cout << "FinalAngle=" << AS5048::stepsToDegrees(plan->getPositionAtTime(1000)) << endl;
	
	return plan;
}



//TODO: Move this to IK planner
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


vector<shared_ptr<MotionPlan> > MotionPlanner::createClosedSolutionMotionPlanFromSteps(vector<Step> & steps)
{
	const double CoastDistance = 150;
	
	vector<shared_ptr<MotionPlan> > motionPlan;

	for (int i=0;i<6;i++) 
	{
		motionPlan.push_back(shared_ptr<MotionPlan>(new MotionPlan()));
	}

	if (steps.size() != 2) throw std::runtime_error("Only a single interval (comprised of two steps) is allowed for this method");
	

	Step * step = &(steps.back());
	Step * s1 = &(steps.front());
	
	double stepTime = 0;
	for (int i=0;i<6;i++)
	{						
		double lastVelocity = 0;

		double delta = step->Positions[i] - s1->Positions[i];
		cout << delta << " ";

		double direction = sgn(delta);
		double maxSpeed = joints.at(i)->getMaxVelocity() * direction;
		double maxAccel = joints.at(i)->getMaxAcceleration();
		double coastVelocity = joints.at(i)->getJointModel()->servoModel.controllerConfig.approachVelocity*direction;

		double v1,jointTime;
		jointTime = KinematicSolver::fourPart_minimumTime(maxAccel,CoastDistance * direction,delta,lastVelocity,coastVelocity,maxSpeed,v1);
		jointTime += 0.0001; //Ensure no non-real results due to imprecision errors

		stepTime = std::max(stepTime,jointTime);
	}
	cout << endl;

	for (int i=0;i<6;i++) 
	{	
		double lastVelocity = 0;		
		double delta = step->Positions[i] - s1->Positions[i];
		double direction = sgn(delta);
		double coastVelocity = joints.at(i)->getJointModel()->servoModel.controllerConfig.approachVelocity*direction;
		double maxAccel = joints.at(i)->getMaxAcceleration();

		PlanSolution sol;
		KinematicSolver::fourPart_calculate(maxAccel,CoastDistance*direction,stepTime,delta,lastVelocity,coastVelocity,sol);
		
		motionPlan.at(i)->motionIntervals.push_back(MotionInterval(lastVelocity,sol.v1,sol.t0));

		if (sol.t1 >= samplePeriod) 
		{
			motionPlan.at(i)->motionIntervals.push_back(MotionInterval(sol.v1,sol.t1));					
		}

		motionPlan.at(i)->motionIntervals.push_back(MotionInterval(sol.v1,coastVelocity,sol.t2));
		motionPlan.at(i)->motionIntervals.push_back(MotionInterval(coastVelocity,sol.t3));

		cout << sol.v1 << " ;";
	}
	cout << endl;

	for (int i=0;i<6;i++)
	{
		motionPlan.at(i)->startAngle = steps.front().Positions[i];
		motionPlan.at(i)->finalAngle = steps.back().Positions[i];
	}

	return motionPlan;
}


/*
 Ensure the final velocity of the specified step is equal to the specified velocity
 
 0. Check that joint will not overshoot given new final velocity, and final velocity of last step.
 
 A. Overshoot
 
 1. Calculate maximum initial velocity, and call this method on the previous step.
 2. Go to B-1
 *Note - If the current step is the first step, condition A should never be reached assuming joint starts from rest.
 
 B. No overshoot
 
 1. Determine minimum time for current step using provided final velocity
 2. If minimum time exceeds step time, then recalculate all other joints
 3. If not, recalculate current step only, using the same step time
 
 
 void MotionPlanner::enforceFinalVelocity2(vector<StepMotionPlan> * stepPlans, std::vector<Step> & steps, int i, int channel, double finalVelocity)
 {
 if (i == 0)
 {
 throw std::runtime_error("It's all ogre");
 }
 
 Step * s0 = &(steps[i]);
 Step * s1 = &(steps[i-1]);
 
 double stepTime = s0->TimeOffset; //Make sure you set this!
 int c = channel;
 
 PredictiveJointController * joint = joints.at(c);
 double lastVelocity = (i>1) ? stepPlans[c].back().intervals.back().endSpeed : 0;
 double delta = s0->Positions[c] - s1->Positions[c];
 double dir = sgn(delta);
 
 //D = (v0*t)+0.5*a*t^2
 //t = (v1-v0)/a
 int solutionType = 0;
 double jointTime = 0, maxInitial = 0;
 double minTime = std::abs((finalVelocity - lastVelocity)/joint->getMaxAcceleration());
 double minDist = lastVelocity*minTime + 0.5*dir*joint->getMaxAcceleration()*std::pow(minTime,2);
 
 //Driving joint
 if (minTime > stepTime)
 {
 solutionType = 1;
 if (std::abs(minDist) > std::abs(delta))
 {
 maxInitial = 1; //lol
 enforceFinalVelocity2(stepPlans, steps, i-1, c, maxInitial);
 
 lastVelocity = (i>1) ? stepPlans[c].back().intervals.back().endSpeed : 0;
 //If velocity has ended up slower than requested, use a three part plan
 if (lastVelocity != maxInitial)
 {
 cout << "Slower!" << endl;
 solutionType = 3;
 }
 }
 else
 {
 throw std::logic_error("Unpossible.");
 }
 }
 //Not the driving joint, use a three part plan
 else
 {
 solutionType = 3;
 if (std::abs(minDist) > std::abs(delta))
 {
 throw std::logic_error("Unpossible.");
 }
 }
 
 if (solutionType == 1)
 {
 jointTime = std::abs((finalVelocity - lastVelocity)/joint->getMaxAcceleration());
 
 //intervals[c].at(i).clear();
 //intervals[c].at(i).push_back(MotionInterval(lastVelocity,finalVelocity,jointTime));
 
 //Recalculate all joints
 int phase = 0;
 for (c=0;c<6 && phase < 1;c++)
 {
 
 }
 }
 else if (solutionType == 3)
 {
 //This should be equal to or less than step time
 jointTime = KinematicSolver::planThree_minimumTime(joint->getMaxAcceleration(), delta, lastVelocity,finalVelocity);
 
 if (jointTime > stepTime)
 {
 throw std::logic_error("Unpossible.");
 }
 
 //build plan using current step time
 PlanSolution sol;
 KinematicSolver::planThree_calculate(joint->getMaxAcceleration(), delta, stepTime, lastVelocity, finalVelocity, sol);
 }
 else
 {
 throw std::runtime_error("Invalid solution type");
 }
 }
 */

/*
 
 1. For each step, determine the optimal time given input speed and distance.
 2. The joint with the largest optimal time is the "driving" joint
 3a. Check if any joint will overshoot using the driving time. Condition: tD*v0 - 0.5*A*tD^2 > D_target
 3b. If no overshoot is found, go to 4a. Otherwise, go to 4b.
 4a. Using the driving time determine the t0, t1, and v0. D(t) = t0*v0 + 0.5*(v1-v0)*t^2
 5a. Go to 2
 
 4b. If an overshoot is found, determine v0 such that: tD*v0 - 0.5A*tD^2 == D_target
 5b. Recalculate the previous step using this maximum speed.
 6. If this results in overshoot, recalculate the next previous. Repeat until first interval is reached or overshoot is gone.
 7. Go to 2.
 
 - Set target final v for step i-1
 - Recalc i-1 for all joints
 
 
 */

//vector<shared_ptr<MotionPlan> > MotionPlanner::buildPlan(vector<Step> & steps)
//{
//	auto stepPlans = new vector<StepMotionPlan>[6];
//
//	for (int i=1;i<steps.size();i++)
//	{
//		Step * s0 = &(steps[i]);
//		Step * s1 = &(steps[i-1]);
//
//		double stepTime = 0;
//		int phase = 0;
//		for (int c=0;c<6 && phase < 4;c++)
//		{
//			PredictiveJointController * joint = joints.at(c);
//			double lastVelocity = (i>1) ? stepPlans[c].back().intervals.back().endSpeed : 0;
//			double delta = s0->Positions[c] - s1->Positions[c];
//
//			if (phase == 0)
//			{
//				double jointTime = KinematicSolver::calculateMinimumTime(joint->getMaxAcceleration(), delta, lastVelocity, joint->getMaxVelocity());
//				jointTime += 0.0001; //Ensure no non-real results due to imprecision errors
//
//				stepTime = std::max(stepTime,jointTime);
//			}
//			else if (phase == 1)
//			{
//				//If changing direction, last velocity must be zero
//				if (sgn(lastVelocity) != sgn(delta) && std::abs(lastVelocity) > 0)
//				{
//					stepPlans[c].back().setFinalVelocity(0);
//				}
//				//Not changing direction, ensure last velocity is below maximum initial speed
//				else
//				{
//					double maxInitial = KinematicSolver::calculateMaximumInitialSpeed(joint->getMaxAcceleration(), delta, joint->getMaxVelocity());
//					//If signs aren't equal, there's no problem.
//					if (sgn(maxInitial) == sgn(lastVelocity) && std::abs(lastVelocity) > std::abs(maxInitial))
//					{
//						stepPlans[c].back().setFinalVelocity(maxInitial);
//					}
//				}
//			}
//			else if (phase == 2)
//			{
//				if (c != 0) throw std::logic_error("lol");
//				//This is a short phase that just calls the historical velocity adjustment
//
//				bool enforcementNeeded = false;
//				for (int x=0;x<6;x++) enforcementNeeded |= stepPlans[x].back().isFinalVelocityEnforced;
//
//				c = -1;
//				if (enforcementNeeded)
//				{
//					phase = 0;
//					enforceFinalVelocity(stepPlans, steps, i);
//				}
//				else
//					phase = 3;
//			}
//			else if (phase == 3)
//			{
//				PlanSolution sol;
//				KinematicSolver::calculatePlan2Part(joint->getMaxAcceleration(), stepTime, delta, lastVelocity, sol);
//
//				vector<MotionInterval> stepIntervals;
//				stepIntervals.push_back(MotionInterval(lastVelocity,sol.v1,sol.t0));
//
//				if (sol.t1 >= samplePeriod)
//				{
//					stepIntervals.push_back(MotionInterval(sol.v1,sol.t1));
//				}
//
//				stepPlans[c].push_back(StepMotionPlan(stepIntervals));
//			}
//
//			if (c == 5)
//			{
//				phase++;
//				c = -1;
//			}
//		}
//	}
//
//	vector<shared_ptr<MotionPlan> > motionPlan;
//
//	for (int c=0;c<6;c++)
//	{
//		motionPlan.push_back(shared_ptr<MotionPlan>(new MotionPlan()));
//
//		auto mi = stepPlans[c];
//		for (auto it=mi.begin();it!=mi.end();it++)
//			for (auto it2=it->begin(); it2 != it->end(); it2++)
//				motionPlan.back()->motionIntervals.push_back(*it2);
//
//		motionPlan.back()->startAngle = steps.front().Positions[c];
//		motionPlan.back()->finalAngle = steps.back().Positions[c];
//	}
//
//
//	return motionPlan;
//}

/*
 
 vector<shared_ptr<MotionPlan> > MotionPlanner::convertStepsToPlans(std::vector<Step> & steps, vector<double> initialAngles)
 {
 
 vector<shared_ptr<MotionPlan> > plans;
 
 int numChannels = 6;
 
 
 bool planDynamic[6];
 
 for (int c=0;c<numChannels;c++)
 {
 auto plan = shared_ptr<MotionPlan>(new MotionPlan());
 plan->startAngle = initialAngles[c];
 plans.push_back(plan);
 plan->finalAngle = steps.back().Positions[c];
 
 planDynamic[c] = (std::abs(plan->finalAngle - plan->startAngle) > DynamicMovementThreshold);
 }
 
 double totalDuration = 0;
 for (int i=1;i < steps.size(); i++)
 {
 Step * s0 = &steps[i];
 Step * s1 = &steps[i-1];
 
 for (int c=0;c<numChannels;c++)
 {
 if (!planDynamic[c]) continue;
 
 double t0 = s0->TimeOffset;
 double t1 = s1->TimeOffset;
 
 double t1_0 = 0.5*(t0+t1);
 
 double lastVelocity = 0;
 if (plans[c]->motionIntervals.size() > 0)
 lastVelocity = plans[c]->motionIntervals.back().endSpeed;
 
 plans[c]->motionIntervals.push_back(MotionInterval(lastVelocity,s0->Velocities[c],t1_0));
 totalDuration = std::max(totalDuration,plans[c]->getPlanDuration());
 }
 }
 
 for (int c=0;c<numChannels;c++)
 {
 if (!planDynamic[c])
 {
 auto plan = plans.at(c);
 double constSpeed = (plan->finalAngle - plan->startAngle)/totalDuration;
 plan->motionIntervals.push_back(MotionInterval(constSpeed,totalDuration));
 }
 }
 
 return plans;
 }
 */


/*
 vector<Step> MotionPlanner::realizeSteps(std::vector<Step> & steps)
 {
 vector<Step> initial = steps;
 
 const int numChannels = 6;
 
 double finalAngles[6];
 
 for (int c=0;c<numChannels;c++)
 {
 if (std::abs(initial.back().Positions[c] - initial.front().Positions[c]) < DynamicMovementThreshold)
 {
 finalAngles[c] = initial.back().Positions[c];
 for (auto it=initial.begin(); it != initial.end(); it++)
 it->Positions[c] = finalAngles[c];
 }
 }
 
 
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
 */
