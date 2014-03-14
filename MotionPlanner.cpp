#include "MotionPlanner.hpp"

using namespace std;
using namespace vmath;
using namespace ikfast2;
using namespace ikfast;

#define MotionPlanDebug false

const double DynamicMovementThreshold = AS5048::degreesToSteps(1.5);

MotionPlanner::MotionPlanner(vector<PredictiveJointController*> joints)
{
	this->joints = joints;

	this->pathInterpolationDistance =1.0;
	this->pathDivisionCount = 1;
	this->pathInterpolationMode = PathInterpolationMode::SingleStep;
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

void MotionPlanner::setPathInterpolationDistance(double pathInterpolationDistance)
{
	this->pathInterpolationDistance = pathInterpolationDistance;
}

void MotionPlanner::setPathInterpolationMode(PathInterpolationMode pathInterpolationMode)
{
	this->pathInterpolationMode = pathInterpolationMode;
}

/*
 1. A set of final velocities is stored in the step plan
 2. Recalculate the joint motion plans for minimum time, while meeting final velocity 
 3. If not possible, recursively adjust final velocities for previous step plans
 
 */
void MotionPlanner::calculateStep(vector<StepMotionPlan> * stepPlans, std::vector<Step> & steps, int i)
{
	if (i <= 0) throw std::logic_error("It's all ogre now");

	if (MotionPlanDebug)
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

		if (abs(delta) < 0.001) delta = 0;
		if (abs(v0) < 0.001) v0 = 0;

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
					// No solution exists. Determine the minimum time needed to traverse the distance at max accel (implies adjusted initial speed)
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
				if (delta == 0 && v0 != 0)
					jointTime = 0;
				else
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
				if (MotionPlanDebug)
				{
					cout << "V0-A = ";
					for (int x=0;x<6;x++)	cout << stepPlans[x].at(i-1).getMotionPlanFinalVelocity() << " ";
					cout << endl;
					
					cout << "V0-B = ";
					for (int x=0;x<6;x++)	cout << ((stepPlans[x].at(i-1).isFinalVelocityEnforced) ? stepPlans[x].at(i-1).finalVelocity : 0.1) << " ";
					cout << endl;
				}
				
				c = -1; phase = 0;
				calculateStep(stepPlans, steps, i-1);
				//After this call, final velocities will meet specifications (they fucking better)
				//Recalculate this step to ensure ALL non-updated channels still have valid solutions
				
				if (MotionPlanDebug)
				{
					cout << "V0-B2 = ";
					for (int x=0;x<6;x++)	cout << stepPlans[x].at(i-1).getMotionPlanFinalVelocity() << " ";
					cout << endl;
				}
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
			double minTime = -1;
			
			if (enforce)
			{
				KinematicSolver::threePart_calculate(aMax, v0, vF, delta, stepTime, sol);

				if (sol.t0 >= minTime) stepIntervals.push_back(MotionInterval(v0,sol.v1,sol.t0));
				if (sol.t1 >= minTime) stepIntervals.push_back(MotionInterval(sol.v1,sol.t1));
				if (sol.t2 >= minTime) stepIntervals.push_back(MotionInterval(sol.v1,vF,sol.t2));
			}
			else
			{
				KinematicSolver::twoPart_calculate(aMax, v0, delta, stepTime, sol);

				if (sol.t0 >= minTime) stepIntervals.push_back(MotionInterval(v0,sol.v1,sol.t0));
				if (sol.t1 >= minTime) stepIntervals.push_back(MotionInterval(sol.v1,sol.t1));
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

vector<shared_ptr<MotionPlan> > MotionPlanner::compileStepMotionPlans(vector<StepMotionPlan> * stepPlans, vector<Step> & steps)
{
	vector<shared_ptr<MotionPlan> > motionPlan;

	for (int c=0;c<6;c++)
	{
		MotionPlan * plan = new MotionPlan();		
		
		plan->startAngle = steps.front().Positions[c];
		plan->finalAngle = steps.back().Positions[c];

		int s= 0;
		auto mi = stepPlans[c];
		for (auto it=mi.begin();it!=mi.end();it++,s++)
		{
			for (auto it2=it->intervals.begin(); it2 != it->intervals.end(); it2++)
				plan->motionIntervals.push_back(*it2);
			
			plan->markKeyframe(steps[s].Positions[c]);
		}
		
		if (plan->motionIntervals.empty())
			throw std::logic_error("Empty plan created");
		
		motionPlan.push_back(shared_ptr<MotionPlan>(plan));
	}

	return motionPlan;
}

vector<shared_ptr<MotionPlan> > MotionPlanner::buildPlan(vector<Step> & steps)
{
	const bool startIntervalEnabled = !true;
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
	
		auto motionPlan = compileStepMotionPlans(stepMotionPlans,steps);
		delete [] stepMotionPlans;
	
		for (int c=0; c<6; c++)
		{
			if (startIntervalEnabled)
			{
				double startSpeed = motionPlan.at(c)->motionIntervals.front().endSpeed;
				double startTime = 0.04; //yay hardcoding
			
				MotionInterval mi(startSpeed,startTime);
				mi.action = MotionInterval::Action::Start;
			
				motionPlan.at(c)->motionIntervals.insert(motionPlan.at(c)->motionIntervals.begin(),mi);
			}
		} 
		
		return motionPlan;
	}
	catch (std::runtime_error & e)
	{
		stringstream ss;
		ss << "Cannot build plan. Error = " << e.what();
		throw runtime_error(ss.str());
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
			stringstream ss;
			ss << "Unexpected value for total effort: " << totalDiff << "rad . Max effort = " << maxDiff << " rad";
			throw std::logic_error(ss.str());
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

shared_ptr<MotionPlan> MotionPlanner::buildOptimalMotionPlan(int jointIndex, const double targetAngle)
{
	if (jointIndex < 0 || jointIndex >= joints.size()) throw std::runtime_error("Invalid joint index");

	auto joint = joints.at(jointIndex);	

	double v0 = 0;
	double startAngle = joint->getCurrentAngle();
	double delta = targetAngle - startAngle;
	double dir = sgn(delta);
	double vF = joint->getJointModel()->servoModel.controllerConfig.approachVelocity * dir;
	
	double minTime = KinematicSolver::threePart_minimumTime(joint->getMaxAcceleration(),joint->getMaxVelocity(),v0,vF,delta);
	minTime += 0.0001;

	if (isnan(minTime) || minTime < 0)
		throw std::runtime_error("Cannot set joint position (negative/complex minimum time)");
	
	PlanSolution sol;

	KinematicSolver::threePart_calculate(joint->getMaxAcceleration(),v0,vF,delta,minTime,sol);

	if (sol.status != PlanSolution::SolutionStatus::OriginalSolution)
	{
		stringstream ss;
		ss << "No valid solution found.";
		throw std::runtime_error(ss.str());
	}
	
	//Add to plan
	
	auto plan = shared_ptr<MotionPlan>(new MotionPlan());

	if (sol.t0 > 0) plan->motionIntervals.push_back(MotionInterval(v0,sol.v1,sol.t0));	
	if (sol.t1 > 0) plan->motionIntervals.push_back(MotionInterval(sol.v1,sol.t1));	
	if (sol.t2 > 0) plan->motionIntervals.push_back(MotionInterval(sol.v1,vF,sol.t2));

	plan->startAngle = startAngle;
	plan->finalAngle = targetAngle;

	double planError = abs(plan->finalAngle - plan->x(plan->getPlanDuration()));
	if (planError > 10.0)
	{
		stringstream ss;
		ss << "Plan error was too high: " << planError;
		throw std::runtime_error(ss.str());
	}
		
	return plan;
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
	
	//cout << "V0=" << sol.v1 << " T0=" << sol.t0 << " T1=" << sol.t1 << " T2=" << sol.t2 << " T3=" << sol.t3 <<endl;
	//cout << "FinalAngle=" << AS5048::stepsToDegrees(plan->x(1000)) << endl;
	
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
	
	int numDivisions;
	switch (pathInterpolationMode)
	{
	case PathInterpolationMode::FixedStepCount:
		numDivisions = pathDivisionCount;
		break;
	case PathInterpolationMode::FixedStepDistance:
		numDivisions = std::round(delta.length()/pathInterpolationDistance);
		break;
	case PathInterpolationMode::SingleStep:
	default:
		numDivisions = 1;
		break;
	}

	for (int i=1;i<=numDivisions;i++)
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
		//cout << delta << " ";

		double direction = sgn(delta);
		double maxSpeed = joints.at(i)->getMaxVelocity() * direction;
		double maxAccel = joints.at(i)->getMaxAcceleration();
		double coastVelocity = joints.at(i)->getJointModel()->servoModel.controllerConfig.approachVelocity*direction;

		double v1,jointTime;
		jointTime = KinematicSolver::fourPart_minimumTime(maxAccel,CoastDistance * direction,delta,lastVelocity,coastVelocity,maxSpeed,v1);
		jointTime += 0.0001; //Ensure no non-real results due to imprecision errors

		stepTime = std::max(stepTime,jointTime);
	}
	//cout << endl;

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

		if (sol.t1 >= Configuration::SamplePeriod) 
		{
			motionPlan.at(i)->motionIntervals.push_back(MotionInterval(sol.v1,sol.t1));					
		}

		motionPlan.at(i)->motionIntervals.push_back(MotionInterval(sol.v1,coastVelocity,sol.t2));
		motionPlan.at(i)->motionIntervals.push_back(MotionInterval(coastVelocity,sol.t3));

		//cout << sol.v1 << " ;";
	}
	//cout << endl;

	for (int i=0;i<6;i++)
	{
		motionPlan.at(i)->startAngle = steps.front().Positions[i];
		motionPlan.at(i)->finalAngle = steps.back().Positions[i];
	}

	return motionPlan;
}

