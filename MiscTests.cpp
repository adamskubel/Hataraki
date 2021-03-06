

/*
 //testServoModel(&(controllers.at(0)->getJointModel()->servoModel));
 //testArmModel(armModel);
 
 //double angles[] = {0,0.1,0,0.1,0.1,0};
 //testPoseDynamics(armModel,angles);
 //double angles2[] = {0,15,0,15,15,0};
 //testPoseDynamics(armModel,angles2);
 //testCsvWriteTime();
 //testFK();
 
 testOptimalPlanBuilding();
 testOptimalPlanBuilding2();
 testCompletePlanBuilding();
 */
/*
void testCsvWriteTime()
{	
	std::ofstream csvLog;
	csvLog.open("TestFile.csv");
	timespec start;
	MathUtil::setNow(start);
	for (int i=0;i<100;i++)
	{
		csvLog <<
			"Time"				<< Configuration::CsvSeparator <<
			"JointStatus"		<< Configuration::CsvSeparator <<
			"RawSensorAngle"	<< Configuration::CsvSeparator <<
			"SensorAngle"		<< Configuration::CsvSeparator <<
			"TargetAngle"		<< Configuration::CsvSeparator <<
			"Velocity"			<< Configuration::CsvSeparator <<
			"VelocityR2"		<< Configuration::CsvSeparator << 
			"TargetVelocity"	<< Configuration::CsvSeparator <<
			"DisturbanceTorque" << Configuration::CsvSeparator <<
			"MotorTorque"		<< Configuration::CsvSeparator <<
			"EffectiveVoltage"	<< Configuration::CsvSeparator <<
			"AverageVoltaged"	<< Configuration::CsvSeparator <<
			"AppliedVoltage"	<< Configuration::CsvSeparator <<
			"ControlMode"		<< Configuration::CsvSeparator <<
			"SecondaryState"	<< Configuration::CsvSeparator <<
			"SensorWriteTime"	<< Configuration::CsvSeparator <<
			"SensorReadTime"	<< Configuration::CsvSeparator <<
			"DriverWriteTime"	<< Configuration::CsvSeparator << endl;
	}
	cout << "Average write time = " << MathUtil::timeSince(start)*10.0 << " ms." << endl;
}

void testServoModel(ServoModel * sm)
{	
	double torque = sm->getTorqueForVoltageSpeed(1.2,1000);
	cout << "V=1.2, S=1000, T=" << torque << endl;
	cout << "V=1.2, T=" << torque << ", S=" << sm->getSpeedForTorqueVoltage(torque,1.2) << endl;
	cout << "T=" << torque << ", S=1000, V=" << sm->getVoltageForTorqueSpeed(torque,1000) << endl;
	
	cout << endl;
	torque = sm->getTorqueForVoltageSpeed(-1.2,-1000);
	cout << "V=-1.2, S=-1000, T=" << torque << endl;
	cout << "V=-1.2, T=" << torque << ", S=" << sm->getSpeedForTorqueVoltage(torque,-1.2) << endl;
	cout << "T=" << torque << ", S=-1000, V=" << sm->getVoltageForTorqueSpeed(torque,-1000) << endl;

	cout << endl;
	cout << "V=-0.19, S=0, T=" << sm->getTorqueForVoltageSpeed(-0.19,0) << endl;
		
	cout << endl;
	cout << "V=0.56, T=0, S=" << sm->getSpeedForTorqueVoltage(0,0.56) << endl;
}

void testArmModel(ArmModel * armModel)
{
	for (int i=0;i<6;i++)
	{
		SegmentModel * segment = &(armModel->segments[i]);
		JointModel * joint = &(armModel->joints[i]);

		cout << "Segment[" << i << "]: Mass = " << segment->mass << endl;
		cout << "Joint[" << i << "]: Axis = " << joint->axisOfRotation.toString() << endl;
		cout << endl;
	}
}


void testPoseDynamics(ArmModel * armModel, double * angles)
{
	PoseDynamics::getInstance().setArmModel(armModel);
		
	vector<double> jointAngles;
	for (int i=0;i<6;i++)
	{
		jointAngles.push_back(MathUtil::degreesToRadians(angles[i]));
	}

	PoseDynamics::getInstance().setJointAngles(jointAngles);

	timespec startTime;
	MathUtil::setNow(startTime);

	PoseDynamics::getInstance().update();

	cout << "Update took " << MathUtil::timeSince(startTime)*1000.0 << " ms." << endl;
	
	MathUtil::setNow(startTime);

	vector<double> torques;
	for (int i=0;i<6;i++)
	{
		torques.push_back(PoseDynamics::getInstance().computeJointTorque(i));
	}
	cout << "All computes took " << MathUtil::timeSince(startTime)*1000.0 << " ms." << endl;

	for (int i=0;i<6;i++)
	{
		//cout << "Angle       [" << i << "]\t= " << MathUtil::radiansToDegrees(PoseDynamics::getInstance().jointAngles[i]) << endl;
		cout << "Torque      [" << i << "]\t= " << torques[i] << endl;
		//cout << "ChildMass   [" << i << "]\t= " << PoseDynamics::getInstance().childPointMassValue[i] << endl;
		//cout << "ChildMassPos[" << i << "]\t= " << PoseDynamics::getInstance().childPointMassPosition[i].toString() << endl;
		//cout << "SegmentPos  [" << i << "]\t= " << PoseDynamics::getInstance().segmentTransforms[i].Translation.toString() << endl;
		cout << endl;
	}



	cout << endl;
}


void printPositionForAngles(double * jointAngles) {
	
	double radAngles[6];

	for (int i=0;i<6;i++)
	{
		radAngles[i] = MathUtil::degreesToRadians(jointAngles[i]);
	}

	IkReal translationMatrix[3];
	IkReal rotationMatrix[9];
	
	ComputeFk(radAngles,translationMatrix,rotationMatrix);
	
	cout << "Endpoint position is " << translationMatrix[0]*100.0 << "," << translationMatrix[1]*100.0 << "," << translationMatrix[2]*100.0 <<endl;
	cout << "Rotation matrix is: ";
	for (int i=0;i<9;i++)
	{
		cout << rotationMatrix[i] << " ";
	}
	cout << endl;
}

void testFK()
{	
	IkReal jointAngles[6] = {0,0,0,0,0,0};
	printPositionForAngles(jointAngles);
	IkReal jointAngles2[6] = {1,20,0,13,88,90};
	printPositionForAngles(jointAngles2);
}

void testMotionPlan()
{
	MotionPlan * plan = new MotionPlan();
	plan->motionIntervals.push_back(MotionInterval(10,2));

	cout << "Constant speed plan, V=10, Duration=2:" << endl;
	cout << "Speed at time(0) = " << plan->getSpeedAtTime(0) << endl;
	cout << "Speed at time(1) = " << plan->getSpeedAtTime(1) << endl;
	cout << "Speed at time(2.1) = " << plan->getSpeedAtTime(2.1) << endl;
	cout << "Position at time(0) = " << plan->getPositionAtTime(0) << endl;
	cout << "Position at time(1.5) = " << plan->getPositionAtTime(1.5) << endl;
	cout << "Position at time(2.0) = " << plan->getPositionAtTime(2) << endl;
	cout << "Position at time(2.1) = " << plan->getPositionAtTime(2.1) << endl;
	delete plan;

	plan = new MotionPlan();
	plan->motionIntervals.push_back(MotionInterval(0,10,2));
	plan->motionIntervals.push_back(MotionInterval(10,10,2));
	plan->startAngle = 1;

	cout << endl;
	cout << "Constant accel plan:" << endl;
	cout << "Plan duration: " << plan->getPlanDuration() << endl;
	plan->startNow();
	cout << "Plan ends in : " << TimeUtil::timeUntil(plan->endTime) << endl;
	cout << "Plan ended at : " << TimeUtil::timeSince(plan->endTime) << endl;

	//cout << "Speed at time(0) = " << plan->getSpeedAtTime(0) << endl;
	//cout << "Speed at time(1) = " << plan->getSpeedAtTime(1) << endl;
	//cout << "Speed at time(2) = " << plan->getSpeedAtTime(2) << endl;
	//cout << "Speed at time(2.1) = " << plan->getSpeedAtTime(2.1) << endl;

	//cout << "Position at time(0) = " << plan->getPositionAtTime(0) << endl;
	//cout << "Position at time(1) = " << plan->getPositionAtTime(1) << endl;
	//cout << "Position at time(2) = " << plan->getPositionAtTime(2) << endl;
	//cout << "Position at time(2.1) = " << plan->getPositionAtTime(2.1) << endl;
	//cout << "Position at time(4) = " << plan->getPositionAtTime(4) << endl;
	//cout << "Position at time(4.1) = " << plan->getPositionAtTime(4.1) << endl;
	//cout << "Position at time(50) = " << plan->getPositionAtTime(50) << endl;

}

void testPlanBuilding()
{
	double startAngle = AS5048::degreesToSteps(64), targetAngle = AS5048::degreesToSteps(32);
	timespec start;
	TimeUtil::setNow(start);

	double accel = 2000, approachSpeed = -500, approachDist = -150;
	double maxSpeed = 1000, s;

	double targetTime = MotionController::optimalSpeed(accel,approachDist,(targetAngle-startAngle),0,approachSpeed,maxSpeed,s);

	cout << "Minimum time = " << targetTime << "s, OptSpeed=" << s << " steps/s" << endl;

	targetTime += 2;

	auto plan = MotionController::buildMotionPlan(startAngle,targetAngle,targetTime,approachSpeed,accel);
	
	cout << "Plan building took: " << TimeUtil::timeSince(start)*1000.0 << " ms" << endl;

	cout << "Speed at time(0) = " << plan->getSpeedAtTime(0) << endl;
	cout << "Speed at time(1) = " << plan->getSpeedAtTime(1) << endl;
	cout << "Speed at time(2) = " << plan->getSpeedAtTime(2) << endl;
	cout << "Speed at time(2.1) = " << plan->getSpeedAtTime(2.1) << endl;
	cout << "Speed at time(3.1) = " << plan->getSpeedAtTime(3.1) << endl;
	cout << "Speed at time(4) = " << plan->getSpeedAtTime(4) << endl;
	cout << "Speed at time(4.1) = " << plan->getSpeedAtTime(4.1) << endl;

	cout << "Position at time(0) = " << plan->getPositionAtTime(0) << endl;
	cout << "Position at time(1) = " << plan->getPositionAtTime(1) << endl;
	cout << "Position at time(2) = " << plan->getPositionAtTime(2) << endl;
	cout << "Position at time(2.1) = " << plan->getPositionAtTime(2.1) << endl;
	cout << "Position at time(4) = " << plan->getPositionAtTime(4) << endl;
	cout << "Position at time(4.1) = " << plan->getPositionAtTime(4.1) << endl;
	cout << "Position at time(50) = " << plan->getPositionAtTime(50) << endl;
}

void validatePlan(MotionPlan * plan)
{
	int count = 0;
	if (plan->motionIntervals.empty())
	{
		cout << "Plan is empty" << endl;
		return;
	}
	for (auto it=plan->motionIntervals.begin(); it != plan->motionIntervals.end(); it++)
	{
		if (it->duration < 0)
		{
			cout << "Error: Interval " << count << " has negative time." << endl;
		}		
		count++;
	}

	for (double d=0;d<5;d++)
	{
		cout << "V("<<d<<")=" << plan->getSpeedAtTime(d) << "\t";
		cout << "P("<<d<<")=" << plan->getPositionAtTime(d);
		cout << endl;
	}
	double d = 1000;
	cout << "P("<<d<<")=" << plan->getPositionAtTime(d) << endl;
}

void testOptimalPlanBuilding()
{
	double startAngle = 0, targetAngle = -364; //AS5048::degreesToSteps(-90);
	timespec start;
	TimeUtil::setNow(start);

	double accel = 5000, approachSpeed = -455, approachDist = -150, initialSpeed = -1018;
	double maxSpeed = 1000, s;

	double targetTime = MotionController::optimalSpeed(accel,approachDist,(targetAngle-startAngle),initialSpeed,approachSpeed,maxSpeed,s);
	
	cout << "Minimum time = " << targetTime << "s, OptSpeed=" << s << " steps/s" << endl;
	targetTime += 0.001;
	//auto plan = MotionController::buildMotionPlan(startAngle,targetAngle,targetTime,approachSpeed,accel);
	PlanSolution result;
	MotionController::calculatePlan(accel,approachDist,targetTime,(targetAngle-startAngle),initialSpeed,approachSpeed,result);

	MotionController::validateSolution(result);
		
	cout << "Accel0=" << result.accel0 << endl;
	cout << "Accel1=" << result.accel1 << endl;
	
	//validatePlan(plan.get());

}

void testOptimalPlanBuilding2()
{
	double startAngle = 0, targetAngle = AS5048::degreesToSteps(-2);
	timespec start;
	TimeUtil::setNow(start);

	double accel = 1000, initialSpeed = 0;
	double maxSpeed = 10000, s;

	double targetTime = MotionController::optimalSpeed2Part(accel,(targetAngle-startAngle),initialSpeed,maxSpeed,s)+0.1;

	cout << endl << endl << "2-Part plan" << endl;
	cout << "Minimum time = " << targetTime << "s, OptSpeed=" << s << " steps/s" << endl;
	
	PlanSolution result;
	MotionController::calculatePlan2Part(accel,targetTime,(targetAngle-startAngle),initialSpeed,result);

	cout << "Accel0=" << result.accel0 << endl;
	cout << "Accel1=" << result.accel1 << endl;

	MotionController::validateSolution(result);

	MotionPlan * plan = new MotionPlan();
	if (result.t0 >= 0.01)
		plan->motionIntervals.push_back(MotionInterval(initialSpeed,result.travelVelocity,result.t0));
	if (result.t1 >= 0.01)
		plan->motionIntervals.push_back(MotionInterval(result.travelVelocity,result.travelVelocity,result.t1));
	plan->startAngle = 0;

	validatePlan(plan);
}

void testCompletePlanBuilding()
{
	vector<MotionStep*> steps;
	for (double s=1;s<5;s++)
	{
		double stepSize = MathUtil::degreesToRadians(-2*s);
		MotionStep * step = new MotionStep();
		for (int i=0;i<6;i++)
		{
			step->jointAngleDelta[i] = stepSize;
		}
		steps.push_back(step);
	}
	
	cout << endl << endl << "Creating motion plan" << endl;
	auto plan = motionController->createMotionPlans(steps, 10000, 10000);
	
	for (auto it=plan.begin();it!=plan.end();it++)
		validatePlan(it->get());
}*/
