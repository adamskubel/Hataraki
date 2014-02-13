#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_1
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_2
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_4
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_8

//#define IKFAST_CLIBRARY
#define IKFAST_NO_MAIN
#define IKFAST_HAS_LIBRARY
#define IKFAST_NAMESPACE ikfast2

#include "ikfast.h"

#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <inttypes.h>
#include <stdio.h>
#include <signal.h>
#include <execinfo.h>

#include <vector>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept> 
#include <queue>

#include "cJSON.h"

#include "I2CBus.hpp"
#include "PredictiveJointController.hpp"
#include "Configuration.hpp"
#include "MotionController.hpp"
#include "MathUtils.hpp"

#define VMATH_NAMESPACE vmath
#include "vmath.h"

#include "PoseDynamics.hpp"
#include "MotionPlan.hpp"


using namespace std;
using namespace ikfast2;

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
	
	ikfast2::ComputeFk(radAngles,translationMatrix,rotationMatrix);
	
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
	cout << "Speed at time(0) = " << plan->getSpeedAtTime(0) << endl;
	cout << "Speed at time(1) = " << plan->getSpeedAtTime(1) << endl;
	cout << "Speed at time(2) = " << plan->getSpeedAtTime(2) << endl;
	cout << "Speed at time(2.1) = " << plan->getSpeedAtTime(2.1) << endl;

	cout << "Position at time(0) = " << plan->getPositionAtTime(0) << endl;
	cout << "Position at time(1) = " << plan->getPositionAtTime(1) << endl;
	cout << "Position at time(2) = " << plan->getPositionAtTime(2) << endl;
	cout << "Position at time(2.1) = " << plan->getPositionAtTime(2.1) << endl;
	cout << "Position at time(4) = " << plan->getPositionAtTime(4) << endl;
	cout << "Position at time(4.1) = " << plan->getPositionAtTime(4.1) << endl;
	cout << "Position at time(50) = " << plan->getPositionAtTime(50) << endl;

}

void handle(int sig) {
	void *array[30];
	size_t size;

	// get void*'s for all entries on the stack
	size = backtrace(array, 30);

	// print out all the frames to stderr
	fprintf(stderr, "Error: signal %d:\n", sig);
	backtrace_symbols_fd(array, size, STDERR_FILENO);
	exit(1);
}


int main(int argc, char *argv[])
{
	double samplePeriod;

	signal(SIGSEGV, handle);
	
	//signal(SIGINT, signal_callback_handler);

	std::string configFileName = "config.json";

	if (argc >= 2)
	{
		configFileName = std::string(argv[1]);
	}

	Configuration::getInstance().loadConfig(configFileName);

	cJSON * globalConfig = cJSON_GetObjectItem(Configuration::getInstance().getRoot(),"GlobalSettings");	
	DRV8830::MaxVoltageStep = DRV8830::voltageToSteps(cJSON_GetObjectItem(globalConfig,"MaxVoltage")->valuedouble);		
	cJSON * jointArray = cJSON_GetObjectItem(Configuration::getInstance().getRoot(),"JointDefinitions");
	
	ArmModel * armModel = new ArmModel(cJSON_GetObjectItem(Configuration::getInstance().getRoot(),"ArmModel"));

	vector<PredictiveJointController*> controllers;

	for (int i=0;i<armModel->joints.size();i++)
	{
		PredictiveJointController * pjc = new PredictiveJointController(&(armModel->joints.at(i)),NULL, 0.01);	
		controllers.push_back(pjc);
	}

	//testServoModel(&(controllers.at(0)->getJointModel()->servoModel));
	//testArmModel(armModel);

	//double angles[] = {0,0.1,0,0.1,0.1,0};
	//testPoseDynamics(armModel,angles);


	//double angles2[] = {0,15,0,15,15,0};
	//testPoseDynamics(armModel,angles2);

	//testCsvWriteTime();
	//
	//testFK();	

	testMotionPlan();
}
