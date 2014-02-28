#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_1
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_2
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_4
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_8

//#define IKFAST_CLIBRARY
#define IKFAST_NO_MAIN
#define IKFAST_HAS_LIBRARY
#define IKFAST_NAMESPACE ikfast2


#include <execinfo.h>

#include <vector>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>

#include "cJSON.h"

#include "I2CBus.hpp"
#include "PredictiveJointController.hpp"
#include "Configuration.hpp"
#include "MotionController.hpp"
#include "MathUtils.hpp"
#include "MotionPlanner.hpp"

#define VMATH_NAMESPACE vmath
#include "vmath.h"

#include "PoseDynamics.hpp"
#include "MotionPlan.hpp"


using namespace std;
using namespace ikfast2;
using namespace vmath;

MotionController * motionController;
vector<PredictiveJointController*> controllers;


double fmt(double d)
{
	return std::round(d/0.01)*0.01;
}



void handle(int sig) {
	void *array[30];
	size_t size;

	// get void*'s for all entries on the stack
	size = backtrace(array, 30);

	// print out all the frames to stderr
	fprintf(stderr, "Error: signal %d:\n", sig);
	backtrace_symbols_fd(array, (int)size, STDERR_FILENO);
	exit(1);
}

void testMotionPlanning();


int main(int argc, char *argv[])
{
	double samplePeriod = 0.01;

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
//	cJSON * jointArray = cJSON_GetObjectItem(Configuration::getInstance().getRoot(),"JointDefinitions");
	
	ArmModel * armModel = new ArmModel(cJSON_GetObjectItem(Configuration::getInstance().getRoot(),"ArmModel"));


	for (int i=0;i<armModel->joints.size();i++)
	{
		PredictiveJointController * pjc = new PredictiveJointController(&(armModel->joints.at(i)),NULL, 0.01);	
		controllers.push_back(pjc);
	}
	
	motionController = new MotionController(controllers,samplePeriod,5);
	
	testMotionPlanning();
}



void testMotionPlanning()
{
	MotionPlanner * mp = new MotionPlanner(controllers);
	
	double  initialAngles_deg[] = {0.1,-6.4,0,34,63,0}; //11 0 -6
	vector<double> intialAngles_rad, initialAngles_step;
	
	for (int i=0;i<6;i++) intialAngles_rad.push_back(MathUtil::degreesToRadians(initialAngles_deg[i]));
	for (int i=0;i<6;i++) initialAngles_step.push_back(AS5048::degreesToSteps(initialAngles_deg[i]));
	
	int divisionCount = 20;
	mp->setPathDivisions(divisionCount);
	auto steps = mp->buildMotionSteps(intialAngles_rad, Vector3d(10,0,-6)/100.0, Matrix3d::createRotationAroundAxis(0, -90, 0));
	
	//if (divisionCount != steps.size()) cout << "Division count is " << steps.size() << ", expected " << divisionCount << endl;
	
	std::ofstream outFile("test_motionplanning.csv");
	int numChannels = 6;
	outFile << "Time,";
	for (int c=0;c<numChannels;c++)
	{
		outFile <<
		"X" << c <<
		",dX" << c <<
//		",ddX" << c <<
		",";
	}
	outFile << endl;
	
	
	auto motionPlan = mp->buildPlan(steps);
	
	cout << "Plan duration = " << motionPlan.front()->getPlanDuration() << endl;
	
	for (int c = 0; c < numChannels; c++)
	{
		cout << "Final = " << motionPlan[c]->finalAngle << ", Final(t) = " << motionPlan[c]->getPositionAtTime(1000) << endl;
	}
	
	for (double t = 0; t < motionPlan.front()->getPlanDuration(); t += 0.01)
	{
		outFile << t << ",";
		for (int c=0;c<numChannels;c++)
		{
			outFile
			<< fmt(motionPlan[c]->getPositionAtTime(t)) << ","
			<< fmt(motionPlan[c]->getSpeedAtTime(t)) << ",";
		}
		outFile << endl;
	}
	outFile.close();	
}
