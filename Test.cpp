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



using namespace std;
using namespace ikfast2;


void testServoModel(ServoModel * sm)
{	
	double torque = sm->getTorqueForVoltageSpeed(1.2,1000);
	cout << "V=1.2, S=1000, T=" << torque << endl;
	cout << "V=1.2, T=" << torque << ", S=" << sm->getSpeedForTorqueVoltage(torque,1.2) << endl;
	cout << "T=" << torque << ", S=1000, V=" << sm->getVoltageForTorqueSpeed(torque,1000) << endl;

	cout << endl;
	torque = 0.2;
	cout << "T=" << torque << ", S=1000, V=" << sm->getVoltageForTorqueSpeed(torque,1000) << endl;	
	torque = 0.15;
	cout << "T=" << torque << ", S=1000, V=" << sm->getVoltageForTorqueSpeed(torque,1000) << endl;	
	cout << "V=1.2, S=100, T=" << sm->getTorqueForVoltageSpeed(1.2,100) << endl;

	
	cout << endl;
	torque = -0.125;
	cout << "T=" << torque << ", S=-910, V=" << sm->getVoltageForTorqueSpeed(torque,-910) << endl;	
	torque = -0.133;
	cout << "T=" << torque << ", S=-910, V=" << sm->getVoltageForTorqueSpeed(torque,-910) << endl;	

	cout << endl;
	torque = sm->getTorqueForVoltageSpeed(-1.2,-1000);
	cout << "V=-1.2, S=-1000, T=" << torque << endl;
	cout << "V=-1.2, T=" << torque << ", S=" << sm->getSpeedForTorqueVoltage(torque,-1.2) << endl;
	cout << "T=" << torque << ", S=-1000, V=" << sm->getVoltageForTorqueSpeed(torque,-1000) << endl;
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
	MotionController::PlanStepCount = cJSON_GetObjectItem(globalConfig,"MotionPlanSteps")->valueint;
	
	cJSON * jointArray = cJSON_GetObjectItem(Configuration::getInstance().getRoot(),"JointDefinitions");
	
	vector<PredictiveJointController*> controllers;

	for (int i=0;i<cJSON_GetArraySize(jointArray);i++)
	{
		cJSON * jointItem =cJSON_GetArrayItem(jointArray,i);
		cout << "Adding joint with name '" << cJSON_GetObjectItem(jointItem,"Name")->valuestring<< "'" << endl;
		PredictiveJointController * pjc = new PredictiveJointController(jointItem,NULL,0.01);		
		controllers.push_back(pjc);
	}

	testServoModel(&(controllers.at(0)->getJointModel()->servoModel));
	
}
