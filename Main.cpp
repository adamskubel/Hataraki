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
#include "AsyncLogger.hpp"

#define VMATH_NAMESPACE vmath
#include "vmath.h"



using namespace std;
using namespace ikfast2;
using namespace vmath;

volatile bool running;

MotionController * motionController;



void updateController()
{
	while (running)
	{
		motionController->updateController();
	}
}



void printPositionForAngles(IkReal * jointAngles) {
	
	IkReal translationMatrix[3];
	IkReal rotationMatrix[9];
	
	ikfast2::ComputeFk(jointAngles,translationMatrix,rotationMatrix);
	
	cout << "Endpoint position is " << translationMatrix[0]*100.0 << "," << translationMatrix[1]*100.0 << "," << translationMatrix[2]*100.0 <<endl;
	cout << "Rotation matrix is: ";
	for (int i=0;i<9;i++)
	{
		cout << rotationMatrix[i] << " ";
	}
	cout << endl;
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
	vector<PredictiveJointController*> controllers;
	cJSON * globalConfig;
	ArmModel * armModel;
	std::string configFileName = "config.json";
	I2CBus * bus = NULL;

	running = false;

	signal(SIGSEGV, handle);

	if (argc >= 2)
	{
		configFileName = std::string(argv[1]);
	}

	//Configuration

	try
	{

		Configuration::getInstance().loadConfig(configFileName);

		globalConfig = cJSON_GetObjectItem(Configuration::getInstance().getRoot(),"GlobalSettings");	

		samplePeriod = 1.0/cJSON_GetObjectItem(globalConfig,"UpdateFrequency")->valuedouble;
		cout << "Operating with period of " << samplePeriod << " seconds. " <<endl;

		DRV8830::MaxVoltageStep = DRV8830::voltageToSteps(cJSON_GetObjectItem(globalConfig,"MaxVoltage")->valuedouble);	
		cout << "Setting maximum voltage to " << DRV8830::stepsToVoltage(DRV8830::MaxVoltageStep) << " V (Step=0x" << hex << DRV8830::MaxVoltageStep << ")" << endl;

		armModel = new ArmModel(cJSON_GetObjectItem(Configuration::getInstance().getRoot(),"ArmModel"));

		cout << "Opening I2C bus... " << endl;
		bus = new I2CBus("/dev/i2c-1");

		for (int i=0;i<armModel->joints.size();i++)
		{
			PredictiveJointController * pjc = new PredictiveJointController(&(armModel->joints.at(i)),bus, samplePeriod);		
			controllers.push_back(pjc);
		}
	}
	catch (ConfigurationException & confEx)
	{
		cout << "Configuration error: " << confEx.what() << endl;
		cout << "Exiting." << endl;
		return 1;
	}
	
	PoseDynamics::getInstance().setArmModel(armModel);
		
	motionController = new MotionController(controllers,samplePeriod,cJSON_GetObjectItem(globalConfig,"MotionPlanSteps")->valueint);
	
	motionController->prepareAllJoints();		
		
	AsyncLogger::getInstance().startThread();

	running = true;
	std::thread jointUpdate(updateController);
	
	try
	{

		while (running) 
		{
			cout << "->>";

			string inputString;
			getline(cin,inputString);

			stringstream input(inputString);
			
			string command;
			input >> command;
						
			try
			{
				if (command.compare("exit") == 0 || command.compare("k") == 0) {
					cout << "Exit command." << endl;
					running = false;
				}
				else if (command.compare("enable") == 0)
				{
					motionController->postTask([motionController](){
						motionController->enableAllJoints();
					});
				}
				else if (command.compare("enable_") == 0)
				{
					int jointIndex;
					input >> jointIndex;

					if (input.fail()) {
						cout << "Invalid input. Usage: enable_ <index>" << endl;
					}
					else
					{
						motionController->postTask([jointIndex,controllers](){
							controllers.at(jointIndex)->enable();
						});
					}
				}
				else if (command.compare("home") == 0) {
					
					motionController->postTask([](){
						motionController->zeroAllJoints();
					});
				}
				else if (command.compare("set") == 0) {

					int jointIndex;
					double angle, velocity;

					input >> jointIndex;
					input >> angle;
					input >> velocity;
					
					if (input.fail()) {
						cout << "Invalid input. Usage: set <index> <angle> <|velocity|>" << endl;
					}
					else
					{
						motionController->postTask([jointIndex,angle,velocity](){
							motionController->setJointPosition(jointIndex, AS5048::degreesToSteps(angle), AS5048::degreesToSteps(velocity));
						});
					}
					
				}
				else if (command.compare("list") == 0) {

					cout << "Angles: ";
					for (auto it = controllers.begin(); it != controllers.end(); it++)
					{					
						cout << AS5048::stepsToDegrees((*it)->getCurrentAngle()) << " ";
					}
					cout << endl;
				}
				else if (command.compare("setpos") == 0) {

					Vector3d targetPosition;
					
					input >> targetPosition.x;
					input >> targetPosition.y;
					input >> targetPosition.z;
					
					targetPosition = targetPosition / 100.0;
					

					if (input.fail()) {
						cout << "Usage: planto <x>cm <y>cm <z>cm" << endl;
					} else {
						//motionController->postTask([targetPosition](){
						motionController->moveToPosition(targetPosition);
						//});
					}
				}
				else if (command.compare("getpos") == 0) {

					double angles[6];
					int i=0;
					for (auto it = controllers.begin(); it != controllers.end(); it++,i++)
					{					
						angles[i] = MathUtil::degreesToRadians((*it)->getCurrentAngle());
					}
					
					printPositionForAngles(angles);
				}
				else if (command.compare("pause") == 0)
				{
					int jointIndex;
					input >> jointIndex;
					if (input.fail()) {
						cout << "Usage: pause <jointIndex>" << endl;
					} else {
						motionController->postTask([controllers,jointIndex](){
							controllers.at(jointIndex)->pause();
						});
					}
				}
				else
				{
					cout << "Invalid entry. Valid commands are: enable, exit, home, set, list, getpos, setpos, pause " << endl;
				}
			}
			catch (std::runtime_error & commandException)
			{
				cout << "Command caused exception: " << commandException.what() << endl;
			}
		}
	}
	catch (std::runtime_error & e)
	{
		cout << "Exception: " << e.what() << endl;
	}

	cout << "Powering down control loops..";	
	jointUpdate.join();
	cout << "Done." << endl;
	
	motionController->shutdown();	
		
	AsyncLogger::getInstance().joinThread();
}
