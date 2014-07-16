#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_1
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_2
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_4
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_8


#include <stdlib.h>
#include <unistd.h>
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
#include <iomanip>

#include "cJSON.h"

#include "I2CBus.hpp"
#include "PredictiveJointController.hpp"
#include "Configuration.hpp"
#include "MathUtils.hpp"
#include "AsyncLogger.hpp"
#include "AsyncI2CSender.hpp"
#include "ThreadUtils.hpp"
#include "ALog.hpp"
#include "RealtimeLoopController.hpp"
#include "ServoDirectController.hpp"
#include "WheelMotionController.hpp"
#include "NavigationController.hpp"
#include "FaceController.hpp"
#include "PCA9552.hpp"

#define VMATH_NAMESPACE vmath
#include "vmath.h"

using namespace std;
using namespace vmath;


map<string,I2CBus*> busMap;

double fmt(double d)
{
	return std::round(d/0.01)*0.01;
}

void handle(int sig) {
	void *symbolArray[50];
	size_t size;

	// get void*'s for all entries on the stack
	size = backtrace(symbolArray, 50);

	// print out all the frames to stderr
	cout << "Error: signal " << sig << endl;
	backtrace_symbols_fd(symbolArray, size, STDERR_FILENO);
	exit(1);
}

map<string,I2CBus*> initializeI2C()
{	
	map<string,I2CBus*> busMap;
//	busMap.insert(make_pair("i2c-1",new I2CBus("/dev/i2c-1")));
//	busMap.insert(make_pair("i2c-2",new I2CBus("/dev/i2c-2")));
	busMap.insert(make_pair("i2c-3",new I2CBus("/dev/i2c-3")));
//	busMap.insert(make_pair("i2c-4",new I2CBus("/dev/i2c-4")));
	return busMap;
}


vector<PredictiveJointController*> loadControllers(DriveModel * driveModel, map<string,I2CBus*> busMap)
{
	vector<PredictiveJointController*> controllers;
	
	for (int i=0;i<driveModel->wheelJoints.size();i++)
	{
		PredictiveJointController * pjc = new PredictiveJointController(&(driveModel->wheelJoints.at(i)),busMap);
		controllers.push_back(pjc);
	}
	
	return controllers;
}

int doRobot(int argc, char * argv[])
{
	vector<PredictiveJointController*> controllers;
	std::string configFileName = "config.json";
	
	DriveModel * driveModel;
	
	bool running = false;
	
	signal(SIGSEGV, handle);
	
	if (argc >= 2)
	{
		configFileName = std::string(argv[1]);
	}
	
	try
	{
		Configuration::getInstance().loadConfig(configFileName);
		DRV8830::MaxVoltageStep = DRV8830::voltageToSteps(Configuration::getInstance().getObject("GlobalSettings.MaxVoltage")->valuedouble);
		driveModel = new DriveModel(Configuration::getInstance().getObject("DriveModel"));
				
		if (Configuration::getInstance().getObject("GlobalSettings.WheelsEnabled")->valueint != 0)
			controllers = loadControllers(driveModel,busMap);
	}
	catch (ConfigurationException & confEx)
	{
		cout << "Configuration error: " << confEx.what() << endl;
		cout << "Exiting." << endl;
		return 1;
	}
	cout << std::setbase(10) << 1.0 << endl;
	
	auto neckJoint = Configuration::getInstance().getObject("JointDefinitions.NeckPitch");
	auto neckJointModel = new JointModel(neckJoint);
	controllers.push_back(new PredictiveJointController(neckJointModel,busMap));
	
	//Initialize controllers
	ServoDirectController * servoDirectController = new ServoDirectController(new MotionPlanner(controllers),controllers);
	
	WheelMotionController * wheelMotionController = new WheelMotionController(controllers,*driveModel,servoDirectController);
	NavigationController * navController = new NavigationController(wheelMotionController);
	
	RealtimeLoopController jointLoop(controllers);
	
	jointLoop.addWatcher(wheelMotionController);
	
	FaceController * fc = new FaceController(new PCA9552(busMap["i2c-3"],96));
	
	
	
	jointLoop.addWatcher(fc);
	
	//Start everything
	AsyncLogger::getInstance().startThread();
	jointLoop.start();
	running = true;
	
	servoDirectController->prepareAllJoints();
	
	//Make main thread lower priority
	ThreadUtils::requestPriority(SCHED_OTHER);
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
				else if (command.compare("enable") == 0 || command.compare("ea") == 0)
				{
					
					string enableName;
					
					if (command.compare("ea") == 0)
						enableName = "all";
					else
						input >> enableName;
					
					if (input.fail()) {
						cout << "Invalid input. Usage: enable <index|all>" << endl;
					}
					else
					{
						bool enableAll = false;
						int jointIndex = 0;
						
						if (enableName.compare("all") == 0)
							enableAll = true;
						else
							jointIndex= std::stoi(enableName);
						
						RealtimeDispatcher::AddTask([servoDirectController,enableAll,jointIndex,controllers](){
							if (enableAll)
								servoDirectController->enableAllJoints();
							else
								controllers.at(jointIndex)->enable();
						});
					}
				}
				else if (command.compare("home") == 0) {
					
					RealtimeDispatcher::AddTask([servoDirectController](){
						servoDirectController->zeroAllJoints();
					});
				}
				else if (command.compare("set") == 0 || command.compare("s") == 0) {
					
					int jointIndex;
					double angle;
					
					input >> jointIndex;
					input >> angle;
					
					if (input.fail()) {
						cout << "Invalid input. Usage: set <index> <angle>" << endl;
					}
					else
					{
						RealtimeDispatcher::AddTask([servoDirectController,jointIndex,angle](){
							servoDirectController->setJointPosition(jointIndex, AS5048::degreesToSteps(angle));
						});
					}
					
				}
				else if (command.compare("ss") == 0 || command.compare("setspeed") == 0)
				{
					int jointIndex;
					double speed, runTime;
					
					input >> jointIndex;
					input >> speed;
					
					if (input.fail()) {
						cout << "Invalid input. Usage: setspeed <index> <speed> [runtime]" << endl;
					}
					else
					{
						input >> runTime;
						if (input.fail()) runTime = 30.0;
						
						RealtimeDispatcher::AddTask([servoDirectController,jointIndex,runTime,speed](){
							servoDirectController->setJointVelocity(jointIndex, AS5048::degreesToSteps(speed),runTime);
						});
					}
					
				}
				else if (command.compare("list") == 0) {
					
					cout << "Angles: ";
					for (auto it = controllers.begin(); it != controllers.end(); it++)
					{
						cout << setprecision(2) << AS5048::stepsToDegrees((*it)->getCurrentAngle()) << " ";
					}
					cout << std::fixed;
					cout << endl;
				}
				else if (command.compare("pause") == 0)
				{
					int jointIndex;
					input >> jointIndex;
					if (input.fail()) {
						cout << "Usage: pause <jointIndex>" << endl;
					} else {
						RealtimeDispatcher::AddTask([controllers,jointIndex](){
							controllers.at(jointIndex)->pause();
						});
					}
				}
				else if (command.compare("time") == 0)
				{
					jointLoop.printTimeDebugInfo();
					//for (auto it = busMap.begin(); it != busMap.end(); it++)
					//{
					//	cout << "Bus: " << it->first <<
					//		". Read = " << it->second->readTime->avg() << " ms" <<
					//		". Write = " << it->second->writeTime->avg() << " ms" <<
					//	endl;
					//}
				}
				else if (command.compare("mv") == 0)
				{
					double distance;
					input >> distance;
					if (input.fail())
					{
						cout << "Usage: mv <distance>" << endl;
					}
					else
					{
						RealtimeDispatcher::AddTask([wheelMotionController,distance](){
							wheelMotionController->translateBy(distance);
						});
					}
				}
				else if (command.compare("rt") == 0)
				{
					double rotation;
					input >> rotation;
					if (input.fail())
					{
						cout << "Usage: rt <angle deg>" << endl;
					}
					else
					{
						RealtimeDispatcher::AddTask([wheelMotionController,rotation](){
							wheelMotionController->rotateBy(rotation);
						});
					}
				}
				else if (command.compare("path") == 0)
				{
					string filename;
					input >> filename;
					if (input.fail()) {
						cout << "Usage: path <pathfile> [repeat count]" << endl;
					} else {
						cJSON * pathObject = Configuration::loadJsonFile(filename);
						
						int repeatCount;
						input >> repeatCount;
						if (input.fail()) repeatCount = 1;
						
						if (pathObject) {
							cout << "Loaded path of " << cJSON_GetArraySize(pathObject) << " length" << endl;
							
							RealtimeDispatcher::AddTask([pathObject,navController,repeatCount](){
								navController->executePath(pathObject,repeatCount);
							});
						}
					}
				}
				else if (command.compare("led") == 0)
				{
					RealtimeDispatcher::AddTask([fc](){
						fc->startTestAnimation();
					});
				}
				else
				{
					cout << "Invalid command" << endl;
				}
			}
			catch (std::runtime_error & commandException)
			{
				cout << "Command caused exception: " << commandException.what() << endl;
			}
		}
	}
	catch (std::logic_error & e)
	{
		cout << "Illogically, " << e.what() << endl;
	}
	catch (std::runtime_error & e)
	{
		cout << "Runtime Exception: " << e.what() << endl;
	}
	
	cout << "Powering down control loops..";
	jointLoop.stop();
	cout << "Done." << endl;
	
	return 0;
}

int main(int argc, char *argv[])
{
	cout << "STARTING TempMotion" << endl;
	std::string configFileName = "config.json";
	
	
//	RealtimeLoopController jointLoop(controllers);

	doRobot(argc,argv);

	AsyncI2CSender::cleanup();
	
	AsyncLogger::getInstance().joinThread();
	
//	alog->sync();
}
