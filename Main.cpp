#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_1
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_2
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_4
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_8


#include "IKFast.hpp"

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
#include "MotionController.hpp"
#include "MathUtils.hpp"
#include "AsyncLogger.hpp"
#include "IKControlUI.hpp"
#include "TrajectoryController.hpp"
#include "AsyncI2CSender.hpp"
#include "ThreadUtils.hpp"
#include "ALog.hpp"
#include "RealtimeLoopController.hpp"
#include "ServoDirectController.hpp"
#include "WheelMotionController.hpp"
#include "NavigationController.hpp"
#include "ObstacleAwareNavigator.hpp"
#include "AntennaDeflectionSensor.hpp"

#define VMATH_NAMESPACE vmath
#include "vmath.h"

using namespace std;
using namespace ikfast;
using namespace vmath;

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

void ikControl(MotionController * motionController)
{
	IKControlUI ui(motionController);
	ui.start();
}


map<string,I2CBus*> initializeI2C()
{	
	map<string,I2CBus*> busMap;
	busMap.insert(make_pair("i2c-1",new I2CBus("/dev/i2c-1")));
	busMap.insert(make_pair("i2c-2",new I2CBus("/dev/i2c-2")));
	return busMap;
}


vector<PredictiveJointController*> loadControllers(ArmModel * armModel, map<string,I2CBus*> busMap)
{
	vector<PredictiveJointController*> controllers;
	
	for (int i=0;i<armModel->joints.size();i++)
	{
		PredictiveJointController * pjc = new PredictiveJointController(&(armModel->joints.at(i)),busMap);		
		controllers.push_back(pjc);
	}
	
	return controllers;
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

vector<AntennaDeflectionSensor*> loadAntennaSensors(cJSON * sensorArray, map<string,I2CBus*> busMap)
{
	vector<AntennaDeflectionSensor*> sensors;

	for (int i=0;i<cJSON_GetArraySize(sensorArray); i++)
	{
		AntennaSensorConfig config(cJSON_GetArrayItem(sensorArray,i));
		cout << "Loaded antenna: " << config.antennaName << endl;
		sensors.push_back(new AntennaDeflectionSensor(config,busMap[config.sensorBusName]));
	}

	return sensors;
}

int main(int argc, char *argv[])
{
	cout << "STARTING BASICMOTION" << endl;
	std::string configFileName = "config.json";		

	map<string,I2CBus*> busMap;
	vector<PredictiveJointController*> controllers;
	vector<AntennaDeflectionSensor*> antennas;
	
	ArmModel * armModel;
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
		armModel = new ArmModel(Configuration::getInstance().getObject("ArmModel"));	
		driveModel = new DriveModel(Configuration::getInstance().getObject("DriveModel"));
		
		busMap = initializeI2C();

		antennas = loadAntennaSensors(Configuration::getInstance().getObject("AntennaSensors"),busMap);
		
		if (Configuration::getInstance().getObject("GlobalSettings.ArmEnabled")->valueint != 0)
			controllers = loadControllers(armModel,busMap);
		else if (Configuration::getInstance().getObject("GlobalSettings.WheelsEnabled")->valueint != 0)
			controllers = loadControllers(driveModel,busMap);
	}
	catch (ConfigurationException & confEx)
	{
		cout << "Configuration error: " << confEx.what() << endl;
		cout << "Exiting." << endl;
		return 1;
	}	
	cout << std::setbase(10) << 1.0 << endl;
	
	//Initialize controllers
	MotionController * motionController = new MotionController(controllers);	
	TrajectoryController * trajectoryController = new TrajectoryController(motionController);
	ServoDirectController * servoDirectController = new ServoDirectController(motionController->getMotionPlanner(),controllers);

	WheelMotionController * wheelMotionController = new ObstacleAwareNavigator(antennas,controllers,*driveModel,servoDirectController);
	NavigationController * navController = new NavigationController(wheelMotionController);

	PoseDynamics::getInstance().setArmModel(armModel);
	
	RealtimeLoopController jointLoop(controllers);

	for (auto it=antennas.begin(); it != antennas.end(); it++)
	{
		jointLoop.addWatcher(*it);
	}
	
	jointLoop.addWatcher(motionController);
	jointLoop.addWatcher(wheelMotionController);
	
	//Start everything
	AsyncLogger::getInstance().startThread();	
	jointLoop.start();	
	running = true;
	
	servoDirectController->prepareAllJoints();
	for (auto it=antennas.begin(); it != antennas.end(); it++)
	{
		(*it)->start();
	}

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
				else if (command.compare("ik") == 0)
				{
					ikControl(motionController);
				}
				else if (command.compare("gl") == 0)
				{
					string filename;
					input >> filename;
					int reps = 1;
					
					if (input.fail()) {
						cout << "Usage: gl <goal list file> [repeat count]" << endl;
					} else {
						
						input >> reps;
						
						if (input.fail()) reps = 1;
						
						cJSON * pathObject = Configuration::loadJsonFile(filename);
						
						if (pathObject) {
							cout << "Loaded path of " << cJSON_GetArraySize(pathObject) << " length" << endl;
							
							RealtimeDispatcher::AddTask([motionController,pathObject,trajectoryController,reps](){
								auto goals = motionController->getTrajectoryPlanner()->buildTrajectory(pathObject,false);
									
								vector<IKGoal> goalList;
								
								for (int i=0;i<reps;i++)
								{
									auto it = goals.begin();
									it++; //Skip initial state
									for (; it != goals.end(); it++)
									{
										goalList.push_back(IKGoal(it->Position,it->Rotation,false));
									}
								}
								
								trajectoryController->executeSequentialGoals(goalList);								
							});
						}
					}
				}
				else if (command.compare("armpath") == 0)
				{
					string filename;
					input >> filename;
					if (input.fail()) {
						cout << "Usage: armpath <pathfile>" << endl;
					} else {
						cJSON * pathObject = Configuration::loadJsonFile(filename);
						
						if (pathObject) {
							cout << "Loaded path of " << cJSON_GetArraySize(pathObject) << " length" << endl;
							
							RealtimeDispatcher::AddTask([pathObject,motionController](){
								auto trajectory = motionController->getTrajectoryPlanner()->buildTrajectory(pathObject,false);
								cout << "Created trajectory with " << trajectory.size() << " states" << endl;
								auto motionPlan = motionController->getMotionPlanner()->buildPlan(trajectory);
								cout << "Created plan of duration " << motionPlan.front()->getPlanDuration() << " s " << endl;
								motionController->executeMotionPlan(motionPlan);
							});
						}
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
	
	motionController->shutdown();
	AsyncI2CSender::cleanup();
	
	AsyncLogger::getInstance().joinThread();
	
//	alog->sync();
}
