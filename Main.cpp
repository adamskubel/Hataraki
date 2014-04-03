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
#include <pthread.h>
#include <sched.h>

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

#include "FlipIdentifier.hpp"

#define VMATH_NAMESPACE vmath
#include "vmath.h"


using namespace std;
using namespace ikfast;
using namespace vmath;

volatile bool running;

MotionController * motionController;


void requestPriority(int priority)
{
	pthread_t this_thread = pthread_self();
	
	struct sched_param params;
	// We'll set the priority to the maximum.
	params.sched_priority = sched_get_priority_max(priority);
	int ret = pthread_setschedparam(this_thread, priority, &params);
	
	if (ret != 0) {
		// Print the error
		std::cout << "Unsuccessful in setting thread realtime prio" << std::endl;
		return;
	}
}


void updateController()
{
	requestPriority(SCHED_FIFO);
	
	while (running)
	{
		motionController->updateController();
	}
}


double fmt(double d)
{
	return std::round(d/0.01)*0.01;
}



void printPositionForAngles(IkReal * jointAngles) {
	
	IkReal t[3];
	IkReal r[9];
	
	ComputeFk(jointAngles,t,r);
		
	Matrix3d rotationMatrix = Matrix3d::fromRowMajorArray(r);	
	Vector3d tipPos = Vector3d(t[0],t[1],t[2])*100.0;
	//Vector3d tip = rotationMatrix * Vector3d(1,0,0);

	double xR,yR,zR;
	MathUtil::extractEulerAngles(rotationMatrix,xR,yR,zR);

	cout << "Tip position: " << tipPos.toString() << endl;
	cout << "Tip angles: " << fmt(xR) << " " << fmt(yR) << " " << fmt(zR) << endl;
	cout << endl;
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

void ikControl()
{
	IKControlUI ui(motionController);
	ui.start();	
}

int main(int argc, char *argv[])
{
	vector<PredictiveJointController*> controllers;
	cJSON * globalConfig;
	ArmModel * armModel;
	std::string configFileName = "config.json";
	TrajectoryController * trajectoryController;

	map<string,I2CBus*> busMap;

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

		DRV8830::MaxVoltageStep = DRV8830::voltageToSteps(cJSON_GetObjectItem(globalConfig,"MaxVoltage")->valuedouble);	
				
		busMap.insert(make_pair("i2c-1",new I2CBus("/dev/i2c-1")));
		busMap.insert(make_pair("i2c-2",new I2CBus("/dev/i2c-2")));

		armModel = new ArmModel(cJSON_GetObjectItem(Configuration::getInstance().getRoot(),"ArmModel"));
		for (int i=0;i<armModel->joints.size();i++)
		{
			PredictiveJointController * pjc = new PredictiveJointController(&(armModel->joints.at(i)),busMap);		
			controllers.push_back(pjc);
		}
				
		motionController = new MotionController(controllers);	
		trajectoryController = new TrajectoryController(motionController);
		PoseDynamics::getInstance().setArmModel(armModel);
	}
	catch (ConfigurationException & confEx)
	{
		cout << "Configuration error: " << confEx.what() << endl;
		cout << "Exiting." << endl;
		return 1;
	}
	
	motionController->prepareAllJoints();	
	cout << std::setbase(10) << 1.0 << endl;	
		
	running = true;
	std::thread jointUpdate(updateController);	
	requestPriority(SCHED_OTHER);

	AsyncLogger::getInstance().startThread();

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
						int jointIndex;

						if (enableName.compare("all") == 0) 
							enableAll = true;
						else 
							jointIndex= std::stoi(enableName);
						
						motionController->postTask([enableAll,jointIndex,controllers](){
							if (enableAll)
								motionController->enableAllJoints();
							else
								controllers.at(jointIndex)->enable();
						});
					}
				}
				else if (command.compare("home") == 0) {
					
					motionController->postTask([](){
						motionController->zeroAllJoints();
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
						motionController->postTask([jointIndex,angle](){
							motionController->setJointPosition(jointIndex, AS5048::degreesToSteps(angle));
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

						motionController->postTask([jointIndex,runTime,speed](){
							motionController->setJointVelocity(jointIndex, AS5048::degreesToSteps(speed),runTime);
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
						motionController->postTask([controllers,jointIndex](){
							controllers.at(jointIndex)->pause();
						});
					}
				}
				else if (command.compare("ik") == 0)
				{
					ikControl();
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
							
							motionController->postTask([pathObject,trajectoryController,reps](){
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
				else if (command.compare("path") == 0)
				{
					string filename;
					input >> filename;
					if (input.fail()) {
						cout << "Usage: path <pathfile>" << endl;
					} else {
						cJSON * pathObject = Configuration::loadJsonFile(filename);
						
						if (pathObject) {
							cout << "Loaded path of " << cJSON_GetArraySize(pathObject) << " length" << endl;
							
							motionController->postTask([pathObject,motionController](){
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
					motionController->printAverageTime();
					//for (auto it = busMap.begin(); it != busMap.end(); it++)
					//{
					//	cout << "Bus: " << it->first <<
					//		". Read = " << it->second->readTime->avg() << " ms" <<
					//		". Write = " << it->second->writeTime->avg() << " ms" <<
					//	endl;
					//}
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
	jointUpdate.join();
	cout << "Done." << endl;
	
	motionController->shutdown();
	
	AsyncLogger::getInstance().joinThread();
}
