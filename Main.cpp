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

#include "FlipIdentifier.hpp"

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
	
	IkReal t[3];
	IkReal r[9];
	
	ikfast2::ComputeFk(jointAngles,t,r);
		
	Matrix3d rotationMatrix = Matrix3d::fromRowMajorArray(r);	
	Vector3d tipPos = Vector3d(t[0],t[1],t[2])*100.0;
	Vector3d tip = rotationMatrix * Vector3d(1,0,0);

	cout << "Endpoint position is: " << tipPos.toString() << endl;
	cout << "Endpoint direction vector: " << tip.toString() << endl;
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
	
	double defaultAccel = cJSON_GetObjectItem(globalConfig,"DefaultAccel")->valuedouble;

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
					motionController->postTask([](){
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
					double angle, travelTime;

					input >> jointIndex;
					input >> angle;
					input >> travelTime;
					
					if (input.fail()) {
						cout << "Invalid input. Usage: set <index> <angle> <time(sec)> [|accel|]" << endl;
					}
					else
					{
						double accel;
						input >> accel;
						if (input.fail()) accel = 0;

						motionController->postTask([jointIndex,angle,travelTime,accel](){
							motionController->setJointPosition(jointIndex, AS5048::degreesToSteps(angle),travelTime,AS5048::degreesToSteps(accel));
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
				else if (command.compare("setpos") == 0 || command.compare("goto") == 0) {

					Vector3d targetPosition;

					input >> targetPosition.x;
					input >> targetPosition.y;
					input >> targetPosition.z;
					
					targetPosition = targetPosition / 100.0;
					
					bool interactive = (command.compare("setpos") == 0);


					if (input.fail()) {
						cout << "Usage: <setpos|goto> <x>cm <y>cm <z>cm [rX rY rZ]" << endl;
					} else {
						
						int pathDivisionCount = 1;
						//input >> pathDivisionCount;
						//if (input.fail()) pathDivisionCount = 1;

						double rX,rY,rZ;
						input >> rX; input >> rY; input >> rZ;
						if (input.fail())
						{
							rY = -90; rX = 0; rZ = 0;
						}
						
						motionController->moveToPosition(targetPosition,Matrix3d::createRotationAroundAxis(rX,rY,rZ),pathDivisionCount,interactive);
					}
				}
				else if (command.compare("getpos") == 0) {

					double angles[6];
					int i=0;
					for (auto it = controllers.begin(); it != controllers.end(); it++,i++)
					{					
						angles[i] = AS5048::stepsToRadians((*it)->getCurrentAngle());
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
				else if (command.compare("flip") == 0)
				{
					int jointIndex,direction;
					input >> jointIndex;
					input >> direction;
					double voltage;
					input >> voltage;

					if (input.fail()) {
						cout << "Usage: flip <jointIndex> <direction> <voltage>" << endl;
					} else {
						motionController->postTask([controllers,direction,jointIndex,voltage](){
							controllers.at(jointIndex)->requestFlip(direction,voltage);
						});
					}
				}
				else if (command.compare("runpatterns") == 0)
				{
					int jointIndex;
					string filename;

					input >> jointIndex;
					input >> filename;

					if (input.fail()) {
						cout << "Usage: flip <jointIndex> <patternFile>" << endl;
					} else {
						string fileString = Configuration::get_file_contents(filename.c_str());
						cJSON * patterns = cJSON_Parse(fileString.c_str());

						if (!patterns) cout << "Invalid JSON file." << endl;
						else
						{
							FlipIdentifier flipId(samplePeriod,jointIndex,motionController);
							flipId.loadPatterns(patterns);
							flipId.execute();
						}
					}
				}
				else
				{
					cout << "Invalid entry. Valid commands are: enable, enable_, exit, k, home, set, list, getpos, setpos, pause, flip" << endl;
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

/*
 
 Primary application architecture
 
 Currently, I use a two threads. The IO+Control thread, and the input thread.
 
 Coupling the IO thread to the control thread may seem stupid, but it's actually kind of logical. 
 Because control is completely dependent on IO, there's no point in doing control unless IO has completed.
 
 However, there are some calculations involved in the control algorithms that can be run async.
 
 Aside from that, the IO --> Control relationship cannot be (and need not be?) broken IF I continue using I2C. SPI has different implications.
 
 So, the system will run on: 
	- A UI thread
	- A collection of IO-Control threads
	- Some thread to coordinate everything
	- One or more threads to perform intense computations in the background

 Right now I have no coordination between IO threads (because there's only one), and the UI thread is driving everything.
 
 The coordination thread manages the IO/Control threads. 
	1. Starts threads for each IO bus 
	2. Distributes commands to the appropriate thread (one message/task queue per thread)
	3. Cleans up, etc
 
 ************
 
 How will wheel control work?
 
 The wheel controller should actually be *very* similar to the joint controller. There are a few key differences in requirements:
	- Rotation can go past 360 degrees
	- The motion plan could be deterministic, or it could be realtime. 
	-- For instance, as we drive along some path, the velocity of each wheel will be constantly adjusted to steer/correct. 
	
 In terms of controller design, the general architecture should be largely the same. Differences will be limited to the planning component
 and to the calculations of angle, which are no longer bounded to +- PI. 
 
 ***********
 
 At this point it is wise to consider the executive control infrastructure. For the "blind" prototype, I have no intention of building in any meaningful
 amount of intelligence. Thus, the sole means of control will be user input. A textual interface is far from ideal - I would prefer direct USB control 
 using a gamepad/mouse/keyboard, but network control using similar peripherals is ok too. Just not over ssh. 
 
 From an architectural perspective, all the control logic is isolated to the UI thread (and possibly some networking threads). 
 Input events can be discrete and continuous. The UI logic should be as light as possible, especially for continuous/analog type inputs. 
 The reasoning is that the interpretation of the input is highly subject on the current state of the system.
 
 ***********
 
 In conclusion, my current architecture is more or less on track. I need to isolate the UI thread from the ex-control logic,
 and I need to isolate the ex-control logic from the device control logic. This should be fairly straightforward, but issues
 may (but will probably not) arise with concurrency if joints aren't executed in a deterministic order.
 
 Wheel are independent of everything, and wheel (lol) likely be on the same bus as each other. 
 
 Next steps:
	- Complete the motion planning
	- Optimize the joint speed controllers to track better
		- Tune speed controller parameters, filters, and logic
		- Some mechanical changes may be needed, particularly with the roll joints.
		- Switching joint #0 to use the timing belt would improve its performance considerably
		- Redesigning joints #4 and #5 to use the planetary mini-motors would give them less backlash and lighter weight
	- Define the controller component, and isolate it from the UI thread
	- Implement the wheel drive controllers, as well as a controller component that controls angles, velocities, etc
	- Build a better UI. Keyboard based is a good start, feedback can be via network or screen, but if a GUI is used it must be ensured there is no performance penalty
 
 
 
 */