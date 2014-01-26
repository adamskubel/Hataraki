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
//#include <math.h>
#include <stdio.h>
#include <signal.h>

#include <vector>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept> 
#include <queue>

#include "I2CBus.hpp"
#include "JointLoop.hpp"
#include "Configuration.hpp"
#include "MotionController.hpp"
#include "MathUtils.hpp"
#include "cJSON.h"

#define VMATH_NAMESPACE vmath
#include "vmath.h"

double MathUtil::PI = 3.141592653589793238462;
double MathUtil::TAU = 6.28318530718;


using namespace std;
using namespace ikfast2;

vector<JointLoop*> joints;
volatile bool running;

MotionController * motionController;



void updateController()
{
	cout << "Starting joint thread" << endl;
	while (running)
	{
		motionController->updateController();
	}
}

std::string get_file_contents(const char *filename)
{
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  if (in)
  {
    std::string contents;
    in.seekg(0, std::ios::end);
    contents.resize(in.tellg());
    in.seekg(0, std::ios::beg);
    in.read(&contents[0], contents.size());
    in.close();
    return(contents);
  }
  throw(errno);
}


void shutdownMotors()
{		
	cout << "Shutting down motors..";	
	for (auto it = joints.begin(); it != joints.end(); it++)
	{
		(*it)->shutdown();
	}		
	cout << "Done" << endl;
}

void signal_callback_handler(int signum)
{
	cout << "Caught signal" << signum << endl;

	for (auto it = joints.begin(); it != joints.end(); it++)
	{
		(*it)->requestEmergencyHalt();
	}	

	running = false;
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

void testFK()
{	
	IkReal jointAngles[6] = {0,0,0,0,0,0};
	printPositionForAngles(jointAngles);
	IkReal jointAngles2[6] = {1,-20,0,13,88,-90};
	printPositionForAngles(jointAngles2);
}


bool applyCommand(double * angles)
{
	bool validCommand = true;
	int i = 0;
	for (auto it = joints.begin(); it != joints.end(); it++,i++)
	{					
		validCommand = validCommand && (*it)->checkCommandValid(angles[i]);
	}

	if (validCommand) {
		i = 0;
		for (auto it = joints.begin(); it != joints.end(); it++,i++)
		{					
			(*it)->setTargetAngle(angles[i]);
		}
		return true;
	}

	return false;	
}

int main(int argc, char *argv[])
{
	double samplePeriod;
		
	if (argc < 2)
	{
		printf("Usage: %s <SampleFreq> \n\n",argv[0]);
		exit(1);
	}

	//signal(SIGINT, signal_callback_handler);

	samplePeriod = 1.0/atof(argv[1]);	
	long sleepTime = (long)(samplePeriod*1000000.0);
	
	cout << "Operating with period of " << samplePeriod << " seconds. " <<endl;

	cout << "Opening JSON configuration file 'config.json'...";
	std::string configString = get_file_contents("config.json");	

	cout << "Done. Parsing...";
	cJSON * configRoot = cJSON_Parse(configString.c_str());
	cout << "Done." << endl;
		
	if (configRoot)
	{
		cout << "Config Version = " << cJSON_GetObjectItem(configRoot,"Version")->valuestring << endl;
	}
	else
	{
		cout << "Error parsing configuration. Exiting." << endl;
	}

	cJSON * globalConfig = cJSON_GetObjectItem(configRoot,"GlobalSettings");

	DRV8830::MaxVoltageStep = DRV8830::voltageToSteps(cJSON_GetObjectItem(globalConfig,"MaxVoltage")->valuedouble);
	cout << "Setting maximum voltage to " << DRV8830::stepsToVoltage(DRV8830::MaxVoltageStep) << " V (Step=0x" << hex << DRV8830::MaxVoltageStep << ")" << endl;

	MotionController::PlanStepCount = cJSON_GetObjectItem(globalConfig,"MotionPlanSteps")->valueint;

	
	cJSON * dataRecordingConfig = cJSON_GetObjectItem(configRoot,"DataRecording");
	Configuration::CsvLoggingEnabled = (cJSON_GetObjectItem(dataRecordingConfig,"Enabled")->valueint != 0);


	cout << "Opening I2C bus... " << endl;
	I2CBus * bus = new I2CBus("/dev/i2c-1");
				
	cout << "Reading joints from JSON." << endl;	
	joints.clear();
	cJSON * jointArray = cJSON_GetObjectItem(configRoot,"Joints");
	for (int i=0;i<cJSON_GetArraySize(jointArray);i++)
	{
		cJSON * jointItem =cJSON_GetArrayItem(jointArray,i);

		cout << "Adding joint with name '" << cJSON_GetObjectItem(jointItem,"Name")->valuestring<< "'" << endl;
		
		JointLoop * joint = new JointLoop(bus, jointItem, samplePeriod);

		joints.push_back(joint);
	}

	motionController = new MotionController(joints,sleepTime);
		
	cJSON_Delete(configRoot);

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
						
			if (command.compare("exit") == 0 || command.compare("k") == 0) {		
				cout << "Exit command." << endl;
				running = false;
			}
			else if (command.compare("home") == 0) {
				
				double angles[6] = {0,0,0,0,0,0};

				if (!applyCommand(angles))
				{
					cout << "One or more joints is unable to accept this command. No action performed." << endl;
				}
			}
			else if (command.compare("setall") == 0) {
				
				double angles[6] = {0,0,0,0,0,0};

				int i=0;
				for (;i<6;i++)
				{
					input >> angles[i];
				}
				
				if (input.fail())
				{
					cout << "Invalid input. Usage: set <angle0> <angle1> ... <angleN-1>" << endl;
				}
				else {

					if (!applyCommand(angles))
					{
						cout << "One or more joints is unable to accept this command. No action performed." << endl;
					}
				}
			}
			else if (command.compare("set") == 0) {

				int jointIndex;
				double angle;

				input >> jointIndex;
				input >> angle;
				
				if (input.fail()) {
					cout << "Invalid input. Usage: set <index> <angle>" << endl;
				}

				if (jointIndex >= 0 && jointIndex < joints.size()){

					JointLoop * joint = joints.at(jointIndex);

					if (joint->checkCommandValid(angle)) {							
						cout << "Setting " << joint->getJointName() << " to " << angle << " degrees" << endl;
						joint->setTargetAngle(angle);				
					}
					else {
						cout << "Commanded angle " << angle << " exceeds joint range [" << joint->getMinAngle() << "," << joint->getMaxAngle() << "]" << endl;
					}
				}
				else {
					cout << "Error! Joint index " << jointIndex << " is not valid." << endl;
				}
			}
			else if (command.compare("setspeed") == 0) {
				
				int jointIndex;
				double speed;

				input >> jointIndex;

				input >> speed;

				if (input.fail()) {
					cout << "Usage: setspeed <index> <joint max speed (deg/s)>" << endl;
				}
				else {					 
					if (jointIndex < joints.size()){

						JointLoop * joint = joints.at(jointIndex);
						joint->setTargetJointVelocity(MathUtil::degreesToRadians(speed));
					}
					else {
						cout << "Error! Joint index " << jointIndex << " is not valid." << endl;
					}
				}
			}
			else if (command.compare("setgains") == 0) {

				int jointIndex;
				double kp,ki,kd;

				input >> jointIndex;

				input >> kp;
				input >> ki;
				input >> kd;

				if (input.fail()) {
					cout << "Usage: setgains <index> <Kp> <Ki> <Kd>" << endl;
				}
				else {					 
					if (jointIndex < joints.size()){

						JointLoop * joint = joints.at(jointIndex);
						cout << "Setting " << joint->getJointName() << " coefficients to " << kp << "," << ki << "," << kd << endl;
						joint->setControllerCoefficients(kp,ki,kd);
					}
					else {
						cout << "Error! Joint index " << jointIndex << " is not valid." << endl;
					}
				}
			}
			else if (command.compare("list") == 0) {

				cout << "Angles: ";
				for (auto it = joints.begin(); it != joints.end(); it++)
				{					
					cout << (*it)->getCurrentAngle() << " ";
				}
				cout << endl;
			}
			else if (command.compare("planto") == 0) {

				double coords[3];
				
				for (int i=0;i<3;i++)
				{
					input >> coords[i];
					coords[i] /= 100.0;
				}

				if (input.fail()) {
					cout << "Usage: planto <x>cm <y>cm <z>cm" << endl;
				} else {
					motionController->moveToPosition(coords);
				}
			}
			else if (command.compare("getpos") == 0) {

				double angles[6];
				int i=0;
				for (auto it = joints.begin(); it != joints.end(); it++,i++)
				{					
					angles[i] = MathUtil::degreesToRadians((*it)->getCurrentAngle());
				}

				angles[1] *= -1.0;
				angles[3] *= -1.0;
				angles[4] *= -1.0;
				
				printPositionForAngles(angles);
			}
			else if (command.compare("spike") == 0) {
				
				int jointIndex, spikeLength, repeatCount, zeroLength;
				double voltage;

				input >> jointIndex;
				input >> voltage;
				input >> spikeLength;
				input >> zeroLength;
				input >> repeatCount;

				if (input.fail() || (jointIndex >= joints.size() || jointIndex < 0)) {
					cout << "Usage: spoke <joint> <voltage> <spikeLength> (Ts) <zeroLength> (Ts) <repeat count>" << endl;
				} else {
					JointLoop * joint = joints.at(jointIndex);
					auto task = [joint,voltage,spikeLength, zeroLength, repeatCount]() {
						joint->pauseJoint();
						joint->executeSpike(voltage,spikeLength,zeroLength,repeatCount);
					};
					motionController->postTask(task);
				}

			}
			else if (command.compare("setpos") == 0) {

				double coords[3];
				
				for (int i=0;i<3;i++)
				{
					input >> coords[i];
					coords[i] /= 100.0;
				}

				if (input.fail()) {
					cout << "Usage: setpos <x>cm <y>cm <z>cm" << endl;
				} else {

					ikfast::IkSolutionList<ikfast2::IkReal> solutions;
					ikfast2::IkReal rotationMatrix[9] = {0,0,-1, 0,1,0, 1,0,0}; 
					//-90 about X = {1,0,0, 0,0,1, 0,-1,0}; //Identity={1,0,0, 0,1,0, 0,0,1};

					if (ikfast2::ComputeIk(coords,rotationMatrix,NULL, solutions)) {

						IkReal solution[6];

						for (int i=0;i<solutions.GetNumSolutions();i++)
						{		
							solutions.GetSolution(i).GetSolution(solution,NULL);
							
							for (int j=0;j<6;j++) {
								solution[j] *= (180.0/3.14159);
							}
							//Invert pitch joint angles
							solution[1] *= -1.0;
							solution[3] *= -1.0;
							solution[4] *= -1.0;
							
							
							cout << "Solution[" << i << "] = ";
							bool validCommand = true;
							int j=0;
							for (auto it = joints.begin(); it != joints.end(); it++,j++)
							{								
								bool jointOk = (*it)->checkCommandValid(solution[j]);
								validCommand = validCommand && jointOk;		

								if (jointOk)
									cout << solution[j] << " ";
								else
									cout << "[" << solution[j] << "] ";
							}
							cout << endl;

							if (validCommand) {
								cout << "This solution is valid. Proceed ([y]es/[n]ext/[a]bort)?";
								string line2;
								getline(cin,line2);

								stringstream ss2(line2);

								string ikConfirm;
								ss2 >> ikConfirm;

								if (ss2.fail()) {
									cout << "Invalid input." << endl;
									break;
								}
								else if (ikConfirm.compare("a") == 0) {
									break;
								}
								else if (ikConfirm.compare("y") == 0){
									applyCommand(solution);
									break;
								}
								else if (ikConfirm.compare("n") == 0) {
									continue;
								}
								else {
									cout << "Invalid input." << endl;
									break;
								}
							}						
						}
					}
					else
					{
						cout << "No solution found :(." << endl;
					}
				}
			}
			else 
			{
				cout << "Invalid entry. Valid commands are: exit, home, set, setall, setgains, list, getpos, setpos " << endl;
			}
		}
	}
	catch (std::runtime_error & e)
	{
		cout << "Exception: " << e.what() << endl;
		
		for (auto it = joints.begin(); it != joints.end(); it++)
		{
			(*it)->requestEmergencyHalt();
		}	

	}

	cout << "Powering down control loops..";	
	jointUpdate.join();
	cout << "Done." << endl;
	
	shutdownMotors();
}
