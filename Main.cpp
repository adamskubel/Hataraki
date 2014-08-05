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
#include "Configuration.hpp"
#include "MathUtils.hpp"
#include "AsyncLogger.hpp"
#include "AsyncI2CSender.hpp"
#include "ThreadUtils.hpp"
#include "ALog.hpp"
#include "RealtimeLoopController.hpp"
#include "TemperatureController.hpp"
#include "DRV8830.hpp"
#include "ADS1015.hpp"

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


int main(int argc, char *argv[])
{
	cout << "STARTING TempMotion" << endl;
	std::string configFileName = "config.json";
	
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
		busMap = initializeI2C();
	}
	catch (ConfigurationException & confEx)
	{
		cout << "Configuration error: " << confEx.what() << endl;
		cout << "Exiting." << endl;
		return 1;
	}
	cout << std::setbase(10) << 1.0 << endl;
	
	ADS1015 * adc = new ADS1015(busMap["i2c-3"],73);
	DRV8830 * heaterDriver = new DRV8830(busMap["i2c-3"],100);
	
	TemperatureController * tempController = new TemperatureController(adc,heaterDriver);
	
	RealtimeLoopController controlLoop;
	controlLoop.addWatcher(tempController);

	//Start everything
	AsyncLogger::getInstance().startThread();
	controlLoop.start();
	running = true;
		
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
				else if (command.compare("temp") == 0)
				{
					RealtimeDispatcher::getInstance().postTask([tempController](){
						tempController->printStatus();
					});
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
	controlLoop.stop();
	cout << "Done." << endl;
	
	return 0;

	AsyncI2CSender::cleanup();
	
	AsyncLogger::getInstance().joinThread();
	
//	alog->sync();
}
