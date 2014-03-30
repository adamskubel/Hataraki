#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_1
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_2
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_4
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_8

#include <unistd.h>

#include <thread>
#include <sstream>

#include "I2CBus.hpp"
#include "DRV8830.hpp"
#include "TimeMultiplexedVoltageConverter.hpp"
#include "TimeUtil.hpp"

I2CBus * driverBus;
volatile bool running;
volatile double targetVoltage;
volatile long updatePeriod;

int driverAddress;

using namespace std;

void voltageControlUpdate()
{
	TimeMultiplexedVoltageConverter tm(3,3.6);
	timespec lastTime = TimeUtil::getNow();

	driverBus->selectAddress(driverAddress);
	
	while (running)
	{
		timespec start = TimeUtil::getNow();

		double voltage = tm.nextVoltage(TimeUtil::timeSince(lastTime),targetVoltage);
		DRV8830::writeVoltage(driverBus,voltage);

		
		double totalTime = TimeUtil::timeSince(start);
		long adjustedSleep = updatePeriod - (totalTime*1000000);
		if (adjustedSleep > 0 && adjustedSleep <= updatePeriod)
			usleep(static_cast<unsigned int>(adjustedSleep));
	}

	DRV8830::writeVoltageMode(driverBus,0,DriveMode::OFF);
}


int main(int argc, char *argv[])
{
	if (argc < 3)
	{
		cout << "Usage: VoltageTest <driver-bus> <driver address>" << endl;
		return 1;
	}
	
	string busName = string(argv[1]);
	driverAddress = stoi(string(argv[2]));
	
	driverBus = new I2CBus(busName);
	
	running = true;
	targetVoltage = 0;
	updatePeriod = (long)(1000000.0/400.0);
	
	thread voltageThread(voltageControlUpdate);

	while (running)
	{
		cout << "->>";

		string inputString;
		getline(cin,inputString);

		stringstream input(inputString);

		string command;
		input >> command;

		if (command.compare("exit") == 0 || command.compare("k") == 0) 
		{			
			running = false;
		}
		else if (command.compare ("set")  == 0 || command.compare("s") == 0)
		{
			double newVoltage;
			input >> newVoltage;

			if (input.fail()) cout << "Usage: set <voltage>" << endl;
			else 
			{
				cout << "Setting voltage to " << newVoltage << " V" << endl;
				targetVoltage = newVoltage;
			}
		}
		else if (command.compare ("freq")  == 0 || command.compare("sf") == 0)
		{
			double newFreq;
			input >> newFreq;

			if (input.fail()) cout << "Usage: freq <frequency>" << endl;
			else 
			{
				cout << "Setting update frequency to " << newFreq << endl;
				updatePeriod = (long)(1000000.0/newFreq);				
			}
		}
		else
		{
			cout << "Invalid command: " << command << endl;
		}
	}

	voltageThread.join();
	return 0;
}