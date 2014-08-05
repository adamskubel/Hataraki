#ifndef HATARAKI_BASICMOTION_DEVICEUTIL_DRV8830_HPP
#define HATARAKI_BASICMOTION_DEVICEUTIL_DRV8830_HPP

#include <iostream>

#include "I2CDevice.hpp"

#define MAX_OFF_VOLTAGE 0.01
#define MIN_DRIVER_VOLTAGE_STEP 6

namespace DriveMode
{
	const int OFF = 0x00;
	const int FORWARD = 0x01;
	const int BACKWARD = 0x02;
	const int BRAKE = 0x03;
}

namespace DRV8830Registers
{
	const int CONTROL = 0x00;
	const int FAULT = 0x01;
}


class DRV8830 : public I2CDevice {
	
public:
	DRV8830(I2CBus * bus, int address);
	void writeVoltage(double voltage);
	
	static int MaxVoltageStep; 
	static double MinDriverVoltage;

	static int voltageToSteps(double input);
	static double stepsToVoltage(int voltageSteps);	
	
	static double voltageToFractionalSteps(double input);
	static double fractionalStepsToVoltage(double input);

	static double getNearestVoltage(double voltage);
	
private:
	static int buildCommand(double voltageMagnitude, int mode);
	static int buildCommand(double voltage);

	void writeCommand(unsigned char command);
	void writeVoltageMode(double voltage, int mode);

	unsigned char readFaultRegister();
	bool hasFault(unsigned char faultRegisterValue);
	void printFaultRegister(unsigned char faultRegisterValue);

};

#endif