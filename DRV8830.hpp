#ifndef HATARAKI_BASICMOTION_DEVICEUTIL_DRV8830_HPP
#define HATARAKI_BASICMOTION_DEVICEUTIL_DRV8830_HPP

#include <iostream>

#include "I2CBus.hpp"

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
	static int CONTROL = 0x00;
	static int FAULT = 0x01;
}

//struct FaultRegister {
//	unsigned char HasFault : 1;
//	unsigned char 
//};

//struct DRV8830Command {
//
//	int Voltage;
//	HBridgeMode Mode;
//	
//	DRV8830Command() {
//		Voltage = 0;
//		Mode = 0;
//	}
//
//	DRV8830Command(int _Voltage, HBridgeMode _Mode) :
//		Voltage(_Voltage),
//		Mode(_Mode)
//	{
//
//	}
//
//
//};

//enum HBridgeMode {
//	Open,
//	Forward,
//	Reverse,
//	Short
//	
//};

class DRV8830 {

public:
	static int MaxVoltageStep; 
	static double MinDriverVoltage;

	static int voltageToSteps(double input);
	static double stepsToVoltage(int voltageSteps);	
	
	static double voltageToFractionalSteps(double input);
	static double fractionalStepsToVoltage(double input);

	static double getNearestVoltage(double voltage);

	static int buildCommand(double voltageMagnitude, int mode);
	static int buildCommand(double voltage);

	static void writeCommand(I2CBus * bus,int command);

	//static void buildVoltageDitherArray(double vTarget,int * vCmdArray, int cycleLength,int cycleCount);

	static void writeVoltage(I2CBus*bus, double voltage);
	static void writeVoltageMode(I2CBus*bus, double voltage, int mode);

	static unsigned char readFaultRegister(I2CBus*bus);
	static bool hasFault(unsigned char faultRegisterValue);
	static void printFaultRegister(unsigned char faultRegisterValue);

};

#endif