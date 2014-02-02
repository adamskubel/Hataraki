#include "DRV8830.hpp"
#include <cmath>

int DRV8830::MaxVoltageStep = 0x3F;
double DRV8830::MinDriverVoltage = 0.49;

using namespace std;

int DRV8830::voltageToSteps(double input)
{
	//v = 4 x 1.285 x (VSET +1) / 64
	//vset = v*(64/(4*1.285)) - 1
	
	if (input < 0)
		input = -input;
	
	int vset = (int)std::round(input * (16.0/1.285));
	
	if (vset > DRV8830::MaxVoltageStep) 
		vset = DRV8830::MaxVoltageStep;

	return vset;
}

unsigned char DRV8830::readFaultRegister(I2CBus * bus)
{
	unsigned char buf[1] = {DRV8830Registers::FAULT};
	bus->writeToBus(buf,1);
	
	unsigned char result[1] = {0};
	bus->readFromBus(result,1);

	return result[0];
}

double DRV8830::getNearestVoltage(double voltageTarget)
{
	double sign = 1;
	if (voltageTarget < 0)
		sign = -1;

	return stepsToVoltage(voltageToSteps(voltageTarget))*sign;
}

bool DRV8830::hasFault(unsigned char faultRegisterValue)
{
	return (faultRegisterValue & 0x01 != 0);
}

void DRV8830::printFaultRegister(unsigned char faultRegisterValue)
{
	cout << "Register value = 0x" << hex << (int)faultRegisterValue << ". Fault flags: ";
	if (faultRegisterValue & 0x01 != 0)
	{
		if (faultRegisterValue & 0x02 != 0)
			cout << "Overcurrent | ";
		if (faultRegisterValue & 0x04 != 0)
			cout << "Undervoltage lockout | ";
		if (faultRegisterValue & 0x08 != 0)
			cout << "Overtemperature condition | ";
		if (faultRegisterValue & 0x10 != 0)
			cout << "Current Limit";
	}
	else
	{
		cout << " none!";
	}
	cout << endl;
}

double DRV8830::stepsToVoltage(int vCmd)
{
	return (1.285/16.0) * ((double)vCmd);
}

double DRV8830::fractionalStepsToVoltage(double vSteps)
{
	return (1.285/16.0) * (vSteps);
}

double DRV8830::voltageToFractionalSteps(double voltage)
{
	return voltage/(1.285/16.0);
}

//void DRV8830::buildVoltageDitherArray(double vTarget,int * vCmdArray, int cycleLength,int cycleCount)
//{
//	int vCmdLow = voltageToSteps(vTarget);
//	int vCmdHigh = vCmdLow+1;
//	if (vCmdHigh > 0x3F)
//		vCmdHigh = 0x3F;
//	
//	double vLow = stepsToVoltage(vCmdLow);
//	double vHigh = stepsToVoltage(vCmdHigh);
//	
//	//Vtarget = a*Vlow+(1-a)*Vhigh
//	//Vt = a*Vlow + Vhigh - a*Vhigh
//	//Vt = a*(Vlow-Vhigh) + Vhigh
//	//(Vt - Vhigh)/a = Vlow-Vhigh
//	//a = (Vt-Vhigh)/(Vlow-Vhigh)
//	
//	double alpha = (vTarget-vHigh)/(vLow-vHigh);
//	
//	int lowCycles = alpha * cycleLength;
//	int highCycles = cycleLength - lowCycles;
//	
//	int cycle = 0, i = 0;
//	for (; cycle < cycleCount; cycle++)
//	{
//		int pos = 0;
//		for (;pos < cycleLength;i++,pos++)
//		{
//			if (pos < lowCycles)
//				vCmdArray[i] = vCmdLow;
//			else
//				vCmdArray[i] = vCmdHigh;
//		}
//	}
//}

int DRV8830::buildCommand(double voltageMag, int mode)
{
	int command = voltageToSteps(voltageMag);
	command = (command << 2) | mode;

	return command;
}

int DRV8830::buildCommand(double voltage)
{	
	int mode = DriveMode::FORWARD;

	if (voltage > MAX_OFF_VOLTAGE) 
		mode = DriveMode::FORWARD;
	else if (voltage < -MAX_OFF_VOLTAGE)
		mode = DriveMode::BACKWARD;

	int command = voltageToSteps(voltage);
	command = (command << 2) | mode;

	return command;
}


void DRV8830::writeVoltage(I2CBus * bus, double voltage)
{
	int mode = DriveMode::FORWARD;

	if (voltage > MAX_OFF_VOLTAGE) 
		mode = DriveMode::FORWARD;
	else if (voltage < -MAX_OFF_VOLTAGE)
		mode = DriveMode::BACKWARD;

	writeVoltageMode(bus,voltage,mode);
}


void DRV8830::writeVoltageMode(I2CBus * bus, double voltage, int mode)
{
	int command = buildCommand(voltage,mode);

	unsigned char buffer[2] = {DRV8830Registers::CONTROL,command};
	bus->writeToBus(buffer,2);
}

void DRV8830::writeCommand(I2CBus * bus,int command)
{
	unsigned char buffer[2] = {DRV8830Registers::CONTROL,command};
	bus->writeToBus(buffer,2);
}
