#include "DRV8830.hpp"
#include <cmath>

int DRV8830::MaxVoltageStep = 0x3F;
double DRV8830::MinDriverVoltage = 0.49;

using namespace std;


DRV8830::DRV8830(I2CBus * bus, int address) : I2CDevice(bus,address)
{
	
}


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

unsigned char DRV8830::readFaultRegister()
{	
	unsigned char result = static_cast<unsigned char>(readByte(1));
	return result;
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
	return ((faultRegisterValue & 0x01) != 0);
}

void DRV8830::printFaultRegister(unsigned char faultRegisterValue)
{
	cout << "Register value = 0x" << hex << (int)faultRegisterValue << ". Fault flags: ";
	if ((faultRegisterValue & 0x01) != 0)
	{
		if ((faultRegisterValue & 0x02) != 0)
			cout << "Overcurrent | ";
		if ((faultRegisterValue & 0x04) != 0)
			cout << "Undervoltage lockout | ";
		if ((faultRegisterValue & 0x08) != 0)
			cout << "Overtemperature condition | ";
		if ((faultRegisterValue & 0x10) != 0)
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
	return fractionalStepsToVoltage((double)vCmd);
}

double DRV8830::fractionalStepsToVoltage(double vSteps)
{
	double v = (1.285/16.0) * (vSteps);
	v = std::round(v/0.01)*0.01;
	return v;
}

double DRV8830::voltageToFractionalSteps(double voltage)
{
	return voltage/(1.285/16.0);
}

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


void DRV8830::writeVoltage(double voltage)
{
	int mode = DriveMode::FORWARD;

	if (voltage > MAX_OFF_VOLTAGE) 
		mode = DriveMode::FORWARD;
	else if (voltage < -MAX_OFF_VOLTAGE)
		mode = DriveMode::BACKWARD;

	writeVoltageMode(voltage,mode);
}


void DRV8830::writeVoltageMode(double voltage, int mode)
{
	writeCommand(buildCommand(voltage,mode));
}

void DRV8830::writeCommand(unsigned char command)
{
	writeByte(DRV8830Registers::CONTROL, command);
}


