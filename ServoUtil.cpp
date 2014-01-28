#include "ServoUtil.hpp"

using namespace std;


bool ServoUtils::validateAndPrintJointFunction(I2CBus * bus, JointModel * joint)
{
	ServoModel * servo = &joint->servoModel;
	
	bool encoderReady = false;
	bool driverReady = false;
	
	cout << "------------------------------------------------" << endl;
	cout << "Initializing joint '" << joint->name << "'" << endl;
	
	//AS5048
	
	bus->selectAddress(servo->sensorAddress);
	
	try
	{
		int rawAngle = AS5048::getSensorAngleSteps(bus);
		int angle = MathUtil::offsetAngleSteps(rawAngle,joint->sensorZeroPosition);
		unsigned char diagnosticFlags = AS5048::getDiagnosticFlags(bus);
		unsigned char agc = AS5048::getAutoGainValue(bus);
		
		cout
			<< "Angle = " << AS5048::stepsToDegrees(angle)
			<< " RawAngle = " << AS5048::stepsToDegrees(rawAngle)
			<< " GainValue = " << dec << (int)agc
			<< " DiagRegister = " << hex << (int)diagnosticFlags
			<< endl;
		
		if (angle < joint->minAngle || angle > joint->maxAngle)
		{
			cout << "Encoder is reporting an angle out of range." << endl;
		}
		else if (!AS5048::isValidStatus(diagnosticFlags)) {
			cout << "Encoder is reporting an invalid condition." << endl;
			AS5048::printDiagnosticFlags(diagnosticFlags);
		}
		else
		{
			//cout << "Encoder initialized." << endl;
			encoderReady = true;
		}
				
	}
	catch (std::runtime_error & e)
	{
		cout << "Exception while trying to communicate with sensor: " << e.what() << endl;
	}
	
	
	//DRV8830
	try
	{
		bus->selectAddress(servo->driverAddress);
		unsigned char driverFaults = DRV8830::readFaultRegister(bus);
		
		if (!DRV8830::hasFault(driverFaults))
		{
			//cout << "Driver initialized." << endl;
			driverReady = true;
		}
		else
		{
			cout << "Driver is in FAULT state. Register value = 0x" << hex << (int)driverFaults << endl;
			DRV8830::printFaultRegister(driverFaults);
		}
	}
	catch (runtime_error & e)
	{
		cout << "Exception while trying to communicate with driver: " << e.what() << endl;
		cout << "Joint is in error state and will not function." << endl;
	}
	
	cout << "------------------------------------------------" << endl << endl;
	
	return encoderReady && driverReady;
}

