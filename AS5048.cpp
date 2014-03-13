#include "AS5048.hpp"

#include "MathUtils.hpp"

int AS5048::TAU_STEPS = 16384;
int AS5048::PI_STEPS = 8192;

using namespace std;

int AS5048::getSensorAngleSteps(I2CBus * bus)
{	
	unsigned char buf[1] = {AS5048Registers::ANGLE};
	bus->writeToBus(buf,1);
	
	unsigned char result[2] = {0,0};
	bus->readFromBus(result,2);
	
	int angle = ((int)result[0]) << 6;
	angle += (int)result[1];
	
	return angle;
}

int AS5048::getSensorMagnitude(I2CBus * bus)
{
	unsigned char buf[1] = {AS5048Registers::MAGNITUDE};
	bus->writeToBus(buf,1);
	
	unsigned char result[2] = {0,0};
	bus->readFromBus(result,2);
	
	int mag = ((int)result[0]) << 6;
	mag += (int)result[1];
	
	return mag;
}

double AS5048::degreesToSteps(double degrees)
{
	return ((double)MAX_ANGLE) * (degrees/360.0);
}

double AS5048::stepsToDegrees(double steps)
{
	return (steps*360.0)/((double)MAX_ANGLE);
}

double AS5048::radiansToSteps(double radians)
{
	return ((double)MAX_ANGLE) * (radians/MathUtil::TAU);
}

double AS5048::stepsToRadians(double steps)
{
	return (steps*MathUtil::TAU)/((double)MAX_ANGLE);
}



unsigned char AS5048::getAutoGainValue(I2CBus* bus)
{	
	unsigned char buf[1] = {AS5048Registers::AUTOGAINCNTRL};
	unsigned char result[1] = {0};
		
	bus->writeToBus(buf,1);
	bus->readFromBus(result,1);

	return result[0];
}

unsigned char AS5048::getDiagnosticFlags(I2CBus*bus)
{
	unsigned char buf[1] = {AS5048Registers::DIAGNOSTICS};
	unsigned char result[1] = {0};

	bus->writeToBus(buf,1);	
	bus->readFromBus(result,1);	

	return result[0];
}

bool AS5048::isValidStatus(unsigned char diagnosticFlags)
{
	return  
		((diagnosticFlags & DIAG_COMP_LOW) == 0) && 
		((diagnosticFlags & DIAG_COMP_HIGH) == 0) &&
		((diagnosticFlags & DIAG_CORDIC_OVERFLOW) == 0);
}

void AS5048::printDiagnosticFlags(unsigned char diagnosticFlags)
{		
	cout << "Diagnostic register = " << hex << (unsigned int)diagnosticFlags << endl;
	
	cout << "Flags: ";

	if ((diagnosticFlags & DIAG_COMP_LOW) != 0)
		cout << "Comp low | (" << (diagnosticFlags & DIAG_COMP_LOW) << ") - ";
	if ((diagnosticFlags & DIAG_COMP_HIGH) != 0)
		cout << "Comp high | ";
	if ((diagnosticFlags & DIAG_CORDIC_OVERFLOW) != 0)
		cout << "Cordic overflow | ";
	if ((diagnosticFlags & DIAG_OFFSETCOMPFINISHED) != 0)
		cout << "OCF";

	cout << endl;
}
