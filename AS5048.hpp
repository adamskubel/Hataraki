#ifndef HATARAKI_BASICMOTION_DEVICEUTIL_AS5048
#define HATARAKI_BASICMOTION_DEVICEUTIL_AS5048

#include <iostream>

#include "I2CBus.hpp"

#define MAX_ANGLE 16384


#define DIAG_COMP_HIGH 0x08
#define DIAG_COMP_LOW 0x04
#define DIAG_CORDIC_OVERFLOW 0x02
#define DIAG_OFFSETCOMPFINISHED 0x01

namespace AS5048Registers
{
	const unsigned char AUTOGAINCNTRL = 251;//250;
	const unsigned char DIAGNOSTICS = 252;//251;
	const unsigned char MAGNITUDE = 253;//252;
	const unsigned char ANGLE = 255;//254;
}

class AS5048 {

public:
	static int TAU_STEPS;
	static int PI_STEPS;

	static int getSensorAngleSteps(I2CBus*);
	//static double getSensorAngle(I2CBus*);

	static double degreesToSteps(double angle);
	static double stepsToDegrees(double steps);
	
	static double radiansToSteps(double angle);
	static double stepsToRadians(double steps);

	static int getSensorMagnitude(I2CBus*);

	static unsigned char getDiagnosticFlags(I2CBus*);
	static unsigned char getAutoGainValue(I2CBus*);

	static bool isValidStatus(unsigned char diagnosticFlags);

	static void printDiagnosticFlags(unsigned char diagnosticFlags);

};

#endif