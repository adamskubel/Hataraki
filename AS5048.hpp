#ifndef HATARAKI_BASICMOTION_DEVICEUTIL_AS5048
#define HATARAKI_BASICMOTION_DEVICEUTIL_AS5048

#include <iostream>

#include "I2CBus.hpp"
#include "MathUtils.hpp"


#define MAX_ANGLE 16384


#define DIAG_COMP_HIGH 0x08
#define DIAG_COMP_LOW 0x04
#define DIAG_CORDIC_OVERFLOW 0x02
#define DIAG_OFFSETCOMPFINISHED 0x01

namespace AS5048Registers
{
	static int AUTOGAINCNTRL = 251;//250;
	static int DIAGNOSTICS = 252;//251;
	static int MAGNITUDE = 253;//252;
	static int ANGLE = 255;//254;
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

	static int getAngleError(int currentAngle, int targetAngle);	

};

#endif