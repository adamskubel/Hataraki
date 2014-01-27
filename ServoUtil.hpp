#ifndef HATARAKI_BASICMOTION_SERVO_UTILS
#define HATARAKI_BASICMOTION_SERVO_UTILS

#include "ServoModel.hpp"
#include "AS5048.hpp"
#include "DRV8830.hpp"
#include "I2CBus.hpp"


class ServoUtils {
public:
	static bool validateAndPrintJointFunction(I2CBus * bus, JointModel * joint);
	
};


#endif