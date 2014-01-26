#ifndef HATARAKI_BASICMOTION_SPEEDCONTROLLER_HPP_
#define HATARAKI_BASICMOTION_SPEEDCONTROLLER_HPP_

#include "MathUtils.hpp"
#include "DRV8830.hpp"
#include "SimpleMovingAverage.hpp"

class SpeedController {

private:

	double Kp,Ki,Kd, motorConstant;

	double currentVoltage;

	int lastAngle;

	double targetSpeed, errorSum, lastSpeed;

	bool closedLoopControlActive, closedLoopControlEnabled;

	double lowpassFilterRC;

	SimpleMovingAverage * movingAverage;

public:

	SpeedController(double Kp, double Ki, double Kd, double motorConstant, double filterRC);

	void setTargetSpeed(double targetSpeed);
	double getTargetSpeed();

	double getLastSpeed();
	
	void updateAngle(double elapsedTime, int newAngle);

	double getNextVoltage();

	void enableClosedLoopControl(bool enable);



};


#endif