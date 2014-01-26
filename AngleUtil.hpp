#ifndef HATARAKI_BASICMOTION_ANGLE_UTIL_HPP_
#define HATARAKI_BASICMOTION_ANGLE_UTIL_HPP_

#include <cmath>


class AngleUtil {
	
public:
	static AngleUtil Degrees;
	static AngleUtil Steps;
	static AngleUtil Radians;

	static double PiRadians = 3.14159;
	static double PiSteps = 8192;
	static double PiDegrees = 180;

private:
	double radianRatio; //units per radian
	double degreesRatio; //units per degree
	double stepRatio; //units per step

	double unitPi; //180 degrees in units
	double unitTau; //2x unitPi

	AngleUtil(double _unitPi) {
		this->unitPi = _unitPi;
		
		unitTau = unitPi*2.0;
		radianRatio = unitPi/PiRadians;
		degreesRatio = unitPi/PiDegrees;
		stepRatio = unitPi/PiSteps;
	}

	double toRadians(double angle) {
		return angle/radianRatio;
	}

	double toDegrees(double angle) {
		return angle/degreesRatio;
	}

	double toSteps(double angle) {
		return angle/stepRatio;
	}

	double subtractAngles(double minuend,double subtrahend)
	{
		double difference = minuend - subtrahend;
		return wrapAngle(difference);
	}

	double normalize(double angle) {		
		if (angle > unitPi)
			return angle - unitTau;
		else if (angle < -unitPi)
			return angle + unitTau;
		else
			return angle;
	}

	
};


#endif
