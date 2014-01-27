#ifndef HATARAKI_BASICMOTION_MATH_UTILS_HPP_
#define HATARAKI_BASICMOTION_MATH_UTILS_HPP_


#include <time.h>

namespace MathUtils {
	
	template <typename T> int sgn(T val) {
		return (T(0) < val) - (val < T(0));
	}
};

class MathUtil {

public:
	static double PI;
	static double TAU;
	
	static double PI_STEPS;	
	static double PI_DEGREES;;
	
	

	static double abs(double a) {

		if (a < 0)
			return -a;
		return a;
	}
	
	
	static double lowpass(double previous, double input, double alpha)
	{
		return alpha * input + (1.0 - alpha) * previous;
	}

	static double lowpass(double previous, double input, double RC, double dT)
	{
		double alpha = dT/(RC + dT);
		return lowpass(previous,input,alpha);
	}

	static double getTimeDelta(struct timespec & t0,struct timespec & t1)
	{
		long nsDiff = t1.tv_nsec - t0.tv_nsec;
		long secDiff = t1.tv_sec - t0.tv_sec;

		double totalTime = nsDiff/1000000000.0;
		totalTime += secDiff;
		
		return totalTime;
	}

	static double subtractAngles(double currentAngleRad,double targetAngleRad)
	{
		return subtractAngles(currentAngleRad,targetAngleRad,MathUtil::PI);
	}
	
	static double subtractAngles(double minuend,double subtrahend, double pi)
	{
		double difference = minuend - subtrahend;
		
		if (difference > pi)
			return difference - (pi*2);
		else if (difference < -pi)
			return difference + pi*2;
		else
			return difference;
	}


	static int offsetAngleSteps(int angle, int zeroPosition) 
	{
		int newAngle = angle - zeroPosition;

		if (newAngle < -AS5048::PI_STEPS)
			newAngle += AS5048::TAU_STEPS;
		else if (newAngle > AS5048::PI_STEPS)
			newAngle -= AS5048::TAU_STEPS;

		return newAngle;
	}

	static double degreesToRadians(double angleDeg){
		return angleDeg*(PI/180.0);
	}

	static double radiansToDegrees(double angleDeg){
		return angleDeg*(180.0/PI);
	}

	static double rpmToRadPerSecond(double rpm) {
		return (rpm*2.0*PI)/60.0;
	}

	static double radPerSecondToRPM(double radPerSecond) {
		return (radPerSecond*60.0)/2.0*PI;
	}

	static void addVectors(const double * v0, const double * v1, double * v01) {
		for (int i=0;i<3;i++){
			v01[i] = v0[i] + v1[i];
		}
	}

	static void subtractVectors(const double * v0, const double * v1, double * v01) {
		for (int i=0;i<3;i++){
			v01[i] = v0[i] - v1[i];
		}
	}

	static void scaleVector(const double * v0, const double scale, double * vScale) {
		for (int i=0;i<3;i++){
			vScale[i] = v0[i] * scale;
		}
	}

	static void setNow(struct timespec & sinceTime) {		
		clock_gettime(CLOCK_REALTIME, &now);
	}

	
	static double timeSince(struct timespec & sinceTime){
		struct timespec now;
		clock_gettime(CLOCK_REALTIME, &now);
		return getTimeDelta(sinceTime,now);
	}


};

#endif
