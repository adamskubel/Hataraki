#ifndef HATARAKI_BASICMOTION_MATH_UTILS_HPP_
#define HATARAKI_BASICMOTION_MATH_UTILS_HPP_


#include <cmath>
#include <limits>
#include <vector>
#include <list>

#define VMATH_NAMESPACE vmath
#include "vmath.h"


//namespace MathUtils {

	template <typename T> int sgn(T val) {
		return (T(0) < val) - (val < T(0));
	}

	
//};

class MathUtil {

public:
	static double PI;
	static double TAU;
	
	static double PI_STEPS;	
	//static double PI_DEGREES;
	
	

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
	
	static double rpmToStepsPerSecond(double rpm) {
		return (rpm*2.0*PI_STEPS)/60.0;
	}

	static double stepsPerSecondToRPM(double stepsPerSecond) {
		return (stepsPerSecond*60.0)/(2.0*PI_STEPS);
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

	static bool linearRegression(std::list<std::pair<double, double> > data, double & slope, double & intercept, double & rValue);
	
	static vmath::Vector3d projectOntoPlane(vmath::Vector4d plane,  vmath::Vector3d point);

	static vmath::Vector4d constructPlane(vmath::Vector3d normal, vmath::Vector3d point);

	static void getRowMajorData(vmath::Matrix3d matrix, double * result);
	
	static void extractEulerAngles(vmath::Matrix3d matrix, double & xR, double & yR, double & zR);
	
	static vmath::Matrix3d createQuaternionRotation(double xDeg, double yDeg, double zDeg);

};

#endif
