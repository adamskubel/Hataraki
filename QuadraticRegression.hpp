#ifndef HATARAKI_BASICMOTION_MATH_QUADRATIC_REGRESSION_HPP_
#define HATARAKI_BASICMOTION_MATH_QUADRATIC_REGRESSION_HPP_

#include <cmath>
#include <list>
//#include <vector>
#include <stdexcept>
#include <iostream>

class QuadraticRegression
{
private:
	std::list<std::pair<double, double> > * pointArray;
	int numOfEntries, maxSize; 
	bool calculationNeeded;

	double lastA, lastB, lastC, lastR2;

	void expire();
	void calculate();

public:

	QuadraticRegression(int maxSize);

	void addPoint(double x, double y);	
	void clearAll();
	int getSize();

	//(a^2)x + bx + c
	double aTerm();
	double bTerm();
	double cTerm();
	double rSquare();

	double y(double x);
	double dy(double x);
	double ddy(double x);


};

#endif