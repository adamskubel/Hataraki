#include "QuadraticRegression.hpp"

QuadraticRegression::QuadraticRegression(int maxSize)
{
	this->numOfEntries = 0;
	this->maxSize = maxSize;
	this->calculationNeeded = true;
	pointArray = new std::list<std::pair<double,double> >();
}

void QuadraticRegression::addPoint(double x, double y) 
{
	pointArray->push_back(std::pair<double,double>(x,y));
	numOfEntries++;
	
	if (numOfEntries > maxSize)
	{
		numOfEntries--;
		pointArray->pop_front();
	}
	expire();
}

void QuadraticRegression::clearAll()
{
	pointArray->clear();
	numOfEntries = 0;
	expire();
}

int QuadraticRegression::getSize()
{
	return numOfEntries;
}

void QuadraticRegression::expire()
{	
	calculationNeeded = true;
}

double QuadraticRegression::aTerm()
{
	calculate();
	return lastA;
}

double QuadraticRegression::bTerm()
{
	calculate();
	return lastB;
}

double QuadraticRegression::cTerm()
{
	calculate();
	return lastC;
}

double QuadraticRegression::rSquare() // get r-squared
{
	calculate();
	return lastR2;
}


double QuadraticRegression::y(double x)
{
	calculate();
	x-= pointArray->front().first;
	return lastA * std::pow(x,2) + lastB * x + lastC;
}

double QuadraticRegression::dy(double x)
{
	calculate();
	x-= pointArray->front().first;
	return 2.0 * lastA * x + lastB;
}

double QuadraticRegression::ddy(double x)
{
	calculate();
	return 2.0 * lastA;
}


void QuadraticRegression::calculate()
{
	if (!calculationNeeded) return;	
	if (numOfEntries < 3) throw std::runtime_error("ur a faget");
	
	calculationNeeded = false;	

	double Sy = 0;
	double Sx = 0, Sx2 = 0, Sx3 = 0, Sx4 = 0;
	double Sxy = 0, Sx2y = 0;
	for (auto it = pointArray->begin(); it != pointArray->end(); it++)
	{
		double x = it->first, y = it->second;
		
		x -= pointArray->front().first;
		double x2 = std::pow(x, 2);
			
		Sy += y;						
		Sx += x;

		Sxy += x * y;
		Sx2y += x2 * y;		

		Sx2 += x2;
		Sx3 += x2*x; //sign?
		Sx4 += std::pow(x2, 2);		
	}

	double s40 = Sx4, s30 = Sx3, s20 = Sx2, s10 = Sx, s00 = numOfEntries;
	double s21 = Sx2y, s11 = Sxy, s01 = Sy;

	double denom = (s40*(s20 * s00 - s10 * s10) -
		s30*(s30 * s00 - s10 * s20) + 
		s20*(s30 * s10 - s20 * s20));

	lastA = (s21*(s20 * s00 - s10 * s10) - 
		s11*(s30 * s00 - s10 * s20) + 
		s01*(s30 * s10 - s20 * s20))/denom;

	lastB = (s40*(s11 * s00 - s01 * s10) - 
		s30*(s21 * s00 - s01 * s20) + 
		s20*(s21 * s10 - s11 * s20))/denom;

	lastC = (s40*(s20 * s01 - s10 * s11) - 
		s30*(s30 * s01 - s10 * s21) + 
		s20*(s30 * s11 - s20 * s21))/denom;

	double ssErr = 0, ssTot = 0;
	double yMean = Sy/numOfEntries;
	for (auto it = pointArray->begin(); it != pointArray->end(); it++)
	{
		double x = it->first, y = it->second;
		ssErr += std::pow(y - this->y(x), 2);
		ssTot += std::pow(y - yMean, 2);
	}
	if (ssTot == 0)
		lastR2 = 1;
	else
		lastR2 = 1.0 - (ssErr/ssTot);
	
	if (lastR2 < -1.0)
		lastR2 = -1.0;
	
}