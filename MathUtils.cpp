#include "MathUtils.hpp"

double MathUtil::PI = 3.141592653589793238462;
double MathUtil::TAU = 6.28318530718;

double MathUtil::PI_STEPS = 8192;


//int linreg(int n, const REAL x[], const REAL y[], REAL* m, REAL* b, REAL* r)
bool MathUtil::linearRegression(std::list<std::pair<double, double> > data, double & slope, double & intercept, double & rValue)
{
	double   sumx = 0.0;                        /* sum of x                      */
	double   sumx2 = 0.0;                       /* sum of x**2                   */
	double   sumxy = 0.0;                       /* sum of x * y                  */
	double   sumy = 0.0;                        /* sum of y                      */
	double   sumy2 = 0.0;                       /* sum of y**2                   */

	double n = data.size();

	//for (int i=0;i<data.size();i++)   
	for (auto it = data.begin(); it != data.end(); it++)
	{ 
		double x = it->first;		
		double y = it->second;

		sumx  += x;       
		sumx2 += std::pow(x,2);  
		sumxy += x * y;
		sumy  += y;      
		sumy2 += std::pow(y,2); 
	} 

	double denom = (n * sumx2 - std::pow(sumx,2));
	if (denom == 0) {
		// singular matrix. can't solve the problem.
		slope = intercept = rValue = 0;
		return false;
	}

	slope = (n * sumxy  -  sumx * sumy) / denom;
	intercept = (sumy * sumx2  -  sumx * sumxy) / denom;

	rValue = (sumxy - sumx * sumy / n) /          /* compute correlation coeff     */
			std::sqrt((sumx2 - std::pow(sumx,2)/n) *
			(sumy2 - std::pow(sumy,2)/n));
	

	return true; 
}