#ifndef HATARAKI_TEST_MATHUTILTEST_HPP_
#define HATARAKI_TEST_MATHUTILTEST_HPP_

#include "MathUtils.hpp"
#include <stdexcept>
#include <iostream>
#include <cmath>

#define VMATH_NAMESPACE vmath
#include "vmath.h"

#define IKFAST_NO_MAIN
#define IKFAST_HAS_LIBRARY
#define IKFAST_NAMESPACE ikfast2
#include "ikfast.h"


using namespace ikfast;
using namespace ikfast2;
using namespace vmath;
using namespace std;

namespace HatarakiTest
{
	static void assertEquals(double expected, double actual, double precision = 0.001)
	{
		if (round(expected/precision)*precision != round(actual/precision)*precision)
		{
			stringstream ss;
			ss << "Expected [" << expected << "] but got [" << actual << "]";
			throw std::logic_error(ss.str());
		}
	}
	
	//-.8 -11.3 0.9 36.8 64.6 0.1
	//  -0.9    -11.3   +0.9    +36.8   +64.7   +0.1    
	void testEulerAngleExtraction()
	{
		Matrix3d r = Matrix3d::createRotationAroundAxis(23, 10, 122);
	
		double xR,yR,zR;
		MathUtil::extractEulerAngles(r, xR, yR, zR);
		assertEquals(23,xR);
		assertEquals(10,yR);
		assertEquals(122,zR);
	}
	
	void testAngleExtractionIK()
	{
		vector<double> angleDeg({-0.9,-11.3,0.9,36.8,64.7,0.1});
		vector<double> angleDeg2({-0.8,-11.3,0.9,36.8,64.6,0.1});
		
		double angleRad[6];
		
		for (int i=0;i<6;i++) angleRad[i] = MathUtil::degreesToRadians(angleDeg[i]);
		
		double r[9];
		double translation[3];
		ikfast2::ComputeFk(angleRad,translation,r);
		Matrix3d rotation = Matrix3d::fromRowMajorArray(r);
		
		double xR,yR,zR;
		MathUtil::extractEulerAngles(r, xR, yR, zR);
		
		cout << "Angles = " << xR << "," << yR << "," << zR << endl;
	}
}



#endif