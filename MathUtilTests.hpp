#ifndef HATARAKI_TEST_MATHUTILTEST_HPP_
#define HATARAKI_TEST_MATHUTILTEST_HPP_

#include "MathUtils.hpp"
#include <stdexcept>
#include <iostream>
#include <cmath>

#define VMATH_NAMESPACE vmath
#include "vmath.h"

#include "IKFast.hpp"

#include "TestUtil.hpp"


using namespace ikfast;
using namespace vmath;
using namespace std;

namespace HatarakiTest
{
	
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
		vector<double> angleDeg2({-0.8,-11.3,0.9,36.8,64.6,60.1});
		
		double angleRad[6];
		
		for (int i=0;i<6;i++) angleRad[i] = MathUtil::degreesToRadians(angleDeg2[i]);
		
		double r[9];
		double translation[3];
		ComputeFk(angleRad,translation,r);
		Matrix3d rotation = Matrix3d::fromRowMajorArray(r);
		
		double xR,yR,zR;
		MathUtil::extractEulerAngles(rotation, xR, yR, zR);
		
		cout << "Angles = " << xR << "," << yR << "," << zR << endl;
	}
	
	void testIKRotation()
	{
		Vector3d pos(10,0,-6);
		pos /= 100.0;
		
		Matrix3d rotation = Matrix3d::createRotationAroundAxis(0,-90,0);
		rotation = rotation * Matrix3d::createRotationAroundAxis(0, 0, 10);
		
		double r[9];
		MathUtil::getRowMajorData(rotation, r);
		
		IkSolutionList<IkReal> solutions;
		timespec ikStart;

		TimeUtil::setNow(ikStart);
		for (int i=0;i<1000; i++)
		{
			ComputeIk(pos,r,NULL,solutions);
		}
		cout << "IK time = " << TimeUtil::timeSince(ikStart) << " ms" << endl;


		//for (int i=0;i<solutions.GetNumSolutions();i++)
		{
			int i=0;
			double solution[6];
			double freeVals[] = {1,2,3};
			solutions.GetSolution(i).GetSolution(solution,freeVals);
			
			Vector3d fkPos;
			double fkRot[9];
			ComputeFk(solution, fkPos, fkRot);
			
			Matrix3d m = Matrix3d::fromRowMajorArray(fkRot);
			
			cout << m * Vector3d(1,0,0) << endl;
			
			double xR,yR,zR;
			MathUtil::extractEulerAngles(m, xR, yR, zR);
			//cout << "Position = " << fkPos.toString() << endl;
			cout << "Angles = " << xR << "," << yR << "," << zR << endl;
			
		}
	}
}

#endif






























