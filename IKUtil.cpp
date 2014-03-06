#include "IKUtil.hpp"

//void printPositionForAngles(IkReal * jointAngles) {
//	
//	IkReal t[3];
//	IkReal r[9];
//	
//	ikfast2::ComputeFk(jointAngles,t,r);
//	
//	Matrix3d rotationMatrix = Matrix3d::fromRowMajorArray(r);
//	Vector3d tipPos = Vector3d(t[0],t[1],t[2])*100.0;
//	Vector3d tip = rotationMatrix * Vector3d(1,0,0);
//	
//	cout << "Endpoint position is: " << tipPos.toString() << endl;
//	cout << "Endpoint direction vector: " << tip.toString() << endl;
//	cout << endl;
//}
