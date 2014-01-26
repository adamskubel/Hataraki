#include "PoseDynamics.hpp"

using namespace vmath;


PoseDynamics::PoseDynamics()
{

}

void PoseDynamics::setConfiguration(cJSON * jsonConfig)
{

}

void PoseDynamics::computeSegmentTransform(int targetJoint, const double * jointAngles, Vector3d & segmentPosition, Matrix3d & segmentRotationMatrix)
{

	double fkAngles[JointCount];
	double tipDistance = 0;

	for (int i=0;i<JointCount;i++) {

		if (i > targetJoint) {
			fkAngles[i] = 0;
			tipDistance += armConfiguration.at(i)->length;
		}
		else
			fkAngles[i] = jointAngles[i];
	}
		

	double r[9]; Vector3d translation;
	ikfast::ComputeFk(fkAngles,translation,r);	
	Matrix3d rotationMatrix = Matrix3d::fromRowMajorArray(r);	
	
		
	Vector3d tipVector(tipDistance,0,0);

	tipVector = rotationMatrix * tipVector;
		
	segmentPosition = translation - tipVector;
	segmentRotationMatrix = rotationMatrix;
}

void PoseDynamics::computeSegmentCoG(int targetJoint, const double * jointAngles, Vector3d & segmentCoG)
{
	Matrix3d segmentRotation;
	Vector3d segmentPosition;

	computeSegmentTransform(targetJoint,jointAngles,segmentPosition,segmentRotation);
	
	Vector3d worldCoG = segmentRotation * armConfiguration.at(targetJoint)->centerOfGravity;
	worldCoG += segmentPosition;

	segmentCoG = worldCoG;
}

Vector3d projectOntoPlane(Vector4d plane,  Vector3d point) {
		
	Vector3d planeNormal = Vector3d(plane.x,plane.y,plane.z);

	double dist = point.dotProduct(planeNormal) + plane.w;
	Vector3d proj = point - (planeNormal * dist);
	return proj;
}

Vector4d constructPlane(Vector3d normal, Vector3d point)
{
	return Vector4d(normal.x,normal.y,normal.z,-(normal.x * point.x + normal.y * point.y + point.z * normal.z));
}

double PoseDynamics::computeTorqueOnJoint(int targetJoint, double * jointAngles)
{
	double totalMass = 0;
	for (auto it = armConfiguration.begin(); it != armConfiguration.end(); it++)
	{
		totalMass += (*it)->mass;
	}
	
	Vector3d targetJointPosition;
	Matrix3d targetJointRotation;
	computeSegmentTransform(targetJoint,jointAngles,targetJointPosition,targetJointRotation);

	Matrix3d inverseRotation = targetJointRotation.inverse();

	Vector3d centerOfGravity;

	for (int i=targetJoint;i<JointCount;i++) {
		
		Vector3d segmentCoG;
		computeSegmentCoG(i,jointAngles,segmentCoG);
		centerOfGravity += segmentCoG * armConfiguration.at(i)->mass/totalMass;
	}

	Vector3d transformedJointAxis = targetJointRotation * armConfiguration.at(targetJoint)->jointAxis;
	Vector4d jointPlane = constructPlane(targetJointPosition,transformedJointAxis );

	Vector3d projectedCoG = projectOntoPlane(jointPlane,centerOfGravity); //Project CoG onto joint rotation plane

	Vector3d gravityVector(0,0,-9.8);

	double gravityScale = transformedJointAxis.crossProduct(gravityVector).length();

	

}