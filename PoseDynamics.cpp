#include "PoseDynamics.hpp"

using namespace vmath;
using namespace std;


PoseDynamics::PoseDynamics() 
{

}

void PoseDynamics::setArmModel(ArmModel * _armModel)
{
	this->armModel = _armModel;
}


void PoseDynamics::computeSegmentTransform(int targetJoint, Vector3d & segmentPosition, Matrix3d & segmentRotationMatrix)
{

	double fkAngles[JointCount];
	double tipDistance = 0;

	for (int i=0;i<JointCount;i++) {

		if (i > targetJoint)
			fkAngles[i] = 0;		
		else
			fkAngles[i] = jointAngles[i];

		if (i >= targetJoint)			
		{
			tipDistance += armModel->segments.at(i).boneLength;
		}
	}
		

	double r[9]; Vector3d translation;
	ikfast2::ComputeFk(fkAngles,translation,r);	
	Matrix3d rotationMatrix = Matrix3d::fromRowMajorArray(r);	
	
		
	Vector3d tipVector(tipDistance,0,0);

	tipVector = rotationMatrix * tipVector;
		
	segmentPosition = translation - tipVector;
	segmentRotationMatrix = rotationMatrix;
}

void PoseDynamics::computeChildPointMass(int targetJoint, Vector3d & pointMassPosition, double & pointMassValue)
{
	pointMassValue = 0;
	for (int i=targetJoint;i<JointCount;i++)
	{
		pointMassValue += armModel->segments.at(i).mass;
	}

	pointMassPosition = Vector3d(0,0,0);
	for (int i=targetJoint;i<JointCount;i++)
	{
		Matrix3d segmentRotation = segmentTransforms[i].Rotation;
		Vector3d segmentPosition = segmentTransforms[i].Translation;
		
		Vector3d segmentCoG = (segmentRotation * armModel->segments.at(i).centerOfMass) + segmentPosition;

		pointMassPosition += segmentCoG * (armModel->segments.at(i).mass/pointMassValue);
	}
}


//Compute moment of inertia and position of each segment
//JointControllers provide current angular velocities
//Thus determining the angular momentum of a point mass rotating about each joint
//There will be two sources of torque:
// - Change of angular momentum resulting from joint angles changing
// - Result of force applied on point mass due to gravity
//Summing the two torque vectors provides the net torque
//I THINK this net torque vector is then projected onto the axis of rotation to determine the relevant
//the torque about the joint
//Return current torque and predicted torque for a given input (or return angular momentum?)

//Step 1: Compute CoG of each segment <-- done
//Step 2: Determine angular momentum using velocities
//Step 3: Determine torque due to forces

Vector3d PoseDynamics::computeTorqueFromAngularAcceleration(int targetJoint)
{
	//torque = dL/dt
	//L = r x p
	//L = I*dP
	//p0 = inertia * angularVelocity0
	//p1 = inertia * angularVelocity1
	
	SegmentModel * segment = &(armModel->segments[targetJoint]);
	JointModel * joint = &(armModel->joints[targetJoint]);
		
	Matrix3d segmentRotation = segmentTransforms[targetJoint].Rotation;
	Vector3d segmentPosition = segmentTransforms[targetJoint].Translation;
	double momentOfIntertia =  0;//[targetJoint];

	double dT = nTime - cTime;
	
	Vector3d angularAcceleration = joint->axisOfRotation * (nJointVelocities.at(targetJoint) - cJointVelocities.at(targetJoint))/dT;
	
	Vector3d dL =  angularAcceleration * momentOfIntertia;
	
	return dL;
}

Vector3d PoseDynamics::computeTorqueFromForces(int targetJoint)
{
	//torque = r x F
	//F = gravity vector ([0,0,-9.8]*mass)
	//r = distance to CoG
	
	const Vector3d Gravity(0,0,-9.8);

	Vector3d position = segmentTransforms[targetJoint].Translation;

	Vector3d r = childPointMassPosition[targetJoint] - position;
	double mass = childPointMassValue[targetJoint];
	
	Vector3d F = Gravity * mass;
	
	Vector3d torque = r.crossProduct(F);
	return torque;
}

double PoseDynamics::computeJointTorque(int targetJoint)
{
	Vector3d tAccel = Vector3d(0,0,0);//computeTorqueFromAngularAcceleration(targetJoint);
	Vector3d tGravity = computeTorqueFromForces(targetJoint);

	Vector3d jointAxis = armModel->joints.at(targetJoint).axisOfRotation;
	
	jointAxis = segmentTransforms[targetJoint].transformVector(jointAxis);

	return jointAxis.dotProduct(tAccel+tGravity);
}

void PoseDynamics::update()
{
	segmentTransforms.clear();
	childPointMassPosition.clear();
	childPointMassValue.clear();

	for (int i=0;i<JointCount;i++)
	{		
		Matrix3d rotation;
		Vector3d translation;
		computeSegmentTransform(i,translation,rotation);
		segmentTransforms.push_back(SegmentTransform(translation,rotation));
	}

	for (int i=0;i<JointCount;i++)
	{	
		Vector3d pointMassPosition;
		double massValue;

		computeChildPointMass(i,pointMassPosition,massValue);

		childPointMassPosition.push_back(pointMassPosition);
		childPointMassValue.push_back(massValue);
	}
}

void PoseDynamics::setJointAngles(vector<double> _jointAngles)
{
	this->jointAngles = _jointAngles;
}
