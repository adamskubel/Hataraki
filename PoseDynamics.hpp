#ifndef HATARAKI_BASICMOTION_POSE_DYNAMICS_HPP_
#define HATARAKI_BASICMOTION_POSE_DYNAMICS_HPP_

#define JointCount 6
#define IKFAST_NO_MAIN
#define IKFAST_HAS_LIBRARY
#define IKFAST_NAMESPACE ikfast2
#include "ikfast.h"
#include "cJSON.h"
#include "MathUtils.hpp"
#include <vector>

#define VMATH_NAMESPACE vmath
#include "vmath.h"

#include "ServoModel.hpp"

class PoseDynamics {
		
	struct SegmentTransform
	{
		vmath::Vector3d Translation;
		vmath::Matrix3d Rotation;
		
		SegmentTransform(vmath::Vector3d _Translation, vmath::Matrix3d _Rotation) :
			Translation(_Translation), Rotation(_Rotation)
		{
		}
	public:
		vmath::Vector3d transformVector(vmath::Vector3d input)
		{
			vmath::Vector3d output = Rotation * input;
			return output + Translation;
		}
	};

public:
	static PoseDynamics& getInstance()
	{
		static PoseDynamics instance;
		return instance;
	}

private:
	ArmModel * armModel;

	PoseDynamics();
	void computeChildPointMass(int targetJoint, vmath::Vector3d & pointMassPosition, double & pointMassValue);
	void computeSegmentTransform(int targetJoint, vmath::Vector3d & segmentPosition, vmath::Matrix3d & segmentRotationMatrix);

	vmath::Vector3d computeTorqueFromForces(int targetJoint);
	vmath::Vector3d computeTorqueFromAngularAcceleration(int targetJoint);
		
	std::vector<double> cJointVelocities;
	std::vector<double> nJointVelocities;
	
	double cTime;
	double nTime;

	//std::vector<double> jointTorques;
	

public:
	std::vector<vmath::Vector3d> childPointMassPosition;
	std::vector<double> childPointMassValue;	
	
	std::vector<double> jointAngles;
	std::vector<SegmentTransform> segmentTransforms;

	void setArmModel(ArmModel * armModel);

	void setJointAngles(std::vector<double> angles);
	void setJointVelocities(std::vector<double> velocities);
	
	void update();

	double computeJointTorque(int targetJoint);

};


#endif