#ifndef HATARAKI_BASICMOTION_POSE_DYNAMICS_HPP_
#define HATARAKI_BASICMOTION_POSE_DYNAMICS_HPP_

#define JointCount 6

#include "IKFast.hpp"

#include "cJSON.h"

#include <vector>

#include "MathUtils.hpp"
#include "ServoModel.hpp"
#include "ArmState.hpp"

class PoseDynamics {
		
	struct SegmentTransform
	{
		vmath::Vector3d Translation;
		vmath::Matrix3d Rotation;
		
		vmath::Vector3d PointMassPosition;
		double PointMassValue;
		double MomentOfInertia;
		
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

	std::vector<vmath::Vector3d> childPointMassPosition;
	std::vector<double> childPointMassValue;	
	std::vector<double> jointAngles;
	std::vector<SegmentTransform> segmentTransforms;
	
	bool updateNeeded;

	PoseDynamics();

	void computeChildPointMass(int targetJoint, vmath::Vector3d & pointMassPosition, double & pointMassValue);
	void computeSegmentTransform(int targetJoint, vmath::Vector3d & segmentPosition, vmath::Matrix3d & segmentRotationMatrix);

	vmath::Vector3d computeTorqueFromForces(int targetJoint);
		

public:
	void setArmModel(ArmModel * armModel);
	void setArmState(ArmState armState);
	
	void update();
		
	double getTorqueForAcceleration(int jointIndex, double accel);
	double computeJointTorque(int targetJoint);
};


#endif