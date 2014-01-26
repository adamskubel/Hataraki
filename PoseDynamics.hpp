#ifndef HATARAKI_BASICMOTION_POSE_DYNAMICS_HPP_
#define HATARAKI_BASICMOTION_POSE_DYNAMICS_HPP_

#define JointCount 6

#include "ikfast.h"
#include "cJSON.h"
#include "MathUtils.hpp"
#include <vector>

#include "vmath.h"

class PoseDynamics {

	struct SegmentDynamics 
	{
		Vector3d centerOfGravity;
		Vector3d jointAxis;
		double mass;
		double length;
	};

public:
	static PoseDynamics& getInstance()
	{
		static PoseDynamics instance;
		return instance;
	}

private:
	std::vector<SegmentDynamics*> armConfiguration;


	PoseDynamics();
	void computeSegmentCoG(int targetJoint, const double * jointAngles, Vector3d & segmentCoG);
	void computeSegmentTransform(int targetJoint, const double * jointAngles, Vector3d & segmentPosition, Matrix3d & segmentRotationMatrix);



public:
	void setConfiguration(cJSON * jsonConfig);

	double computeTorqueOnJoint(int targetJoint, double * jointAngles);

	



};


#endif