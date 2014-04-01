#ifndef HATARAKI_BASICMOTION_MOTION_PLANNING_OPSPACESTATE_HPP_
#define HATARAKI_BASICMOTION_MOTION_PLANNING_OPSPACESTATE_HPP_

#include "MathUtils.hpp"


class OpSpaceState {

public:
	vmath::Vector3d Position;
	vmath::Matrix3d Rotation;		
		
	OpSpaceState(vmath::Vector3d _Position, vmath::Matrix3d _Rotation)
	{
		this->Position = _Position;
		this->Rotation = _Rotation;
	}

};


#endif