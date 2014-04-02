#ifndef HATARAKI_BASICMOTION_MOTION_PLANNING_OPSPACESTATE_HPP_
#define HATARAKI_BASICMOTION_MOTION_PLANNING_OPSPACESTATE_HPP_

#include <string>
#include <sstream>

#include "MathUtils.hpp"


class OpSpaceState {

public:
	vmath::Vector3d Position;
	vmath::Matrix3d Rotation;		

	OpSpaceState()
	{

	}
		
	OpSpaceState(vmath::Vector3d _Position, vmath::Matrix3d _Rotation)
	{
		this->Position = _Position;
		this->Rotation = _Rotation;
	}
	
	std::string toString()
	{
		return "T=" + Position.toString(); // + ";R=" + Rotation.toString();
	}

};


#endif