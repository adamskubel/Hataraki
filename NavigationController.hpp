#ifndef HATARAKI_BASICMOTION_WHEEL_NAVIGATION_CONTROLLER_HPP_
#define HATARAKI_BASICMOTION_WHEEL_NAVIGATION_CONTROLLER_HPP_

#include <vector>
#include <iostream>
#include <string>

#include "Configuration.hpp"
#include "WheelMotionController.hpp"
#include "MathUtils.hpp"


struct NavTransform {

	enum class Action {
		Rotation,
		Translation
	};

	vmath::Vector2d Offset;
	double Rotation;
	Action TransformAction;

	NavTransform(vmath::Vector2d _Offset, double _Rotation)
	{
		this->Offset = _Offset;
		this->Rotation = _Rotation;

		if (Rotation == 0)
			TransformAction = Action::Translation;
		else
			TransformAction = Action::Rotation;
	}

};


class NavigationController {

private:
	WheelMotionController * wheelController;
	std::vector<NavTransform> currentPath;
	std::vector<NavTransform> buildPath(cJSON * pathObject);
	int currentPathStep, pathExecutionsRemaining;
	
	void nextPathStep();
	

public:
	NavigationController(WheelMotionController * wheelController);

	void executePath(cJSON * pathObject, int repeatCount);

};

#endif