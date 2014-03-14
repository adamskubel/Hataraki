#ifndef HATARAKI_BASICMOTION_FLIP_IDENTIFIER_HPP_
#define HATARAKI_BASICMOTION_FLIP_IDENTIFIER_HPP_

#include <vector>
#include <cmath>
#include <iostream>

#include <unistd.h>

#include "cJSON.h"

#include "PredictiveJointController.hpp"
#include "MotionController.hpp"

class FlipIdentifier {

private:
	PredictiveJointController * jointController;	
	std::vector<std::vector<double> > patterns;
	MotionController * motionController;

	std::vector<double> parsePattern(cJSON * rawPattern);

	int jointIndex;

public:
	FlipIdentifier(int jointIndex, MotionController * motionController);

	void loadPatterns(cJSON * patternArray);

	void execute();


};


#endif