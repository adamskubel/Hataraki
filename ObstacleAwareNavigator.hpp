#ifndef HATARAKI_BASICMOTION_WHEELS_OBSTACLE_AWARE_NAVIGATOR_HPP_
#define HATARAKI_BASICMOTION_WHEELS_OBSTACLE_AWARE_NAVIGATOR_HPP_

#include "WheelMotionController.hpp"
#include "AntennaDeflectionSensor.hpp"

class ObstacleAwareNavigator : public WheelMotionController {

private:
	std::vector<AntennaDeflectionSensor*> antennaSensors;
	bool colliding;

public:
	ObstacleAwareNavigator(std::vector<AntennaDeflectionSensor*> antennaSensors, std::vector<PredictiveJointController*> wheelControllers, DriveModel driveModel, ServoDirectController * directController);

	void deflectionEvent();

	void update();

};

#endif