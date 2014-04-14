#ifndef HATARAKI_BASICMOTION_WHEEL_MOTION_CONTROLLER_HPP_
#define HATARAKI_BASICMOTION_WHEEL_MOTION_CONTROLLER_HPP_

#include <vector>
#include <functional>

#include "PredictiveJointController.hpp"
#include "DriveModel.hpp"
#include "IRealtimeUpdatable.hpp"
#include "ServoDirectController.hpp"

class WheelMotionController : public IRealtimeUpdatable {

public:
	enum class State
	{
		Active,
		Waiting
	};
	
private:
	DriveModel driveModel;
	std::vector<PredictiveJointController*> wheels;
	ServoDirectController * directController;
	std::vector<std::function<void(State)> > watchers;
	
	void setState(State newState);
	
protected:	
	State state;
	

public:
	WheelMotionController(std::vector<PredictiveJointController*> wheelControllers, DriveModel driveModel, ServoDirectController * directController);

	void translateBy(double distance);
	void rotateBy(double angleDegrees);
	void stopMotion();
	
	virtual void update() override;
	void shutdown() override;
	
	void addStateWatcher(std::function<void(State)> watcher);
	
};


#endif