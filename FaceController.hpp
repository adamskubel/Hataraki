#ifndef HATARAKI_UBI_FACECONTROLLER_HPP_
#define HATARAKI_UBI_FACECONTROLLER_HPP_


#include "IRealtimeUpdatable.hpp"
#include "PCA9552.hpp"
#include "TimeUtil.hpp"
#include <vector>

class FaceController : public IRealtimeUpdatable
{
public:
	FaceController(PCA9552 * ledController);

	void startTestAnimation();

	void update();
	void shutdown();

private:
	PCA9552 * ledController;
	timespec animStartTime,lastUpdateTime;
	bool runAnim;
};

#endif
