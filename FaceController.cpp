#include "FaceController.hpp"
#include <cmath>

using namespace std;

FaceController::FaceController(PCA9552 * ledController)
{
	this->ledController = ledController;
	runAnim = false;
}

void FaceController::update()
{
	if (!runAnim) return;

	if (TimeUtil::timeSince(lastUpdateTime) < (1.0/10.0))
		return;

	vector<float> frame;

	float animFocus = fmodf(TimeUtil::timeSince(animStartTime),10.0f)/10.0f;
//
	for (int i=0;i<16;i++)
	{
		frame.push_back(0);
//		frame.push_back(max<float>(0, 2.0f - abs(animFocus - (float)i)));
	}
	float onValue = animFocus;
	frame[2] = onValue;
	frame[6] = onValue;
	frame[9] = onValue;
	frame[12] = onValue;
	
	ledController->setGridBrightness(frame);

	TimeUtil::setNow(lastUpdateTime);
	//runAnim = false;
}

void FaceController::startTestAnimation()
{
	animStartTime = TimeUtil::getNow();
	lastUpdateTime = TimeUtil::getNow();
	runAnim = true;
}


void FaceController::shutdown()
{
	
}