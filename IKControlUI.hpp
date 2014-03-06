#ifndef HATARKAI_BASICMOTION_UI_IKCONTROL_HPP_
#define HATARKAI_BASICMOTION_UI_IKCONTROL_HPP_

#include <ncurses.h>

#include <vector>
#include <string>
#include <map>
#include <memory>

#include "MotionController.hpp"
#include "PredictiveJointController.hpp"
#include "UIElement.hpp"
#include "MotionPlan.hpp"

class IKControlUI {
	

private:
	const int RotationEditing = 0;
	const int TranslationEditing = 1;
	const int AngleEditing = 2;
	WINDOW * uiWindow;
	
	MotionController * motionController;
	std::map<std::string,UIElement*> elements;
	
	std::vector<NumberBox*> actualAngles, inputAngles;
	
	int editingState;
	
	void init();
	bool handleElementSelection(int c);
	void handleInput();
	void updateAll();
	
	void updateArmStatus(bool copyToInput);
	
	void setSelected(UIElement * element);
	
	UIElement * selectedElement;
	
	UIElement * getNextElementFrom(int x, int y, int xDirection, int yDirection);
	
	void calculatePlan();
	void executePlan();
	
	vmath::Vector3d getVectorFromElements(std::vector<std::string> elementNames);
	
	std::vector<std::shared_ptr<MotionPlan> > pendingMotionPlan;
	
public:
	IKControlUI(MotionController * motionController);
	void start();
	
	volatile bool running;
	
	
	
};


#endif