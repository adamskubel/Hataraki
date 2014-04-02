#ifndef HATARKAI_BASICMOTION_UI_IKCONTROL_HPP_
#define HATARKAI_BASICMOTION_UI_IKCONTROL_HPP_

#include <ncurses.h>

#include <vector>
#include <string>
#include <map>
#include <memory>
#include <mutex>

#include "MotionController.hpp"
#include "PredictiveJointController.hpp"
#include "UIElement.hpp"
#include "MotionPlan.hpp"

//class UIControlProvider : public DirectControlProvider
//{
//public:
//	UIControlProvider() :
//		currentGoal(IKGoal::stopGoal())
//	{
//		hasControl = false;
//	}
//		
//	void grantControl()
//	{
//		hasControl = true;
//	}
//	
//	void revokeControl()
//	{
//		hasControl = false;
//	}
//	
//	void setGoal(IKGoal goal);
//	
//	IKGoal nextGoal();
//	void motionComplete();
//	void motionOutOfRange();
//
//	IKGoal currentGoal;
//
//	bool hasControl, hasError;
//	std::string errorText;
//
//	std::mutex goalMutex;
//
//};

class IKControlUI {
	

private:	
	//UI properties
	WINDOW * uiWindow;	
	std::map<std::string,UIElement*> elements;
	UIElement * selectedElement;
	
	//UI members
	void init();
	bool handleElementSelection(int c);
	void handleInput();
	void updateAll();	
	UIElement * getNextElementFrom(int x, int y, int xDirection, int yDirection);
	void setSelected(UIElement * element);
	
	//Direct control
	WINDOW * directControlWindow;
	bool directModeActive;
//	UIControlProvider * controlProvider;

	bool doDirectControl(int c);	
	void drawDirectControlWindow();

	//Numeric input/output
	std::vector<NumberBox*> actualAngles, inputAngles;
	vmath::Vector3d getVectorFromElements(std::vector<std::string> elementNames);
	void updateArmStatus(bool copyToInput);
	
	std::vector<vmath::Vector3d> Favorites;
	
	//Motion planning
	MotionController * motionController;
	std::vector<std::shared_ptr<MotionPlan> > pendingMotionPlan;	
	
	void calculatePlan();
	void executePlan();
	
	void setTargetGoal(vmath::Vector3d position, vmath::Vector3d eulerAngles);
	
public:
	IKControlUI(MotionController * motionController);
	void start();
	
	volatile bool running;
	
	
	
};


#endif