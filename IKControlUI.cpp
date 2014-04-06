#include "IKControlUI.hpp"

using namespace std;
using namespace vmath;

/*

 T: Tx Ty Tz    |   Ta --
 R: rX rY rZ	|	Ra --
 A: a0 ... a5	|	Aa --
 -------------------------
 #Div [1-10] Mode: [auto,target confirm,target angle confirm]
 ----
 Keys:	T->x,y,z or numbers
		R->x,y,z or numbers
		arrow up/down moves highlighted
		arrow left/right 
*/

IKControlUI::IKControlUI(MotionController * motionController)
{
	this->motionController = motionController;
	this->selectedElement = NULL;
	this->relativeMode = false;
	this->relativeScale = 0.25;
//	this->controlProvider = new UIControlProvider();
	init();
}

void IKControlUI::init()
{
	int x0 = 18;
	int columnWidth = 12;
	directModeActive = false;
	
	Favorites = {
		{10,0,6},{0,90,0},	//o
		{14,0,4},{0,0,0},	//p
		{10,-6,0},{0,0,-90}, //i
		{4,0,4},{0,-90,0},
		{4,-7,-1.5},{0,0,160}};
	
	Offsets = {
		{-1,0,0},{0,-90,0},	//w
		{1,0,0},{0,-90,0},	//s
		{0,1,0},{0,-90,0},	//a
		{0,-1,0},{0,-90,0},	//d
		{0,0,-1},{0,-90,0},	//q
		{0,0,1},{0,-90,0}};	//e
	
	elements.insert(make_pair("T",new UIElement(3,1,"T:")));
	elements["T"]->setText("[T]ranslation:");
	
	
	elements.insert(make_pair("Tx",new NumberBox(3,x0+columnWidth*0,"Tx")));
	elements.insert(make_pair("Ty",new NumberBox(3,x0+columnWidth*1,"Ty")));
	elements.insert(make_pair("Tz",new NumberBox(3,x0+columnWidth*2,"Tz")));
	
	elements.insert(make_pair("R",new UIElement(4,1,"R:")));
	elements["R"]->setText("[R]otation:");
	
	elements.insert(make_pair("Rx",new NumberBox(4,x0+columnWidth*0,"Rx")));
	elements.insert(make_pair("Ry",new NumberBox(4,x0+columnWidth*1,"Ry")));
	elements.insert(make_pair("Rz",new NumberBox(4,x0+columnWidth*2,"Rz")));
		
	elements.insert(make_pair("AngleLabel",new UIElement(5,1,"AngleLabel")));
	elements["AngleLabel"]->setText("[A]ngles:");

	elements["Rx"]->setWidth(12);
	elements["Ry"]->setWidth(12);
	elements["Rz"]->setWidth(12);

	elements["Tx"]->setWidth(12);
	elements["Ty"]->setWidth(12);
	elements["Tz"]->setWidth(12);
	
	int angleColumnWidth = 8;
	for (int i=0;i<6;i++)
	{
		stringstream name;
		name << "iA" << i;
		NumberBox * inputAngle = new NumberBox(5,x0+angleColumnWidth*i,name.str());
		inputAngle->setPrecision(1);
		inputAngle->setBorderVisible(false);
		inputAngle->setIncrement(0.1);
		inputAngle->setWidth(7);
		elements.insert(make_pair(name.str(),inputAngle));
		inputAngles.push_back(inputAngle);
	}
	
	vector<NumberBox*> armValues;
	x0 = 76;
	armValues.push_back(new NumberBox(3,x0+columnWidth*0,"aTx"));
	armValues.push_back(new NumberBox(3,x0+columnWidth*1,"aTy"));
	armValues.push_back(new NumberBox(3,x0+columnWidth*2,"aTz"));
		
	armValues.push_back(new NumberBox(4,x0+columnWidth*0,"aRx"));
	armValues.push_back(new NumberBox(4,x0+columnWidth*1,"aRy"));
	armValues.push_back(new NumberBox(4,x0+columnWidth*2,"aRz"));
	
	for (auto it = armValues.begin(); it != armValues.end(); it++)
	{
		NumberBox * n = *it;
		elements.insert(make_pair(n->Name,n));
		n->setActive(false);
		n->setBorderVisible(false);
	}
	
	
	
	for (int i=0;i<6;i++)
	{
		stringstream name;
		name << "aA" << i;
		NumberBox * actualAngle = new NumberBox(5,x0+angleColumnWidth*i,name.str());
		elements.insert(make_pair(name.str(),actualAngle));
		actualAngle->setPrecision(1);
		actualAngle->setActive(false);
		actualAngle->setBorderVisible(false);
		actualAngles.push_back(actualAngle);
	}
	
	
	elements.insert(make_pair("DivisionLabel",new UIElement(6,1,"DivisionLabel","Path Divisions:")));
	elements.insert(make_pair("DivisionNumBox",new NumberBox(6,18,"DivisionNumBox")));

	((NumberBox*)elements["DivisionNumBox"])->setIncrement(1);
	((NumberBox*)elements["DivisionNumBox"])->setPrecision(0);
	((NumberBox*)elements["DivisionNumBox"])->setValue(1);
	
	elements.insert(make_pair("PathStepLength_Label",new UIElement(6,24,"PathStepLength_Label","Step size (mm):")));
	elements.insert(make_pair("PathStepLength",new NumberBox(6,38,"PathStepLength")));
	
	((NumberBox*)elements["PathStepLength"])->setIncrement(0.1);
	((NumberBox*)elements["PathStepLength"])->setPrecision(1);
	((NumberBox*)elements["PathStepLength"])->setValue(2.5);
	
	int helpX = 4,helpY = 8, helpSpacing = 6;
	
	vector<pair<string,string> > helpKeys({
		{"Direct Mode","d"},
		{"Update angles","u"},
		{"Go to angles","g"},
		{"Calculate angles","c"},
		{"Emergency Stop","k"},
		{"Exit","x"}
	});
	
	for (int i =0; i < helpKeys.size();i++)
	{
		stringstream keyLabel;
		keyLabel << helpKeys.at(i).second << " --> ";
		stringstream name;
		name << "HelpKey_Action" << i;
		elements.insert(make_pair(name.str(),new UIElement(helpY+i,helpX,name.str(),keyLabel.str())));
		name.clear();
		name << "HelpKey_Key" << i;
		elements.insert(make_pair(name.str(),new UIElement(helpY+i,helpX+helpSpacing,name.str(),helpKeys.at(i).first)));
	}

	elements.insert(make_pair("StatusLog",new UIElement(15,1,"StatusLog","Hello!")));
	
}



void IKControlUI::start()
{	
	initscr();
	cbreak();
	noecho();
	curs_set(0);

	int windowHeight = LINES, windowWidth = COLS;
	uiWindow = newwin(windowHeight,windowWidth,0,0); //(LINES - windowHeight)/2,(COLS - windowWidth)/2);
	//int directWindowHeight = 20, directWindowWidth = 80;
	//directControlWindow = newwin(LINES - directWindowHeight)/2,(COLS - directWindowWidth)/2);
	keypad(uiWindow,TRUE);

	box(uiWindow,0,0);
	wrefresh(uiWindow);	
	mvwprintw(uiWindow,0,0,"IK Control UI");
	
	setSelected(elements["T"]);
	updateArmStatus(true);
	updateAll();
		
	running = true;
	while (running)
	{
		handleInput();
		updateAll();
	}
	
	endwin();
}

void IKControlUI::updateAll()
{
	for (auto it = elements.begin(); it != elements.end(); it++)
	{
		if (it->second->isUpdateNeeded())
			it->second->update(uiWindow);
	}
	wrefresh(uiWindow);
}


bool IKControlUI::handleElementSelection(int c)
{
	UIElement * selectNext = NULL;
	int startX = 0, startY = 0;
	if (selectedElement != NULL)
	{
		startX = selectedElement->getX();
		startY = selectedElement->getY();
	}
	switch (c)
	{
		case KEY_RIGHT:
		case '\t':
		case 10:
			selectNext = getNextElementFrom(startX,startY,1,0);
			break;
		case KEY_LEFT:
			selectNext = getNextElementFrom(startX,startY,-1,0);
			break;
		case KEY_UP:
			selectNext = getNextElementFrom(startX,startY,0,-1);
			break;
		case KEY_DOWN:
			selectNext = getNextElementFrom(startX,startY,0,1);
			break;
	}
	
	if (selectNext != NULL)
	{
		setSelected(selectNext);
		return true;
	}
	return false;
}

void IKControlUI::handleInput()
{
	int c = wgetch(uiWindow);
		
	bool handled = false;
	
	try
	{
		handled = handled || handleElementSelection(c);
		handled = handled || (selectedElement != NULL && selectedElement->handleInput(uiWindow,c));
		
		if (!handled)
		{
			switch (c)
			{
				case 'o':
					setTargetGoal(Favorites[0], Favorites[1],false);
					executePlan();
					break;
				case 'p':
					setTargetGoal(Favorites[2], Favorites[3],false);
					executePlan();
					break;
				case 'i':
					setTargetGoal(Favorites[8], Favorites[9],false);
					executePlan();
					break;
				case 'w':
					setTargetGoal(Offsets[0], Offsets[1], true);
					executePlan();
					break;
				case 's':
					setTargetGoal(Offsets[2], Offsets[3], true);
					executePlan();
					break;
				case 'a':
					setTargetGoal(Offsets[4], Offsets[5], true);
					executePlan();
					break;
				case 'd':
					setTargetGoal(Offsets[6], Offsets[7], true);
					executePlan();
					break;
				case 'q':
					setTargetGoal(Offsets[8], Offsets[9], true);
					executePlan();
					break;
				case 'e':
					setTargetGoal(Offsets[10], Offsets[11], true);
					executePlan();
					break;
				case 'U':
					updateArmStatus(true);
					break;
				case 'u':
					updateArmStatus(false);
					break;
				case 'x':
					running = false;
					break;
				case 'c':
					calculatePlan();
					break;
				case 'g':
					executePlan();
					break;
				case 'k':
					motionController->shutdown();
					running = false;
					break;
				case 'h':
					motionController->postTask([this](){motionController->zeroAllJoints();});
					break;
				case 'r':
					elements["StatusLog"]->setText("Direct mode enabled");
					directModeActive = true;
					break;
			}
		}
	}
	catch (std::runtime_error & e)
	{
		elements["StatusLog"]->setText(e.what());
	}
}


Vector3d IKControlUI::getVectorFromElements(vector<string> elementNames)
{
	NumberBox * x = (NumberBox*)elements[elementNames[0]];
	NumberBox * y = (NumberBox*)elements[elementNames[1]];
	NumberBox * z = (NumberBox*)elements[elementNames[2]];
	
	return Vector3d(x->getValue(),y->getValue(),z->getValue());
}

void IKControlUI::calculatePlan()
{
	int divisionCount = (int)((NumberBox*)elements["DivisionNumBox"])->getValue();	
	if (divisionCount < 1 || divisionCount > 100) throw std::runtime_error("Invalid value for division count");
	
	Vector3d targetPosition = getVectorFromElements({"Tx","Ty","Tz"}) / 100.0;
	Vector3d targetEulerAngles = getVectorFromElements({"Rx","Ry","Rz"});
	
	Matrix3d targetRotation = Matrix3d::createRotationAroundAxis(targetEulerAngles.x, targetEulerAngles.y,targetEulerAngles.z);


	motionController->getTrajectoryPlanner()->setPathDivisions(divisionCount);
	motionController->getTrajectoryPlanner()->setPathInterpolationMode(PathInterpolationMode::FixedStepCount);
	
	auto trajectory = motionController->getTrajectoryPlanner()->buildTrajectory(IKGoal(targetPosition, targetRotation, relativeMode));
	
	Vector3d finalPos = trajectory.back().Position*100.0;
	
	((NumberBox*)elements["Tx"])->setValue(finalPos.x);
	((NumberBox*)elements["Ty"])->setValue(finalPos.y);
	((NumberBox*)elements["Tz"])->setValue(finalPos.z);

	pendingMotionPlan = motionController->getMotionPlanner()->buildPlan(trajectory);
	
	for (int i=0;i<6;i++)
	{
		inputAngles[i]->setValue(AS5048::stepsToDegrees(pendingMotionPlan[i]->finalAngle));
	}
}

void IKControlUI::executePlan()
{
	if (pendingMotionPlan.empty()) 	calculatePlan();
	motionController->executeMotionPlan(pendingMotionPlan);
}

void IKControlUI::setSelected(UIElement * element)
{
	if (selectedElement != NULL)
	{
		selectedElement->setSelected(false);
	}
	
	selectedElement = element;
	selectedElement->setSelected(true);
}

void IKControlUI::setTargetGoal(Vector3d position, Vector3d eulerAngles, bool relative)
{
	if (relative)
	{
		relativeScale = ((NumberBox*)elements["PathStepLength"])->getValue()/10.0;
		position *= relativeScale;
	}
		
	((NumberBox*)elements["Tx"])->setValue(position.x);
	((NumberBox*)elements["Ty"])->setValue(position.y);
	((NumberBox*)elements["Tz"])->setValue(position.z);
	
	if (!relative)
	{
		((NumberBox*)elements["Rx"])->setValue(eulerAngles.x);
		((NumberBox*)elements["Ry"])->setValue(eulerAngles.y);
		((NumberBox*)elements["Rz"])->setValue(eulerAngles.z);
	}
	
	relativeMode = relative;

	calculatePlan();
}

void IKControlUI::updateArmStatus(bool copyToInput)
{
	Vector3d translation;
	Matrix3d rotationMatrix;
	
	vector<double> angles;
	
	motionController->getTransform(angles, translation, rotationMatrix);

	translation *= 100.0;
	
	double xR,yR,zR;
	MathUtil::extractEulerAngles(rotationMatrix,xR,yR,zR);
	
	
	if (relativeMode)
	{
		Vector3d targetPosition = getVectorFromElements({"Tx","Ty","Tz"});
		translation = translation - targetPosition;
	}
	
	((NumberBox*)elements["aTx"])->setValue(translation.x);
	((NumberBox*)elements["aTy"])->setValue(translation.y);
	((NumberBox*)elements["aTz"])->setValue(translation.z);
	
	((NumberBox*)elements["aRx"])->setValue(xR);
	((NumberBox*)elements["aRy"])->setValue(yR);
	((NumberBox*)elements["aRz"])->setValue(zR);
	
	if (copyToInput)
	{
		((NumberBox*)elements["Tx"])->setValue(translation.x);
		((NumberBox*)elements["Ty"])->setValue(translation.y);
		((NumberBox*)elements["Tz"])->setValue(translation.z);
		((NumberBox*)elements["Rx"])->setValue(xR);
		((NumberBox*)elements["Ry"])->setValue(yR);
		((NumberBox*)elements["Rz"])->setValue(zR);		
	}
	
	for (int i=0;i<6;i++)
	{
		if (motionController->getJointByIndex(i)->getJointStatus() == JointStatus::Active)
		{
			if (relativeMode)
				actualAngles[i]->setValue(angles[i] - inputAngles[i]->getValue());
			else
				actualAngles[i]->setValue(angles[i]);
			
			if (copyToInput)
				inputAngles[i]->setValue(angles[i]);
		}
		else
		{
			actualAngles[i]->setText("ERR");
		}

	}
	
}

UIElement * IKControlUI::getNextElementFrom(int x, int y, int xDirection, int yDirection)
{
	vector<UIElement*> inlineElements;
	
	if (xDirection != 0)
	{
		for (auto it = elements.begin(); it != elements.end(); it++)
		{
			if (it->second->isActive() && it->second->getY() == y && sgn(it->second->getX() - x) == sgn(xDirection))
				inlineElements.push_back(it->second);
		}
		
		if (inlineElements.empty())
		{
			return NULL;
		}
		else
		{
			std::sort(inlineElements.begin(),inlineElements.end(),[x](UIElement * e0, UIElement * e1) { return abs(e0->getX() - x) <  abs(e1->getX() - x);});
			return inlineElements.front();
		}
	}
	else
	{
		int tolerance = 4;
		for (auto it = elements.begin(); it != elements.end(); it++)
		{
			if (it->second->isActive() && abs(it->second->getX() - x) < tolerance && sgn(it->second->getY() - y) == sgn(yDirection))
				inlineElements.push_back(it->second);
		}
		
		if (inlineElements.empty())
		{
			return NULL;
		}
		else
		{
			std::sort(inlineElements.begin(),inlineElements.end(),[y](UIElement * e0, UIElement * e1) { return abs(e0->getY() - y) <  abs(e1->getY() - y);});
			return inlineElements.front();
		}
	}
}








