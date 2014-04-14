#include "NavigationController.hpp"

using namespace std;
using namespace vmath;

NavigationController::NavigationController(WheelMotionController * wheelController)
{
	this->wheelController = wheelController;
	this->currentPathStep = 0;
	this->pathExecutionsRemaining = 0;
	
	wheelController->addStateWatcher([this](WheelMotionController::State newState){
		
		cout << "State changed to " << (int)(newState) << endl;
		if (newState == WheelMotionController::State::Waiting)
			this->nextPathStep();
	});
}

vector<NavTransform> NavigationController::buildPath(cJSON * pathObject)
{
	vector<NavTransform> path;
	
	for (int i=0;i<cJSON_GetArraySize(pathObject);i++)
	{
		cJSON * pathStep = cJSON_GetArrayItem(pathObject,i);
		
		Vector2d position;
		auto obj =  cJSON_GetObjectItem(pathStep,"Translation");
		if (obj != NULL)
			position = Configuration::getVector2dFromJSON(obj);
		
		double rotation = 0;
		obj =  cJSON_GetObjectItem(pathStep,"Rotation");
		if (obj != NULL)
			rotation = obj->valuedouble;
		
		path.push_back(NavTransform(position,rotation));
	}
	
	return path;
}

void NavigationController::executePath(cJSON * pathObject, int repeatCount)
{	
	currentPath = buildPath(pathObject);
	currentPathStep = -1;
	
	pathExecutionsRemaining = repeatCount;
	
	cout << "Executing path of length " << currentPath.size() << endl;
	
	nextPathStep();
}

void NavigationController::nextPathStep()
{
	currentPathStep++;
	
	if (currentPathStep >= currentPath.size())
	{
		pathExecutionsRemaining--;
		currentPathStep = 0;
	}
		
	if (pathExecutionsRemaining <= 0)
	{
		return;
	}
	
	NavTransform t = currentPath.at(currentPathStep);
	
	if (t.TransformAction == NavTransform::Action::Rotation)
	{
//		cout << "Path step " << currentPathStep << " = rt " << t.Rotation << endl;
		wheelController->rotateBy(t.Rotation);
	}
	else
	{
//		cout << "Path step " << currentPathStep << " = mv " << t.Offset() << endl;
		wheelController->translateBy(t.Offset.x);
	}
}



























