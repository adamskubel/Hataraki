#include "DriveModel.hpp"


double DriveModel::stepsToMeters(double steps)
{
	return metersPerStep*steps;
}

double DriveModel::metersToSteps(double meters)
{
	return meters/metersPerStep;
}