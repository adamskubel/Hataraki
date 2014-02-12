#ifndef HATARAKI_BASICMOTION_STEP_CONTROLLER_HPP_
#define HATARAKI_BASICMOTION_STEP_CONTROLLER_HPP_


class StepController {
	
private:
	//Stepping states
	std::vector<double> stepVoltages;
	SteppingState steppingState;
	int stepVoltageIndex;
	struct timespec readDelayStart;
	double stepStartPosition;
	double stepInitialTargetDistance;
	int stepExpectedDirection;
	double stepVoltageIntegral;
	
public:

	
	void executeStep(double voltage, int energizeLength, int coastStepCount);
	void executeStep(std::vector<double> & voltagePattern);

	void doStepControl();

	void runController(	
	
}

#endif