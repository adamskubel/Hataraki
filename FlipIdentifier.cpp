#include "FlipIdentifier.hpp"

#include "AsyncLogger.hpp"

using namespace std;

FlipIdentifier::FlipIdentifier(int _jointIndex, MotionController * _motionController)
{
	this->motionController = _motionController;
	this->jointIndex = _jointIndex;

	this->jointController = motionController->getJointByIndex(jointIndex);
}

void FlipIdentifier::loadPatterns(cJSON * patternArray)
{
	patterns.clear();
	for (int i=0; i < cJSON_GetArraySize(patternArray); i++)
	{
		cJSON * pattern = cJSON_GetArrayItem(patternArray,i);
		patterns.push_back(parsePattern(pattern));
	}
	cout << "Loaded " << patterns.size() << " patterns." << endl;
}

std::vector<double> FlipIdentifier::parsePattern(cJSON * rawPattern)
{
	std::vector<double> pattern;
	
	for (int i=0; i < cJSON_GetArraySize(rawPattern); i++)
	{
		cJSON * interval = cJSON_GetArrayItem(rawPattern,i);
		double voltage = cJSON_GetArrayItem(interval,0)->valuedouble;
		double duration = cJSON_GetArrayItem(interval,1)->valuedouble/1000.0;

		int repeats = (int)std::round(duration / Configuration::SamplePeriod);

		for (int j=0;j<repeats;j++) pattern.push_back(voltage);
	}

	return pattern;
}


void FlipIdentifier::execute()
{
	/*
	stringstream ss;
	ss << "Pattern,Score,Extent,FinalAngle" << endl;
	AsyncLogger::getInstance().postLogTask("PatternResults.csv",ss.str());


	vector<double> alignPattern;
	for (int i=0;i<100;i++)
		alignPattern.push_back(-0.88);
	


	std::string input;
	for (int r = 0; r < 2; r++)
	{
		for (int i=0;i<patterns.size();i++)
		{
			cout << "Executing pattern[" << i << "]" << endl;

			motionController->postTask([this](){
				const double TestAngle = AS5048::degreesToSteps(10);
				const double TestAngleSpeed = AS5048::degreesToSteps(20);	
				motionController->setJointPosition(jointIndex,TestAngle);
			});		

			cout << "Waiting for joint to reach target..." << endl;

			usleep(1000*500); 
			while (!jointController->jointReadyForCommand()) {};
			usleep(1000*500); 

			jointController->requestPattern(alignPattern);
			cout << "Waiting for joint to align..." << endl;

			usleep(1000*500); 		
			while (!jointController->jointReadyForCommand()) {}; 
			usleep(1000*500); 		
			double startAngle = std::round(AS5048::stepsToDegrees(jointController->getCurrentAngle())/0.01)*0.01;
		
			jointController->requestPattern(patterns[i]);
			cout << "Running pattern..." << endl;

			usleep(1000*500); 
			while (!jointController->jointReadyForCommand()) {};
		
			double finalAngle = std::round(AS5048::stepsToDegrees(jointController->getCurrentAngle())/0.01)*0.01;

			cout << "Push and hold joint in alignment direction. Enter [y] when ready:"; cin >> input;
			usleep(1000*500); 		

			double extent1 = std::round(AS5048::stepsToDegrees(jointController->getCurrentAngle())/0.01)*0.01;
		
			double error;
			if (finalAngle > startAngle && extent1 > startAngle)
				error = finalAngle - startAngle;
			else
				error = extent1 - startAngle;			

			cout << "Start=" << startAngle << ", EndOffset=" << finalAngle-startAngle << ", Extents=" << (extent1-startAngle) << ". Error = " << error << endl;	
		
			stringstream ss;
			ss << i << "," << error << "," << extent1-startAngle << "," << finalAngle - startAngle << endl;
			AsyncLogger::getInstance().postLogTask("PatternResults.csv",ss.str());
		}
	}
	 */
}