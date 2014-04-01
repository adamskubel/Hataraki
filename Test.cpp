#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_1
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_2
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_4
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_8

//#define IKFAST_CLIBRARY
#define IKFAST_NO_MAIN
#define IKFAST_HAS_LIBRARY
#define IKFAST_NAMESPACE ikfast2


#include <execinfo.h>

#include <vector>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <cmath>

#include "cJSON.h"

#include "I2CBus.hpp"
#include "PredictiveJointController.hpp"
#include "Configuration.hpp"
#include "MotionController.hpp"
#include "MathUtils.hpp"
#include "MotionPlanner.hpp"
#include "TimeUtil.hpp"

#define VMATH_NAMESPACE vmath
#include "vmath.h"

#include "PoseDynamics.hpp"
#include "MotionPlan.hpp"
#include "QuadraticRegression.hpp"

#include "MathUtilTests.hpp"
#include "IKControlUI.hpp"
#include "TMVoltageTest.hpp"


using namespace std;
using namespace ikfast2;
using namespace vmath;

MotionController * motionController;
vector<PredictiveJointController*> controllers;


double fmt(double d)
{
	return std::round(d/0.01)*0.01;
}


class FakeJoint : public PredictiveJointController
{
public:
	double Angle;
	
	FakeJoint(JointModel *jointModel) :
		PredictiveJointController(jointModel,map<string,I2CBus*>())
	{
		Angle = 0;
	}
	
	FakeJoint(JointModel *jointModel, double Angle) :
		PredictiveJointController(jointModel,map<string,I2CBus*>())
	{
		this->Angle = Angle;
	}
	
	virtual double getCurrentAngle()
	{
		return Angle;
	}
};

void testMotionPlanning();
void testQuadRegression();
void testQuadRegressionWithData();
void testDynamicTorque();
void testUI();
void testServoModel(ServoModel * sm);
void testKinematicPerformance();

int main(int argc, char *argv[])
{
	std::string configFileName = "config.json";

	if (argc >= 2)
	{
		configFileName = std::string(argv[1]);
	}

	Configuration::getInstance().loadConfig(configFileName);

	cJSON * globalConfig = cJSON_GetObjectItem(Configuration::getInstance().getRoot(),"GlobalSettings");	
	DRV8830::MaxVoltageStep = DRV8830::voltageToSteps(cJSON_GetObjectItem(globalConfig,"MaxVoltage")->valuedouble);		
//	cJSON * jointArray = cJSON_GetObjectItem(Configuration::getInstance().getRoot(),"JointDefinitions");
	
	ArmModel * armModel = new ArmModel(cJSON_GetObjectItem(Configuration::getInstance().getRoot(),"ArmModel"));
	PoseDynamics::getInstance().setArmModel(armModel);
	
	map<string,I2CBus*> busMap;

	for (int i=0;i<armModel->joints.size();i++)
	{
		PredictiveJointController * pjc = new FakeJoint(&(armModel->joints.at(i)),0);
		controllers.push_back(pjc);
	}
	motionController = new MotionController(controllers,5);
	

	
//	vector<double> lastAngles({1,1,1,1,1,1});
//	vector<double> initialAnglesConv(6);
//	std::transform(lastAngles.begin(),lastAngles.end(),initialAnglesConv.begin(),AS5048::radiansToSteps);
//	
//	for (int i=0;i<6;i++) cout << initialAnglesConv[i] << endl;
	
//	testUI();
	
//	cout << std::ios_base::dec << endl;
//	testQuadRegressionWithData();
//	testDynamicTorque();

//	cout << endl << endl << " --- MotionPlanning --- " << endl;
	testMotionPlanning();
	
//	HatarakiTest::testEulerAngleExtraction();
//	HatarakiTest::testIKRotation();
//	HatarakiTest::testTMVoltageConverter();
	
//	HatarakiTest::testAngleExtractionIK();
//	testKinematicPerformance();
	
//	testServoModel(&(controllers[0]->getJointModel()->servoModel));
}

void testLowPassFilter()
{
	
	LowpassFilter f(0.001);
	vector<double> testNum = {1,2,3,4,5};
	for (int i=0;i<testNum.size();i++)
	{
		cout << f.next(testNum[i]) << " ";
		usleep(1000);
		
	}
	cout << endl;
}

void testKinematicPerformance()
{
	timespec start;
	TimeUtil::setNow(start);
	for (int i=0;i<1000;i++)
	{
		PlanSolution sol;
		double minTime = KinematicSolver::threePart_minimumTime(10,100, 10, 0, 1000);
		KinematicSolver::threePart_calculate(10, 100, 10, 1000, minTime, sol);
	}
	cout << "Kinematic time = " << TimeUtil::timeSince(start) << " ms " << endl;
}

void testUI()
{
	IKControlUI ui(motionController);
	ui.start();
}

void testDynamicTorque()
{
	PoseDynamics::getInstance().setJointAngles({0,0,0,0,0,0});
//	PoseDynamics::getInstance().setJointAngles({100,-640,0,3400,6300,0});
	PoseDynamics::getInstance().update();
	
	cout << "-- Dynamic Torques -- " << endl;
	for (int i =0;i<6;i++)
	{
		double torque = PoseDynamics::getInstance().getTorqueForAcceleration(i, AS5048::stepsToRadians(5000));
		cout << torque << "  ";
	}
	cout << endl;
}

void testQuadRegression()
{
	QuadraticRegression * qr = new QuadraticRegression(15);
	
	for (int i=0;i<15;i++)
	{
		qr->addPoint(i, 5);
	}

	timespec time;
	TimeUtil::setNow(time);
	double a=0,b=9,c=0;
	a = qr->aTerm(); b = qr->bTerm(); c = qr->cTerm();
	double elapsed = TimeUtil::timeSince(time);
	
	cout << "A=" << a << " B=" << b << " C=" << c << " R= " << qr->rSquare() << endl;
	cout << "Took " << elapsed*1000.0 << " ms" << endl;
	
	delete qr;
	
}

void testQuadRegressionWithData()
{
	std::ifstream inFile;
	inFile.open("SampleAngleData.csv", std::ifstream::in);
	if (inFile.fail()) throw std::runtime_error("Failed to open in file.");
	
	
	std::ofstream outFile;
	outFile.open("/users/adamskubel/Desktop/test_out.csv",std::ofstream::out);
	
	if (outFile.fail()) throw std::runtime_error("Failed to open out file.");
	
	int length = 10;
	QuadraticRegression * qr = new QuadraticRegression(length);
	
	list<pair<double,double> > data;
	
	pair<double,double> last;
	outFile << "Time,Position,Accel,R2,LinSpeed,DerivSpeed,Delta" << endl;
	std::string v0,v1;
	while (getline(inFile,v0,',')) {
		getline(inFile,v1,'\n');
		double num1,num2;
		stringstream input(v0);
		input >> num1;
		stringstream input2(v1);
		input2 >> num2;
		
		
		if (input.fail() || input2.fail())
		{
			throw std::runtime_error("Error parsing input file.");
		}
		
		last = data.back();
		data.push_back(make_pair(num1,num2));
		
		if (data.size() > length) data.pop_front();
		
		qr->addPoint(num1, num2);
		
		if (qr->getSize() >= length)
		{
			double delta = (data.back().second - last.second)/(data.back().first - last.first);
			double slope, intercept,r2;
			MathUtil::linearRegression(data, slope, intercept, r2);
			outFile << num1 << "," << num2 << "," << qr->ddy(num1) << "," << qr->rSquare()
			<< "," << slope << "," << qr->dy(num1) << "," << delta << endl;
		}
	}
	
	outFile.close();
}

void testServoModel(ServoModel * sm)
{
	double torque = sm->getTorqueForVoltageSpeed(1.8,1000);
	cout << "V=1.8, S=1000, T=" << torque << endl;
	cout << "V=1.8, T=" << torque << ", S=" << sm->getSpeedForTorqueVoltage(torque,1.8) << endl;
	cout << "T=" << torque << ", S=1000, V=" << sm->getVoltageForTorqueSpeed(torque,1000) << endl;
	
	cout << endl;
	torque = sm->getTorqueForVoltageSpeed(-1.8,-1000);
	cout << "V=-1.8, S=-1000, T=" << torque << endl;
	cout << "V=-1.8, T=" << torque << ", S=" << sm->getSpeedForTorqueVoltage(torque,-1.8) << endl;
	cout << "T=" << torque << ", S=-1000, V=" << sm->getVoltageForTorqueSpeed(torque,-1000) << endl;
	
	cout << endl;
	cout << "V=-0.19, S=0, T=" << sm->getTorqueForVoltageSpeed(-0.19,0) << endl;
	
	cout << endl;
	cout << "V=0.56, T=0, S=" << sm->getSpeedForTorqueVoltage(0,0.56) << endl;
}

void setAngles(vector<double> angles)
{
	for (int i=0;i<6;i++)
		dynamic_cast<FakeJoint*>(controllers[i])->Angle = angles[i];
}

void setAnglesDegrees(vector<double> anglesDegrees)
{
	for (int i=0;i<6;i++)
		dynamic_cast<FakeJoint*>(controllers[i])->Angle = AS5048::degreesToSteps(anglesDegrees[i]);
}

void validatePlan(vector<shared_ptr<MotionPlan>> & motionPlan)
{
	cout << "Plan duration = " << motionPlan.front()->getPlanDuration() << endl;
	
	for (int c = 0; c < motionPlan.size(); c++)
	{
		double f0 = motionPlan[c]->finalAngle;
		double f1 =	motionPlan[c]->x(10);
		
		if (std::abs(f0 - f1) > 1.0)
		{
			cout << "Final = " << f0 << ", Final(t) = " <<  f1 << endl;
		}
	}
}

void testMotionPlanning()
{
	MotionPlanner * mp = new MotionPlanner(controllers);
	
	const vector<double> angles1 = {0.1,-6.4,0,34,63,0}; //11 0 -6
	const vector<double> angles2 = {0,-17,0,53,34,0}; //12 0 -5.5, 0 -70 0
	const vector<double> angles3  = {0,-22,0,60,41,0}; //11 0 -5.5, 0 -70 0
	const vector<double> angles4  = {0.2,-20,1.1,64.1,45.7,-0.5}; //10 0 -6
	
	
	std::ofstream outFile("/users/adamskubel/desktop/test_motionplanning.csv");
	int numChannels = 6;
	outFile << "Time,TipX,TipY,TipZ,LinX,LinY,LinZ,";
	
	std::string names[] = {"Roll0","Pitch0","Roll1","Pitch1","Pitch2","Roll2"};
	
	for (int c=0;c<numChannels;c++)
	{
		std::string name = names[c];
		outFile << name << ".x," << name << ".dx," << name << ".k," << name << ".ddx" << ",";
	}
	outFile << "KeyTipX,KeyTipY,KeyTipZ";
	outFile << endl;
	
	setAnglesDegrees(angles4); //Set with IK?
	
	double r[9];
	Vector3d startPosition;
	vector<double> a(6);
	transform(angles1.begin(), angles1.end(), a.begin(), MathUtil::degreesToRadians);
	ComputeFk(a.data(),startPosition,r);
	
	Vector3d endPosition(11,0,-6);
	endPosition /= 100;
	
	mp->setPathInterpolationMode(PathInterpolationMode::SingleStep);
	mp->setPathDivisions(20);
	auto motionPlan = mp->buildPlan(IKGoal(endPosition, Matrix3d::createRotationAroundAxis(0, -90, 0),false));
		
	validatePlan(motionPlan);
	
	double duration = motionPlan.front()->getPlanDuration();
	for (double t = 0.0; t < duration; t += 0.001)
	{
		Vector3d actualTip,linearTip, keyTip;
		vector<double> jointAngles, kfJointAngles;
		
		for (int c=0;c<numChannels;c++) jointAngles.push_back(AS5048::stepsToRadians(motionPlan[c]->x(t)));
				
		linearTip = startPosition + (endPosition-startPosition)*(t/duration);
		ComputeFk(jointAngles.data(), actualTip, r);
		
		linearTip *= 100;
		actualTip *= 100;
		
		outFile << t << ",";
		
		outFile << actualTip.x << "," << actualTip.y << "," << actualTip.z
		<< "," << linearTip.x << "," << linearTip.y << "," << linearTip.z << ",";
		for (int c=0;c<numChannels;c++)
		{
			auto kF= motionPlan[c]->keyframes.lower_bound(t);
			if (kF == motionPlan[c]->keyframes.end()) kF--;
			
			
			kfJointAngles.push_back(AS5048::stepsToRadians(kF->second));
						
			outFile
			<< fmt(motionPlan[c]->x(t)) << ","
			<< fmt(motionPlan[c]->dx(t)) << ","
			<< fmt(kF->second) << ","
			<< fmt((motionPlan[c]->ddx(t)*0.1)) << ",";
		}
		
		ComputeFk(kfJointAngles.data(), keyTip, r);
		keyTip *= 100.0;
		outFile << keyTip.x << "," << keyTip.y << "," << keyTip.z;
		
		outFile << endl;
	}
	outFile.close();	
}

















