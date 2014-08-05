#include "TemperatureController.hpp"
#include <cmath>

using namespace std;

TemperatureController::TemperatureController(ADS1015 * adc, DRV8830 * powerController)
{
	this->adc = adc;
	this->power = powerController;
	updateCount = 0;
}

void TemperatureController::update()
{
	adc->setChannel(0);
	sensorVoltage = adc->readVoltage();
	
	const double voltageInput = 3.3;
	const double staticResistor = 4300;
	
	double a = sensorVoltage/voltageInput;
	
	sensorResistance = staticResistor * (a/(1-a));
	sensorTemp = resistanceToTemperature(sensorResistance);
	updateCount++;
}

double TemperatureController::resistanceToTemperature(double resistance)
{
	const double t0 = 298.15;
	const double beta = 3977;
	const double r0 = 10000;
	const double e = 2.71828;
	const double r_inf = r0 * pow(e,-beta/t0);
	
	return beta/log(resistance/r_inf) - 273.15;
}

void TemperatureController::printStatus()
{
	cout << "Voltage = " << sensorVoltage << ", Resistance = " << sensorResistance << ", Temp = " << sensorTemp << endl;
	cout << "Count = " << updateCount << endl;
}

void TemperatureController::shutdown()
{
	
}