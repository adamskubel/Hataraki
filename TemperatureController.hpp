#ifndef HATARAKI_TEMPERATURE_CONTROLLER_HPP_
#define HATARAKI_TEMPERATURE_CONTROLLER_HPP_

#include "DRV8830.hpp"
#include "ADS1015.hpp"
#include "IRealtimeUpdatable.hpp"
#include <iostream>

class TemperatureController : public IRealtimeUpdatable {
	
private:
	ADS1015 * adc;
	DRV8830 * power;
	
	int updateCount;
	double sensorVoltage;
	double sensorTemp;
	double sensorResistance;
	
	static double resistanceToTemperature(double resistance);
	
public:
	TemperatureController(ADS1015 * adc, DRV8830 * powerControl);
	
	void update();
	void shutdown();
	
	void printStatus();
	
	
	
};

#endif