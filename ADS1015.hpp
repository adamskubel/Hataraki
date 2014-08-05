#ifndef HATARAKI_DEVICEUTIL_ADS1015_HPP_
#define HATARAKI_DEVICEUTIL_ADS1015_HPP_

#include "I2CDevice.hpp"

struct ConfigData {
	
	int mux;
	int pga;
	bool singleShotMode;
	int dataRate;
	bool windowComparatorMode;
	bool activeHighAlert;
	bool latchAlertOutput;
	int comparatorQueue;
	
	bool startSingleShotRead;
	
	
public:
	ConfigData() {
		mux = 0;
		pga = 2;
		singleShotMode = true;
		dataRate = 4;
		windowComparatorMode = false;
		activeHighAlert = false;
		latchAlertOutput = false;
		comparatorQueue = 3;
		startSingleShotRead = false;
	}
	
	int buildConfigWord() {
		
		int word = 0;
		word += ((int)startSingleShotRead) << 15;
		word += mux << 12;
		word += pga << 9;
		word += ((int)singleShotMode) << 8;
		word += dataRate << 5;
		word += ((int)windowComparatorMode) << 4;
		word += ((int)activeHighAlert) << 3;
		word += ((int)latchAlertOutput) << 2;
		word += comparatorQueue;
		
		return word;
	}
	
};

class ADS1015 : public I2CDevice {
	
private:
	struct Registers {
		
		const static int Conversion = 0;
		const static int Config = 1;
		
	};
	
	
	
	
	ConfigData config;
	
public:
	ADS1015(I2CBus * bus, int address);
	
	double readVoltage();
	
	void setChannel(int channel);
	
	static double convResult(int result);
	
};


#endif