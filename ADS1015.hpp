#ifndef HATARAKI_DEVICEUTIL_ADS1015_HPP_
#define HATARAKI_DEVICEUTIL_ADS1015_HPP_

#include "I2CDevice.hpp"


class ADS1015 : public I2CDevice {
	
private:
	struct Registers {
		
		int POINTER = 0;
		int CONVERSION = 1;
		int CONTROL = 2;
		
	};
	
public:
	ADS1015(I2CBus * bus, int address);
	
	double readVoltage();
	
	void setInput(int input);
	
};


#endif