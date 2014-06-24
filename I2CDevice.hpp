#ifndef HATARAKI_DEVICES_I2CDEVICE_HPP_
#define HATARAKI_DEVICES_I2CDEVICE_HPP_

#include "I2CBus.hpp"

class I2CDevice {
	
public:
	I2CDevice(I2CBus * bus, int deviceAddress);
};


#endif