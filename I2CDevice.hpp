#ifndef HATARAKI_DEVICES_I2CDEVICE_HPP_
#define HATARAKI_DEVICES_I2CDEVICE_HPP_

#include "I2CBus.hpp"

class I2CDevice {
	
public:
	I2CDevice(I2CBus * bus, int deviceAddress)
	{
		this->bus = bus;
		this->deviceAddress = deviceAddress;
	}
	
protected:
	void writeByte(int registerAddress,unsigned char data)
	{
		bus->selectAddress(deviceAddress);
		bus->writeByte(registerAddress, data);
	}
	
	int readByte(int registerAddress)
	{
		bus->selectAddress(deviceAddress);
		return bus->readByte(registerAddress);
	}
	
private:
	I2CBus * bus;
	int deviceAddress;
	
	
};


#endif