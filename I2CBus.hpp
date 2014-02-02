#ifndef HATARAKI_BASICMOTION_I2CBUS
#define HATARAKI_BASICMOTION_I2CBUS

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <inttypes.h>
#include <stdio.h>

#include <stdexcept>
#include <sstream>

#include "SimpleMovingAverage.hpp"

class I2CBus {

private:
	int file,currentAddr;

	SimpleMovingAverage * sma;

public:
//	static I2CBus * getBus(string busname);
	
	I2CBus(const char * busname);

	void selectAddress(int addr);

	void writeToBus(unsigned char * buf, int length);
	void readFromBus(unsigned char * buffer, int length);
	
	void setRegister(unsigned char regAddr, unsigned char regValue);
	int getRegisterValue(unsigned char regAddr);		

	double getAverageDataRate(); //bits per second

};

#endif