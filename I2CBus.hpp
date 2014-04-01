#ifndef HATARAKI_BASICMOTION_I2CBUS
#define HATARAKI_BASICMOTION_I2CBUS

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <inttypes.h>
#include <stdio.h>


#include <string>

#ifdef __linux__
#define I2C_SUPPORTED
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#endif

#include <stdexcept>
#include <sstream>

#include "TimeUtil.hpp"
#include "SimpleMovingAverage.hpp"

class I2CBus {

private:
	int file,currentAddr;
	std::string busName;

public:	
	I2CBus(std::string busNames);
	
	SimpleMovingAverage * writeTime, * readTime;
	void selectAddress(int addr);

	void writeToBus(unsigned char * buf, int length);
	void readFromBus(unsigned char * buffer, int length);
	
	int readWord(int address);
	int readByte(int address);
	
	void writeByte(int address, int data);
	void writeWord(int address, int data);
	
	void setRegister(unsigned char regAddr, unsigned char regValue);
	int getRegisterValue(unsigned char regAddr);	


};

#endif