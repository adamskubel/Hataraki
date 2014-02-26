#include "I2CBus.hpp"

using namespace std;


I2CBus::I2CBus(const char * busname)
{
	
}

void I2CBus::writeToBus(unsigned char * buf, int length)
{
}

void I2CBus::readFromBus(unsigned char * buffer, int length)
{
}

void I2CBus::selectAddress(int addr)
{
	
}

#ifdef I2C_SUPPORTED

I2CBus::I2CBus(const char * busname)
{

    char filename[40];
	sprintf(filename,"%s",busname);
    if ((file = open(filename,O_RDWR)) < 0) {

		stringstream ss;
		ss << "Failed to open the bus. Name=" << filename << ", Error=" << strerror(errno);        
		throw std::runtime_error(ss.str());
    }	   

	sma = new SimpleMovingAverage(10);
}

void I2CBus::writeToBus(unsigned char * buf, int length)
{
	int res = 0;
	if ((res = write(file,buf,length)) != length) {		
		stringstream ss;
		ss << "Failed to write to the i2c bus. Address = " << currentAddr << ", Error=" << strerror(errno);        
		throw std::runtime_error(ss.str());
	}
}

void I2CBus::readFromBus(unsigned char * buffer, int length)
{
	//unsigned char * buf = new unsigned char[length];
	if (read(file,buffer,length) != length) 	{
		stringstream ss;
		ss << "Failed to read from the i2c bus. Address = " << currentAddr << ", Error=" << strerror(errno);        
		throw std::runtime_error(ss.str());
	}
}

void I2CBus::selectAddress(int addr)
{
	if (currentAddr != addr)
	{
		if (ioctl(file,I2C_SLAVE,addr) < 0) {
			stringstream ss;
			ss << "Failed to set address. TargetAddress = " << addr << ", Error=" << strerror(errno);        
			throw std::runtime_error(ss.str());
		}
		currentAddr = addr;
	}
}


double I2CBus::getAverageDataRate()
{
	return sma->avg();
}

#endif