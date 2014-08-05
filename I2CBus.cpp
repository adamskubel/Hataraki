#include "I2CBus.hpp"

using namespace std;


#ifdef I2C_SUPPORTED

I2CBus::I2CBus(string busname)
{
	writeTime = new SimpleMovingAverage(100);	
	readTime = new SimpleMovingAverage(100);

	this->busName = busname;
	
    char filename[40];
	sprintf(filename,"%s",busname.c_str());
    if ((file = open(filename,O_RDWR)) < 0) {

		stringstream ss;
		ss << "Failed to open the bus. Name=" << filename << ", Error=" << strerror(errno);        
		throw std::runtime_error(ss.str());
    }
	else
	{
		cout << "Opened bus " << busname.c_str() << endl;
	}

}

int i2c_smbus_access(int file, char read_write, unsigned char command,int size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	return ioctl(file, I2C_SMBUS, &args);
}

int i2c_smbus_read_byte_data(int file, unsigned char command)
{
	union i2c_smbus_data data;
	if (i2c_smbus_access(file, I2C_SMBUS_READ, command,I2C_SMBUS_BYTE_DATA, &data))
		return -1;
	else
		return 0x0FF & data.byte;
}

int i2c_smbus_write_byte_data(int file, unsigned char command, unsigned char value)
{
	union i2c_smbus_data data;
	data.byte = value;
	return i2c_smbus_access(file, I2C_SMBUS_WRITE, command,I2C_SMBUS_BYTE_DATA, &data);
}

int i2c_smbus_read_word_data(int file, unsigned char command)
{
	union i2c_smbus_data data;
	if (i2c_smbus_access(file, I2C_SMBUS_READ, command,I2C_SMBUS_WORD_DATA, &data))
		return -1;
	else
		return 0x0FFFF & data.word;
}

int i2c_smbus_write_word_data(int file, unsigned char command, unsigned int value)
{
	union i2c_smbus_data data;
	data.word = value;
	return i2c_smbus_access(file, I2C_SMBUS_WRITE, command,I2C_SMBUS_WORD_DATA, &data);
}


void I2CBus::writeWord(int registerAddress, int data)
{
	i2c_smbus_write_word_data(file,(unsigned char)registerAddress,(unsigned int)data);
}

void I2CBus::writeByte(int registerAddress, int data)
{
	i2c_smbus_write_byte_data(file,(unsigned char)registerAddress,(unsigned char)data);
}

int I2CBus::readWord(int registerAddress)
{
	int res = 0;
//	timespec readStart = TimeUtil::getNow();
	res = i2c_smbus_read_word_data(file,(unsigned char)registerAddress);
//	readTime->add(TimeUtil::timeSince(readStart)*1000.0);
	return res;
}

int I2CBus::readByte(int registerAddress)
{
//	timespec readStart = TimeUtil::getNow();
	int res = i2c_smbus_read_byte_data(file,(unsigned char)registerAddress);
//	readTime->add(TimeUtil::timeSince(readStart)*1000.0);
	return res;
}

void I2CBus::selectAddress(int addr)
{
	if (currentAddr != addr)
	{
		if (ioctl(file,I2C_SLAVE_FORCE,addr) < 0) {
			stringstream ss;
			ss << "Failed to set address. TargetAddress = " << addr << ", Error=" << strerror(errno);
			throw std::runtime_error(ss.str());
		}
		currentAddr = addr;
	}
}

#else

I2CBus::I2CBus(string busname)
{
	
}

int I2CBus::readWord(int registerAddress)
{
	return 0;
}

int I2CBus::readByte(int registerAddress)
{
	return 0;
}

void I2CBus::writeWord(int registerAddress, int data)
{
}

void I2CBus::writeByte(int registerAddress, int data)
{

}

void I2CBus::selectAddress(int addr)
{
	
}
#endif


string I2CBus::getName()
{
	return busName;
}