#include "ADS1015.hpp"
#include <iostream>

using namespace std;

ADS1015::ADS1015(I2CBus * bus, int address) : I2CDevice(bus,address)
{
	
}

void ADS1015::setChannel(int channel)
{
	if (channel > 3 || channel < 0) throw std::logic_error("ADS1015 channel must be between 0 and 3");
	
	config.mux = 4 + channel;
}

double ADS1015::convResult(int result)
{
	//2's complement conversion
	
	int lsb = (result & 0xFF00) >> 8;
	int msb = (result & 0x00FF);
	
	result = ((msb << 8) + lsb) >> 4;
	if ((result & 0x0800) != 0) result -= 0xFFF;
	
	double scale = 4.096/ ((double)0x7FF);
	
	return (double)result * scale;
}

double ADS1015::readVoltage()
{
	config.startSingleShotRead = true;
	config.pga = 1;
	
	int configWord = config.buildConfigWord();
	
	writeWord(ADS1015::Registers::Config, configWord);
//	cout << std::hex << "Wrote word: " << configWord << endl;
	
	TimeUtil::sleepFor(0.01); //10ms
	
//	int confCheck = readWord(ADS1015::Registers::Config);
//	cout << std::hex << "Set: " << configWord << " Read:" << confCheck << endl;
	
	int result = readWord(ADS1015::Registers::Conversion);
	double volts = convResult(result);
//	cout << std::hex <<  "Result = " << result << " Voltage = " << volts << endl;
	return volts;
}


