#ifndef HATARAKI_DEVICES_PCA9455_HPP_
#define HATARAKI_DEVICES_PCA9455_HPP_

#include <iostream>
#include <vector>

#include "I2CDevice.hpp"

class PCA9552 : public I2CDevice {
	
private:
	struct Registers
	{
		const static int MODE0 = 0x00;
		const static int MODE1 = 0x01;
		
		const static int PWM0 = 0x0A;
		const static int IREF0 = 0x22;
		
		const static int LEDOUT0 = 0x02;
		const static int LEDOUT1 = 0x03;
		const static int LEDOUT2 = 0x04;
		const static int LEDOUT3 = 0x05;
		
		
		
	};
	
	struct CellState
	{
		CellState()
		{
			Current = 0;
			DutyCycle = 0;
		}
		
		CellState(int current, int pwm) {
		
			this->Current = current;
			this->DutyCycle = pwm;
		}
		
		int Current;
		int DutyCycle;
//		float Intensity;

	public:
		bool equals(CellState &other)
		{
			return this->Current == other.Current && this->DutyCycle == other.DutyCycle;
		}
	};
	
	const int Cells = 16;
	const int PWMReg0 = 0x0A;
	const int CurrentReg0 = 0x22;
	
	std::vector<CellState> activeState;
	float maxCurrent;
	
	CellState generateNextCellState(int cellId, CellState currentState, float targetBrightness);
	
	float getIntensity(float targetBrightness, int cellId);
	int getCellId(int x, int y);
	
	void commitGridState(std::vector<CellState> nextState);
	
public:
	PCA9552(I2CBus * bus, int address);
	
	void writeDefaults();
	
	void setGridBrightness(std::vector<float> values);
	void setCellBrightness(int x, int y, float brightness);
	
	
	
};


#endif


