#include "PCA9552.hpp"

using namespace std;

PCA9552::PCA9552(I2CBus * bus, int address) : I2CDevice(bus,address)
{
	writeDefaults();
	
	
	
	activeState.clear();
	for (int i=0;i<Cells;i++)
	{
		activeState.push_back(CellState(0,0));
	}
}

void PCA9552::writeDefaults()
{
	//Disable autoincrement, subaddresses, and allcall address
	writeByte(Registers::MODE0,0b10000000);
	
	//Set all LED output states to individual PWM control only, disable group PWM
	writeByte(Registers::LEDOUT0,0b10101010);
	writeByte(Registers::LEDOUT1,0b10101010);
	writeByte(Registers::LEDOUT2,0b10101010);
	writeByte(Registers::LEDOUT3,0b10101010);
	
	cout << "Writing PCA9552 defaults" << endl;
}

void PCA9552::setGridBrightness(vector<float> brightness)
{
	if (brightness.size() < Cells) throw std::runtime_error("Frame definition must be 16 elements in size");
	vector<CellState> newState(Cells);
	for (int i=0; i < Cells; i++)
	{
		newState[i] = generateNextCellState(i, activeState[i], brightness[i]);
	}
	commitGridState(newState);
}

//Need some sort of function to map intensity to brightness (brightness = perceived intensity)
float PCA9552::getIntensity(float targetBrightness, int cellId)
{
	return targetBrightness;
}


/**
 Generate a new cell state based on requested brightness. 
 This determines both current and duty cycle. Testing required to understand practical relationships.
 */
PCA9552::CellState PCA9552::generateNextCellState(int cellId, CellState currentState, float targetBrightness)
{
	int cellIntensity = std::round(getIntensity(targetBrightness, cellId) * 65535.f);
	
	if (cellIntensity <= 0)
	{
		return CellState(0,0);
	}
	else
	{
		int currentValue = (cellIntensity & 0xFF00) >> 8;
		
//		cout << "CurrentValue = " << currentValue << endl;
		currentValue = min(220,currentValue);
		currentValue = max(5,currentValue);
		
		int pwmValue = cellIntensity & 0x00FF;
//		cout << "PWM1 = " << pwmValue << endl;
		pwmValue = min(255,pwmValue);
		pwmValue = max(10,pwmValue);
		
//		cout << "Intensity = " << cellIntensity << " Current = " << currentValue << " PWM = " << pwmValue << endl;
		return CellState(currentValue,pwmValue);
	}
}

//Inefficient cell-by-cell write, need to figure out block writes
void PCA9552::commitGridState(std::vector<CellState> nextState)
{
	for (int i=0; i < Cells; i++)
	{
		CellState active = activeState[i];
		CellState next = nextState[i];
		
		//TODO: Block i2c writes
		if (active.DutyCycle != next.DutyCycle)
			writeByte(PWMReg0+i,next.DutyCycle);
		if (active.Current != next.Current)
			writeByte(CurrentReg0+i,next.Current);
	}
	
	activeState = nextState;
}


















