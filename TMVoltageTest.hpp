#include "TimeMultiplexedVoltageConverter.hpp"

#include "TestUtil.hpp"
#include "DRV8830.hpp"

namespace HatarakiTest
{
	double volts(double steps)
	{
		return DRV8830::fractionalStepsToVoltage(steps);
	}
	
	void testTMVoltageConverter()
	{
		TimeMultiplexedVoltageConverter * tm = new TimeMultiplexedVoltageConverter(2,2.4);
		
		double t0 = 0.1;
		assertEquals(0,tm->nextVoltage(t0, 0));
		assertEquals(0,tm->nextVoltage(t0,0));
		//
		assertEquals(volts(11),tm->nextVoltage(t0,volts(10.4)));
		assertEquals(volts(11),tm->nextVoltage(t0,volts(10.4)));
		
		assertEquals(volts(10),tm->nextVoltage(t0,volts(10.6)));
		assertEquals(volts(11),tm->nextVoltage(t0,volts(10.6)));
		
		tm->reset();
		assertEquals(volts(10),tm->nextVoltage(t0,volts(10.4)));
		assertEquals(volts(11),tm->nextVoltage(t0,volts(10.4)));
		
		tm->reset();
		assertEquals(0.40, tm->nextVoltage(t0,0.32));
		assertEquals(0.24, tm->nextVoltage(t0,0.32));
		assertEquals(0.40, tm->nextVoltage(t0,0.32));
		
		tm->reset();
		assertEquals(0.40, tm->nextVoltage(t0,0.12));
		assertEquals(0.24, tm->nextVoltage(t0,0.12));
		assertEquals(0.24, tm->nextVoltage(t0,0.12));
		
		
	}
	
};
