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
		TimeMultiplexedVoltageConverter * tm = new TimeMultiplexedVoltageConverter(3,2.4);
		
		double t0 = 0.1;
		assertEquals(0,tm->nextVoltage(t0, 0));
		assertEquals(0,tm->nextVoltage(t0,0));
		//
		tm->reset();
		assertEquals(volts(10),tm->nextVoltage(t0,volts(10.4)));
		assertEquals(volts(11),tm->nextVoltage(t0,volts(10.4)));
		
		assertEquals(volts(11),tm->nextVoltage(t0,volts(10.6)));
		assertEquals(volts(10),tm->nextVoltage(t0,volts(10.6)));
		
		cout << "2" << endl;
		
		tm->reset();
		assertEquals(volts(10),tm->nextVoltage(t0,volts(10.4)));
		assertEquals(volts(11),tm->nextVoltage(t0,volts(10.4)));
		
		
		cout << "3" << endl;
		
		tm->reset();
		assertEquals("A",0.72,tm->nextVoltage(t0,0.69));
		assertEquals("B",0.64,tm->nextVoltage(t0,0.69));
		assertEquals("C",0.72,tm->nextVoltage(t0,0.69));
		assertEquals("D",0.64,tm->nextVoltage(t0,0.69));
		assertEquals("E",0.72,tm->nextVoltage(t0,0.69));
		
	}
	
};
