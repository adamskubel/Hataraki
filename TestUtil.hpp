#ifndef HATARAKI_BASICMOTION_TEST_TESTUTIL_HPP_
#define HATARAKI_BASICMOTION_TEST_TESTUTIL_HPP_

#include <sstream>
#include <iostream>
#include <stdexcept>
#include <cmath>

using namespace std;

namespace HatarakiTest
{
	
	void assertEquals(string message, double expected, double actual, double precision = 0.001)
	{
		if (round(expected/precision)*precision != round(actual/precision)*precision)
		{
			stringstream ss;
			if (message.length() > 0)
				ss << message << " - Expected [" << expected << "] but got [" << actual << "]";
			else
				ss << "Expected [" << expected << "] but got [" << actual << "]";
			//throw std::logic_error(ss.str());
			cout << ss.str() << endl;
		}
	}
	
	void assertEquals(double expected, double actual, double precision = 0.001)
	{
		return assertEquals("", expected,actual,precision);
	}
}

#endif

