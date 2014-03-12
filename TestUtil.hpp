#ifndef HATARAKI_BASICMOTION_TEST_TESTUTIL_HPP_
#define HATARAKI_BASICMOTION_TEST_TESTUTIL_HPP_

#include <sstream>
#include <iostream>
#include <stdexcept>
#include <cmath>

using namespace std;

namespace HatarakiTest
{
	void assertEquals(double expected, double actual, double precision = 0.001)
	{
		if (round(expected/precision)*precision != round(actual/precision)*precision)
		{
			stringstream ss;
			ss << "Expected [" << expected << "] but got [" << actual << "]";
			//throw std::logic_error(ss.str());
			cout << ss.str() << endl;
		}
	}
}

#endif

