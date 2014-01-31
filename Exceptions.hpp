#ifndef HATARAKI_BASICMOTION_EXCEPTIONS_HPP_
#define HATARAKI_BASICMOTION_EXCEPTIONS_HPP_

#include <stdexcept>

//Invalid input by user
class IllegalCommandException : std::runtime_error
{
public:
	IllegalCommandException(std::string message)
	{
		
	}
};

//Potential loss of control
class CriticalException : std::runtime_error
{
public:
	CriticalException(std::string message)
	{
		
	}
};



#endif
