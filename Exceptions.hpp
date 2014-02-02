#ifndef HATARAKI_BASICMOTION_EXCEPTIONS_HPP_
#define HATARAKI_BASICMOTION_EXCEPTIONS_HPP_

#include <stdexcept>

//Invalid input by user
class IllegalCommandException : public std::runtime_error
{
public:
	IllegalCommandException(std::string message):
	  std::runtime_error(message)
	{
		
	}
};

//Potential loss of control
class CriticalException : public  std::runtime_error
{
public:
	CriticalException(std::string message):
	  std::runtime_error(message)
	{
		
	}
};


//Configuration error
class ConfigurationException : public std::runtime_error
{
public:
	ConfigurationException(std::string message) :
	  std::runtime_error(message)
	{

	}

};



#endif
