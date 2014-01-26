#ifndef HATARAKI_BASICMOTION_CONFIGURATION_HPP_
#define HATARAKI_BASICMOTION_CONFIGURATION_HPP_

#include "cJSON.h"
#include <string>
#include <stdexcept>

class Configuration {

public:
	static bool CsvLoggingEnabled;


	static void AssertConfigExists(cJSON * configItem,std::string configItemName) {

		if (configItem == NULL)
			throw std::runtime_error("Configuration error, JSON object " + configItemName + " is missing");

	}


};


#endif