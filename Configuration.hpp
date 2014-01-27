#ifndef HATARAKI_BASICMOTION_CONFIGURATION_HPP_
#define HATARAKI_BASICMOTION_CONFIGURATION_HPP_

#include "cJSON.h"
#include <string>
#include <stdexcept>
#include <vector>
#include <sstream>

class Configuration {

private:
	static std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems)
	{
		std::stringstream ss(s);
		std::string item;
		while (std::getline(ss, item, delim)) {
			elems.push_back(item);
		}
		return elems;
	}
	
	
	static std::vector<std::string> split(const std::string &s, char delim)
	{
		std::vector<std::string> elems;
		split(s, delim, elems);
		return elems;
	}
	
	cJSON * root;

	
public:
	static bool CsvLoggingEnabled;
	static std::string CsvSeparator(",");
	
	static Configuration & getInstance()
	{
		static Configuration instance;
		return instance;
	}


	static void AssertConfigExists(cJSON * configItem,std::string configItemName) {

		if (configItem == NULL)
			throw std::runtime_error("Configuration error, JSON object " + configItemName + " is missing");

	}

	cJSON * getObject(std::string childPath);
	cJSON * getObject(cJSON * parent, std::string childPath);


};


#endif