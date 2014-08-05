#ifndef HATARAKI_BASICMOTION_CONFIGURATION_HPP_
#define HATARAKI_BASICMOTION_CONFIGURATION_HPP_

#include <string>
#include <stdexcept>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cmath>

#define VMATH_NAMESPACE vmath
#include "vmath.h"
#include "cJSON.h"

#include "MathUtils.hpp"

#include "Exceptions.hpp"

class ConfigurationObject {

protected:
	cJSON * rootObject;
	ConfigurationObject(cJSON * rootObject);

public:
	double getDouble(std::string key);
	std::string getString(std::string key);
	bool getBool(std::string key);
	int getInteger(std::string key);
	cJSON * getObject(std::string key);

};

class Configuration {

private:
	Configuration();	
	Configuration(Configuration const&);
	void operator=(Configuration const&); 
	~Configuration();

	cJSON * root;

	
public:
	static std::string get_file_contents(const char *filename);
	static cJSON * loadJsonFile(std::string filename);

	static bool CsvLoggingEnabled;
	static bool FastLogging;
	static char CsvSeparator;
	static double SamplePeriod;
	static bool AsyncDriverCommunication;

	static Configuration & getInstance();
	
	static void AssertConfigExists(cJSON * configItem,std::string configItemName);
	static vmath::Vector3d getVectorFromJSON(cJSON * vectorObj);
	static vmath::Vector2d getVector2dFromJSON(cJSON * vectorObj);
	static std::vector<double> getVoltagePatternFromJSON(cJSON * vectorObj);


	cJSON * getRoot();

	void loadConfig(std::string configFileName);

	cJSON * getObject(std::string childPath);
	cJSON * getObject(cJSON * parent, std::string childPath);
	


};


#endif