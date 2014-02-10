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

#include "Exceptions.hpp"

class Configuration {

private:
	//static std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
	//static std::vector<std::string> split(const std::string &s, char delim);
	
	Configuration();	
	Configuration(Configuration const&);
	void operator=(Configuration const&); 
	~Configuration();

	cJSON * root;

	
public:
	static std::string get_file_contents(const char *filename);

	static bool CsvLoggingEnabled;
	static std::string CsvSeparator;
	
	static Configuration & getInstance();
	
	static void AssertConfigExists(cJSON * configItem,std::string configItemName);
	static vmath::Vector3d getVectorFromJSON(cJSON * vectorObj);
	static std::vector<double> getVoltagePatternFromJSON(cJSON * vectorObj);


	cJSON * getRoot();

	void loadConfig(std::string configFileName);

	cJSON * getObject(std::string childPath);
	cJSON * getObject(cJSON * parent, std::string childPath);
	


};


#endif