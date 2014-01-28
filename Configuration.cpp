#include "Configuration.hpp"

bool Configuration::CsvLoggingEnabled = false;
std::string Configuration::CsvSeparator = ",";

using namespace std;

Configuration::Configuration()
{	
	root = NULL;
}

Configuration::~Configuration()
{	
	if (root != NULL)
		cJSON_Delete(root);
}

Configuration & Configuration::getInstance()
{
	static Configuration instance;
	return instance;
}

cJSON * Configuration::getObject(std::string childPath)
{
	return getObject(root,childPath);
}

void Configuration::AssertConfigExists(cJSON * configItem,std::string configItemName) 
{
	if (configItem == NULL)
		throw std::runtime_error("Configuration error, JSON object " + configItemName + " is missing");

	//std::cout << "Item " << configItemName << " exists" << std::endl;
}


//std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems)
void split(std::string s, char delim, std::vector<std::string> &elems)
{
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	//return elems;
}
	
	
//std::vector<std::string> split(const std::string &s, char delim)
//{
//	std::vector<std::string> elems;
//	split(s, delim, elems);
//	return elems;
//}
//	

cJSON * Configuration::getObject(cJSON * parent, std::string childPath)
{
	if (!parent) 
	{
		throw runtime_error("Configuration::getObject - Invalid parent object.");
	}
	cJSON * item = parent;

	vector<string> path;
	split(childPath,'.',path);
		
	for (auto it = path.begin(); it != path.end(); it++)
	{		
		cJSON * nextItem = cJSON_GetObjectItem(item, it->c_str());
		
		int itemArraySize = cJSON_GetArraySize(item);
		if (!nextItem && itemArraySize > 0)
		{
			for (int i=0; i<cJSON_GetArraySize(item); i++)
			{
				cJSON * arrayItem = cJSON_GetArrayItem(item, i);
				cJSON * nameItem = cJSON_GetObjectItem(arrayItem, "Name");
				
				if (nameItem && it->compare(nameItem->valuestring) == 0)
				{
					nextItem = arrayItem;
					//cJSON_Delete(nameItem);
					break;
				}
				else
				{
					//cJSON_Delete(arrayItem);
					//cJSON_Delete(nameItem);
				}
			}
		}
		
		if (!nextItem)
		{
			throw runtime_error("Unable to find JSON object '" + childPath + "'. Closest=" + *it);
		}
		item = nextItem;
	}
	return item;
}




std::string get_file_contents(const char *filename)
{
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  if (in)
  {
    std::string contents;
    in.seekg(0, std::ios::end);
    contents.resize(in.tellg());
    in.seekg(0, std::ios::beg);
    in.read(&contents[0], contents.size());
    in.close();
    return(contents);
  }
  throw(errno);
}

void Configuration::loadConfig(std::string configFileName)
{	
	cout << "Opening JSON configuration file " << configFileName;
	std::string configString = get_file_contents(configFileName.c_str());

	cout << "Done. Parsing...";
	root = cJSON_Parse(configString.c_str());

	if (!root)
	{		
		cout << "Error parsing configuration. Exiting." << endl;
		throw std::runtime_error("Invalid JSON syntax");
	}	 
	cout << "Done." << endl;
					
	cout << "Config Version = " << cJSON_GetObjectItem(root,"Version")->valuestring << endl;

	cJSON * dataRecordingConfig = cJSON_GetObjectItem(root,"DataRecording");
	Configuration::CsvLoggingEnabled = (cJSON_GetObjectItem(dataRecordingConfig,"Enabled")->valueint != 0);

}


cJSON * Configuration::getRoot()
{
	return root;
}