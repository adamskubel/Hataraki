#include "Configuration.hpp"

bool Configuration::CsvLoggingEnabled = false;
std::string Configuration::CsvSeparator = ",";

using namespace std;

cJSON * Configuration::getObject(std::string childPath)
{
	return getObject(root,childPath);
}

cJSON * Configuration::getObject(cJSON * parent, std::string childPath)
{
	vector<string> path = split(childPath,'.');
	
	cJSON * item = parent;
	
	for (auto it = path.begin(); it != path.end(); it++)
	{
		cJSON * nextItem = cJSON_GetObjectItem(item, it->c_str());
		if (!nextItem)
		{
			for (int i=0; i<cJSON_GetArraySize(item); i++)
			{
				cJSON * arrayItem = cJSON_GetArrayItem(item, i);
				cJSON * nameItem = cJSON_GetObjectItem(nextItem, "Name");
				
				if (nameItem && it->compare(nameItem->valuestring) == 0)
				{
					nextItem = arrayItem;
					cJSON_Delete(nameItem);
					break;
				}
				else
				{
					cJSON_Delete(arrayItem);
					cJSON_Delete(nameItem);
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

