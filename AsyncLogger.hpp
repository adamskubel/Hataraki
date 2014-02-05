#ifndef HATARAKI_BASICMOTION_UTIL_ASYNCLOGGER_HPP_
#define HATARAKI_BASICMOTION_UTIL_ASYNCLOGGER_HPP_

#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_1
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_2
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_4
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_8

#include <fstream>
#include <thread>
#include <mutex>
#include <functional>
#include <queue>
#include <unordered_map>
#include <algorithm> 
#include <memory>
#include <iostream>
#include <chrono>
#include <string>
#include <unistd.h>

class AsyncLogger {

	struct LogTask
	{
		std::string filename;
		std::string message;

		LogTask(std::string _filename, std::string _message) :
			filename(_filename), message(_message)
		{

		}
	};


private:
	AsyncLogger() 
	{
		logThread = NULL;
		running = false;
	}

public:
	static AsyncLogger & getInstance()
	{
		static AsyncLogger instance;
		return instance;
	}

private:
	std::unordered_map<std::string,std::ofstream*> filemap;
	std::queue<LogTask*> taskQueue;
	std::mutex taskQueueMutex;
	std::thread * logThread;

	volatile bool running;

	void run();
	

public:
	//void postLogTask(std::string filename, std::function<void(std::ofstream&)> function);
	void postLogTask(std::string filename, std::string message);

	void startThread();
	void joinThread();




};


#endif