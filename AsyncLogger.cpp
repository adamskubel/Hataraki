#include "AsyncLogger.hpp"

using namespace std;


string AsyncLogger::LogFileName = "basicmotion.log";
string AsyncLogger::DefaultLogTag = "";

void AsyncLogger::postLogTask(string filename, string message)
{
	std::lock_guard<std::mutex> locks(taskQueueMutex);
	taskQueue.push(new LogTask(filename,message));
}


void AsyncLogger::postConsoleOutTask(string message)
{
	std::lock_guard<std::mutex> locks(taskQueueMutex);	
	taskQueue.push(new LogTask("cout",message));
}



void AsyncLogger::log(std::string message)
{	
	log(DefaultLogTag,message);
}

void AsyncLogger::log(stringstream & messageStream)
{	
	log(DefaultLogTag,messageStream.str());
}

void AsyncLogger::log(std::string tag, std::string message)
{
	stringstream ss;
	
	ss << std::round(TimeUtil::timeSince(TimeUtil::ApplicationStart)/0.001)*0.001 << " - ";
	if (tag.length() > 0)
		ss << "[" << tag << "] " << message;
	else
		ss << message;

	ss << endl;
	getInstance().postLogTask(LogFileName, ss.str());
}
void AsyncLogger::log(std::string tag, std::stringstream & messageStream)
{
	log(tag,messageStream.str());
}


void AsyncLogger::run()
{
	while (running)
	{
		try
		{
			taskQueueMutex.lock();

			if (taskQueue.empty())
			{
				taskQueueMutex.unlock();
				usleep(1000);
				//this_thread::sleep_for(std::chrono::milliseconds(1));
				continue;
			}

			LogTask * task = taskQueue.front(); 		
			taskQueue.pop();
			taskQueueMutex.unlock();

			if (task->filename.compare("cout") == 0)
			{
				cout << task->message;
			}
			else
			{
				std::ofstream * logFile;
				auto it = filemap.find(task->filename);
				if (it == filemap.end())
				{
					logFile = new ofstream(task->filename.c_str(),std::ofstream::out);
					filemap.insert(std::make_pair(task->filename,logFile));
				}
				else
					logFile = it->second;

				*logFile << task->message;

				logFile->flush();
			}
			delete task;
		}
		catch (std::exception & e)
		{
			cout << "AsyncLogger exception. " << e.what() << endl;
			running = false;
		}
	}


	try
	{
		for (auto it = filemap.begin(); it != filemap.end(); it++)
		{
			it->second->close();
		}
	}
	catch (std::exception & e)
	{
		cout << "AsyncLogger exception while closing: " << e.what() << endl;	
	}	
}

void AsyncLogger::startThread()
{
	if (!running)
	{
		running = true;
		logThread = new std::thread([this](){ this->run();});
	}
	else
		cout << "Log thread already running." << endl;
}

void AsyncLogger::joinThread()
{
	if (running)
	{
		running = false;
		cout << "Shutting down logging thread." << endl;
		if (logThread != NULL)
		{
			logThread->join();
			delete logThread;
			logThread = NULL;
		}
		else
		{
			cout << "Thread not defined??" << endl;
		}
	}
	else
	{
		cout << "Log thread not started." << endl;
	}
}