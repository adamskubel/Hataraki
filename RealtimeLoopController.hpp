#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_1
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_2
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_4
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_8

#ifndef __BasicMotion_XCode__JointLoopController__
#define __BasicMotion_XCode__JointLoopController__

#include <map>
#include <vector>
#include <iostream>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

#include "SimpleMovingAverage.hpp"
#include "PredictiveJointController.hpp"
#include "IRealtimeUpdatable.hpp"


class RealtimeLoopController {
	
public:
	RealtimeLoopController(std::vector<PredictiveJointController*> controllers);
	
	void start();
	void stop();
	
	void printTimeDebugInfo();
	
	void addWatcher(IRealtimeUpdatable * watcher);
	
private:
	std::map<std::string,SimpleMovingAverage*> timeSMA_map;
	std::vector<PredictiveJointController*> jointControllers;
	std::vector<IRealtimeUpdatable*> loopWatchers;
	
	std::thread jointLoopThread;
	
	volatile bool running;

	double updatePeriod;
	long updateCount;
	
	void update();
	void shutdown();

	void run();
	
};

class RealtimeDispatcher {

private:	
	std::queue<std::function<void()> > taskQueue;
	std::mutex taskQueueMutex;
		
public:
	void postTask(std::function<void()> task);
	void runTasks();
	static RealtimeDispatcher & getInstance()
	{
		static RealtimeDispatcher instance;
		return instance;
	}
	
public:
	static void AddTask(std::function<void()> task)
	{
		getInstance().postTask(task);
	}

};

#endif