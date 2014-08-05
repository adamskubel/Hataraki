#include "RealtimeLoopController.hpp"
#include "TimeUtil.hpp"
#include "AsyncLogger.hpp"

using namespace std;

#define TimeDebug false

RealtimeLoopController::RealtimeLoopController()
{
	updateCount = 0;
	updatePeriod = Configuration::SamplePeriod;
	timeSMA_map.insert(make_pair("All",new SimpleMovingAverage(30)));
}

void RealtimeLoopController::start()
{
	running = true;
	rtThread = thread([this](){this->run();});
}

void RealtimeLoopController::stop()
{
	running = false;
	rtThread.join();
}

void RealtimeLoopController::run()
{	
	while (running)
	{
		update();
	}
	shutdown();
}

void RealtimeLoopController::update()
{
	timespec start, step;
	
	try
	{
		TimeUtil::setNow(start);
		
		for (auto it = loopWatchers.begin(); it != loopWatchers.end(); it++)
		{
//			if (TimeDebug) TimeUtil::setNow(step);
			(*it)->update();
//			if (TimeDebug) timeSMA_map[(*it)->getJointModel()->name]->add(TimeUtil::timeSince(step)*1000.0);
		}
//		if (TimeDebug) timeSMA_map["All"]->add(TimeUtil::timeSince(start)*1000.0);
		
		RealtimeDispatcher::getInstance().runTasks();
		
		updateCount++;
		TimeUtil::sleepFor(updatePeriod - TimeUtil::timeSince(start));
	}
	catch (std::runtime_error & e)
	{
		stop();
		stringstream ss;
		ss << "Exception thrown during control loop execution: " << e.what();
		AsyncLogger::log(ss);
		cout << ss.str() << endl;
	}
}

void RealtimeLoopController::shutdown()
{
	for (auto it = loopWatchers.begin(); it != loopWatchers.end(); it++)
	{
		(*it)->shutdown();
	}
}

void RealtimeLoopController::addWatcher(IRealtimeUpdatable * watcher)
{
	loopWatchers.push_back(watcher);
}

void RealtimeDispatcher::postTask(function<void()> task)
{
	lock_guard<mutex> locks(taskQueueMutex);
	taskQueue.push(task);
}

void RealtimeDispatcher::runTasks()
{
	timespec step;
	TimeUtil::setNow(step);
	if (taskQueueMutex.try_lock()) {
		while (!taskQueue.empty()) {
			try
			{
				taskQueue.front()();
			}
			catch (std::runtime_error & e)
			{
				stringstream ss;
				ss << "Exception executing scheduled task: " << e.what();
				AsyncLogger::log(ss);
			}
			taskQueue.pop();
		}
		taskQueueMutex.unlock();
	}
	TimeUtil::assertTime(step,"Task execution");
}

void RealtimeLoopController::printTimeDebugInfo()
{
	RealtimeDispatcher::AddTask([this](){
		for (auto it = timeSMA_map.begin(); it != timeSMA_map.end(); it++)
		{
			cout << it->first << " = " << it->second->avg() << endl;
		}
	});
}