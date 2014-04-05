#include "AsyncI2CSender.hpp"

using namespace std;

//map<string,shared_ptr<AsyncI2CSender> > AsyncI2CSender::senderMap;
map<string,AsyncI2CSender*> AsyncI2CSender::senderMap;

//shared_ptr<AsyncI2CSender> AsyncI2CSender::forBus(I2CBus * bus)
AsyncI2CSender* AsyncI2CSender::forBus(I2CBus * bus)
{
	auto res = senderMap.find(bus->getName());
	if (res == senderMap.end())
	{
		auto newBus = new AsyncI2CSender(bus); //make_shared<AsyncI2CSender>(bus);
		senderMap.insert(make_pair(bus->getName(),newBus));
		AsyncLogger::log("AsyncI2CSender for bus '" + bus->getName() + "' initialized");
		return newBus;
	}
	else
		return res->second;
}

void AsyncI2CSender::cleanup()
{
	for (auto it = senderMap.begin(); it != senderMap.end(); it++)
	{
		delete it->second;
	}
}

AsyncI2CSender::~AsyncI2CSender()
{
	AsyncLogger::log("AsyncI2CSender '" + bus->getName() + "' shut down");
	running = false;
	queueReadyCondition.notify_all();
	this->messageThread.join();
}

AsyncI2CSender::AsyncI2CSender(I2CBus * bus)
{
	this->bus = bus;
	this->running = true;
	this->messageThread = thread([this](){this->run();});
}

void AsyncI2CSender::postMessage(I2CMessage message)
{
	{
		unique_lock<mutex> locks(messageQueueMutex);
		messageQueue.push(message);
	}
	queueReadyCondition.notify_one();
}

I2CMessage AsyncI2CSender::popMessage()
{
	unique_lock<mutex> lock(messageQueueMutex);
	this->queueReadyCondition.wait(lock, [=]{ return !this->messageQueue.empty() || !this->running; });
	if (running)
	{
		I2CMessage rc(move(messageQueue.front()));
		messageQueue.pop();
		return rc;
	}
	else
	{
		return I2CMessage(0,0,0);
	}
}

void AsyncI2CSender::run()
{
	while (running)
	{		
		I2CMessage msg = popMessage();
		
		if (running)
		{
			bus->selectAddress(msg.Address);
			bus->writeByte(msg.Register, msg.Data);
		}
	}
}
