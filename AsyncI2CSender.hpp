#ifndef HATARAKI_BASICMOTION_IO_ASYNC_I2C_SENDER_HPP_
#define HATARAKI_BASICMOTION_IO_ASYNC_I2C_SENDER_HPP_

#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_1
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_2
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_4
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_8

#include <thread>
#include <map>
#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <memory>

#include "AsyncLogger.hpp"

#include "I2CBus.hpp"


struct I2CMessage {

	int Address;
	int Register;
	
	bool isWord;
	int Data;
	
	I2CMessage(int Address, int Register, int Data)
	{
		isWord = false;
		this->Address = Address;
		this->Register = Register;
		this->Data = Data;
	}

};

class AsyncI2CSender {

public:
//	static std::shared_ptr<AsyncI2CSender> forBus(I2CBus * bus);
	static AsyncI2CSender * forBus(I2CBus * bus);
	static void cleanup();

private:
	
	static std::map<std::string,AsyncI2CSender*> senderMap;
//	static std::map<std::string,std::shared_ptr<AsyncI2CSender>> senderMap;
	volatile bool running;
	
	I2CBus * bus;
	std::queue<I2CMessage> messageQueue;
	std::mutex messageQueueMutex;	
	std::thread messageThread;
    std::condition_variable queueReadyCondition;
	
	I2CMessage popMessage();


public:
	AsyncI2CSender(I2CBus * bus);
	~AsyncI2CSender();

	void postMessage(I2CMessage message);

	void run();
	

};

#endif