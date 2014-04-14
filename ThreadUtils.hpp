#ifndef HATARAKI_BASICMOTION_UTIL_THREADUTILS_HPP_
#define HATARAKI_BASICMOTION_UTIL_THREADUTILS_HPP_

#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_1
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_2
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_4
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_8

#include <pthread.h>
#include <sched.h>

class ThreadUtils {
	
public:
	static void requestPriority(int priority)
	{
		pthread_t this_thread = pthread_self();
	
		struct sched_param params;
		// We'll set the priority to the maximum.
		params.sched_priority = sched_get_priority_max(priority);
		int ret = pthread_setschedparam(this_thread, priority, &params);
	
		if (ret != 0) {
			// Print the error
			std::cout << "Unsuccessful in setting thread realtime prio" << std::endl;
			return;
		}
	}
};

#endif