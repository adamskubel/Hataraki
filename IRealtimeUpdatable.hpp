#ifndef __BasicMotion_XCode__IRealtimeUpdatable__
#define __BasicMotion_XCode__IRealtimeUpdatable__


class IRealtimeUpdatable {
	
public:
	virtual void update() = 0;
	virtual void shutdown() = 0;
	
};

#endif
 