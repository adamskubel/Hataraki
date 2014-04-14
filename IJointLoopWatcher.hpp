#ifndef __BasicMotion_XCode__IJointLoopWatcher__
#define __BasicMotion_XCode__IJointLoopWatcher__


class IJointLoopWatcher {
	
public:
	virtual void update() = 0;
	virtual void shutdown() = 0;
	
};

#endif
 