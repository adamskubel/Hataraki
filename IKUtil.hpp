#ifndef HATARAKI_BASICMOTION_IKUTIL_HPP_
#define HATARAKI_BASICMOTION_IKUTIL_HPP_

#include "MathUtils.hpp"


class IKUtil {

public:
	static vmath::Vector3d getPosition(std::vector<double> angles);
	
};


#endif
