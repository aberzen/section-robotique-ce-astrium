/*
 * GpsAngle.hpp
 *
 *  Created on: 24 déc. 2013
 *      Author: Robotique
 */

#ifndef GPSANGLE_HPP_
#define GPSANGLE_HPP_

#include <stdint.h>

namespace hw {

class GpsAngle {
public:
	GpsAngle();
	virtual ~GpsAngle();

public:
	int16_t angle;
	int8_t frac_min;
	int8_t frac_sec;
};

} /* namespace hw */

#endif /* GPSANGLE_HPP_ */
