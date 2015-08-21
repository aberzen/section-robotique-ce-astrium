/*
 * GpsPosition.hpp
 *
 *  Created on: 24 déc. 2013
 *      Author: Robotique
 */

#ifndef GPSPOSITION_HPP_
#define GPSPOSITION_HPP_

#include "GpsAngle.hpp"

namespace hw {

class GpsPosition {
public:
	GpsPosition();
	virtual ~GpsPosition();

public:
	GpsAngle latitude;
	GpsAngle longitude;
	float altitude;
};

} /* namespace hw */

#endif /* GPSPOSITION_HPP_ */
