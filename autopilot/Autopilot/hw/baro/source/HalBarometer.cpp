/*
 * HalBarometer.cpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#include "../include/HalBarometer.hpp"

namespace hw {

HalBarometer::HalBarometer() :
	SensorDriver(),
	_rawPressure(0),
	_rawTemperature(0)
{
}

HalBarometer::~HalBarometer() {
}

} /* namespace hw */
