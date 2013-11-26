/*
 * Barometer.cpp
 *
 *  Created on: 24 mars 2013
 *      Author: Aberzen
 */

#include "../include/Barometer.hpp"

namespace sensor {

Barometer::Barometer(hw::HalBarometer& hal) :
		Sensor(),
		_pressure(0),
		_temperature(0),
		_hal(hal)
{
}

Barometer::~Barometer() {
}

} /* namespace sensor */
