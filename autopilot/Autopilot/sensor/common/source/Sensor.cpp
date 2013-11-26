/*
 * Sensor.cpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#include "../include/Sensor.hpp"

namespace sensor {

Sensor::Sensor() :
		_isAvailable(false)
{
}

Sensor::~Sensor() {
}

/** @brief Init the process */
status Sensor::initialize()
{
	_isAvailable = false;

	return 0;
}

} /* namespace sensor */
