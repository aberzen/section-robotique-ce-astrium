/*
 * SensorDriver.cpp
 *
 *  Created on: 8 août 2013
 *      Author: Aberzen
 */

#include "../include/SensorDriver.hpp"

namespace hw {

SensorDriver::SensorDriver() :
	Driver(),
	_isAvailable(false)

{
}

SensorDriver::~SensorDriver() {
}

/** @brief Initialize the HW */
status SensorDriver::initialize()
{
	_isAvailable = false;
	return 0;
}

/** @brief Reset the HW */
status SensorDriver::reset()
{
	_isAvailable = false;
	return 0;
}

} /* namespace hw */
