/*
 * Gps.cpp
 *
 *  Created on: 7 juil. 2013
 *      Author: Aberzen
 */

#include "../include/Gps.hpp"

namespace sensor {

Gps::Gps() {
}

Gps::~Gps() {
}

/** @brief Init the process */
status Gps::initialize()
{
	/* Super */
	return Sensor::initialize();
}

/** @brief Execute the process */
status Gps::execute()
{
	return 0;
}


} /* namespace sensor */
