/*
 * Sonar.cpp
 *
 *  Created on: 7 juil. 2013
 *      Author: Aberzen
 */

#include "../include/Sonar.hpp"

namespace sensor {

Sonar::Sonar() {
}

Sonar::~Sonar()
{
}

/** @brief Init the process */
status Sonar::initialize()
{
	return Sensor::initialize();
}

/** @brief Execute the process */
status Sonar::execute()
{
	return 0;
}


} /* namespace sensor */
