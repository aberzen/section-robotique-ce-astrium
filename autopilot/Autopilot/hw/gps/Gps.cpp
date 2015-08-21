/*
 * Gps.cpp
 *
 *  Created on: 24 déc. 2013
 *      Author: Robotique
 */

#include <hw/gps/Gps.hpp>

namespace hw {

Gps::Gps(
		/* Outputs */
		Output& out
		)
: Driver(),
  _out(out)
{
}

Gps::~Gps() {
}


/** @brief Init the process */
bool Gps::initialize()
{
	_out.isAvailable = false;
	return true;
}

/** @brief Execute the process */
void Gps::execute()
{
}

/** @brief Reset the HW */
void Gps::reset()
{
}

} /* namespace hw */
