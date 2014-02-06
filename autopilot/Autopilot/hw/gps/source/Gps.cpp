/*
 * Gps.cpp
 *
 *  Created on: 24 déc. 2013
 *      Author: Robotique
 */

#include <hw/gps/include/Gps.hpp>

namespace hw {

Gps::Gps(
		/* Outputs */
		Output& out
		)
: Driver(),
  _out(out)
{
	// TODO Auto-generated constructor stub

}

Gps::~Gps() {
	// TODO Auto-generated destructor stub
}


/** @brief Init the process */
void Gps::initialize()
{
	_out.isAvailable = false;
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
