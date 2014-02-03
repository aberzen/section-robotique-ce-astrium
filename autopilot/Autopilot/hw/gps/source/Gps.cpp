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
infra::status Gps::initialize()
{
	_out.isAvailable = false;
	return 0;
}

/** @brief Execute the process */
infra::status Gps::execute()
{
	return 0;
}

/** @brief Reset the HW */
infra::status Gps::reset()
{
	return 0;
}

} /* namespace hw */
