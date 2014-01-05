/*
 * HalBarometer.cpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#include "../include/HalBarometer.hpp"

namespace hw {

HalBarometer::HalBarometer(
		/* Inputs */
		/* Outputs */
		Output& out
		/* Parameters */
) :
	Driver(),
	_out(out)
{
}

HalBarometer::~HalBarometer() {
}

/** @brief Initialize the HW */
infra::status HalBarometer::initialize()
{
	_out.isAvailable = false;
	return 0;
}

/** @brief Reset the HW */
infra::status HalBarometer::reset()
{
	_out.isAvailable = false;
	return 0;
}

} /* namespace hw */
