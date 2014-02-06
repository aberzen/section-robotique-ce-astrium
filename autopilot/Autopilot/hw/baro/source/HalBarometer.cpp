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
void HalBarometer::initialize()
{
	_out.isAvailable = false;
}

/** @brief Reset the HW */
void HalBarometer::reset()
{
	_out.isAvailable = false;
}

} /* namespace hw */
