/*
 * Magnetometer.cpp
 *
 *  Created on: 7 juil. 2013
 *      Author: Aberzen
 */

#include "../include/Magnetometer.hpp"

namespace sensor {

Magnetometer::Magnetometer(hw::HalMagnetometer& hal,
		math::Matrix3f& mat_UB,
		math::Vector3f& bias_B) :
	Sensor(),
	_hal(hal),
	_magFieldMeas(0.,0.,0.),
	_mat_UB(mat_UB),
	_bias_B(bias_B)
{
}

Magnetometer::~Magnetometer()
{
}

/** @brief Init the process */
status Magnetometer::initialize()
{
	return Sensor::initialize();
}

/** @brief Execute the process */
status Magnetometer::execute()
{
	_magFieldMeas = _mat_UB * _hal.readMagField() + _bias_B;

	/* Nothing else to do ? */
	return 0;
}

} /* namespace sensor */
