/*
 * Ms5611Barometer.cpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#include <infra/include/Task.hpp>
#include "../include/Ms5611Barometer.hpp"

namespace sensor {

Ms5611Barometer::Ms5611Barometer(hw::HalBarometerMs5611OverSpi& hal) :
		Barometer(hal),
		_prePro(hal)
{
}

Ms5611Barometer::~Ms5611Barometer() {
}


/** @brief Init the process */
status Ms5611Barometer::initialize()
{
	/* Initialize super */
	if (0 > Barometer::initialize())
		return -1;

	/* Ok */
	return 0;
}

/** @brief Execute the process */
status Ms5611Barometer::execute()
{
	_isAvailable = _hal.isAvailable();
	if (_isAvailable)
	{
		/* Read raw pressure */
		const uint32_t& rawPressure = _hal.readRawPressure();

		/* Read raw pressure */
		const uint32_t& rawTemperature = _hal.readRawTemperature();

		/* Pre process the data */
		_prePro.preProcess(
				rawPressure,
				rawTemperature,
				(float&)_pressure,
				(int16_t&)_temperature);
	}

	return 0;
}


} /* namespace sensor */
