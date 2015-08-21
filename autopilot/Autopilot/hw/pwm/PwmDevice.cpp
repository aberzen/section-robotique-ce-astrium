/*
 * PwmDevice.cpp
 *
 *  Created on: 3 janv. 2014
 *      Author: Robotique
 */

#include <hw/pwm/PwmDevice.hpp>

namespace hw {

PwmDevice::PwmDevice(
		/* Inputs */
		/* Outputs */
		uint16_t& channel,
		/* Param */
		uint8_t idxChannel,
		/* Dependencies */
		hw::pwm_t& pwm
		)
: _channel(channel),
  _idxChannel(idxChannel),
  _pwm(pwm)
{
}

PwmDevice::~PwmDevice() {
	/* Switch the device off */
	switchOff();
}

/** @brief Init the process */
void PwmDevice::initialize()
{
	/* Switch the device off */
	switchOff();
}

/** @brief switch On */
void PwmDevice::switchOn()
{
	// TODO: Not implemented
}

/** @brief switch Off */
void PwmDevice::switchOff()
{
	// TODO: Not implemented
}


} /* namespace hw */
