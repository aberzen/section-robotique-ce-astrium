/*
 * PwmDevice.cpp
 *
 *  Created on: 3 janv. 2014
 *      Author: Robotique
 */

#include <hw/serial/include/FastSerial.hpp>
#include <hw/pwm/include/PwmDevice.hpp>

namespace hw {

PwmDevice::PwmDevice(
		/* Inputs */
		/* Outputs */
		uint16_t& channel,
		/* Param */
		uint8_t idxChannel,
		/* Dependencies */
		hw::Pwm& pwm
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
::infra::status PwmDevice::initialize()
{
	/* Switch the device off */
	switchOff();
}

/** @brief switch On */
void PwmDevice::switchOn()
{
//	Serial.printf("switchOn %d\n", _idxChannel);

	/* Enable the channel */
	_pwm.enable_out(_idxChannel);

	/* Force the channel output */
	_pwm.force_out(_idxChannel);
}

/** @brief switch Off */
void PwmDevice::switchOff()
{
	/* Force the channel output */
	_pwm.force_out(_idxChannel);

	/* Enable the channel */
	_pwm.disable_out(_idxChannel);
}


} /* namespace hw */
