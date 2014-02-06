/*
 * PwmDevice.hpp
 *
 *  Created on: 3 janv. 2014
 *      Author: Robotique
 */

#ifndef PWMDEVICE_HPP_
#define PWMDEVICE_HPP_

#include <hw/pwm/include/Pwm.hpp>

namespace hw {

class PwmDevice {
public:
	virtual ~PwmDevice();

	/** @brief Init the process */
	virtual void initialize();

	/** @brief switch On */
	virtual void switchOn();

	/** @brief switch Off */
	virtual void switchOff();

protected:
	PwmDevice(
			/* Inputs */
			/* Outputs */
			uint16_t& channel,
			/* Param */
			uint8_t idxChannel,
			/* Dependencies */
			hw::Pwm& pwm
			);

protected:
	/** @brief Pwm value */
	uint16_t& _channel;

	/** @brief Channel index */
	uint8_t _idxChannel;

	/** @brief Pwm interface */
	hw::Pwm& _pwm;

};

} /* namespace hw */

#endif /* PWMDEVICE_HPP_ */
