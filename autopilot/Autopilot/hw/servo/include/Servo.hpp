/*
 * Servo.hpp
 *
 *  Created on: 3 janv. 2014
 *      Author: Robotique
 */

#ifndef SERVO_HPP_
#define SERVO_HPP_

#include <hw/pwm/include/PwmDevice.hpp>

namespace hw {

class Servo: public hw::PwmDevice {
public:
	typedef struct
	{
		float angleMax;
		float angleMin;
		float scale;
		int16_t offset;
		uint8_t idxChannel;
	} Param;
public:
	Servo(
			/* Inputs */
			/* Outputs */
			uint16_t& channel,
			/* Param */
			const Param& param,
			/* Dependencies */
			hw::Pwm& pwm
			);
	virtual ~Servo();

	void moveTo(float angle);

	/** @brief switch On */
	virtual void switchOn();

	/** @brief switch Off */
	virtual void switchOff();

protected:
	const Param& _param;
};

} /* namespace hw */

#endif /* SERVO_HPP_ */
