/*
 * Servo.cpp
 *
 *  Created on: 3 janv. 2014
 *      Author: Robotique
 */

#include <hw/servo/include/Servo.hpp>
#include <math/include/MathMacro.hpp>

namespace hw {

Servo::Servo(
		/* Inputs */
		/* Outputs */
		uint16_t& channel,
		/* Param */
		const Param& param,
		/* Dependencies */
		hw::Pwm& pwm)
: PwmDevice(channel, param.idxChannel,pwm),
  _param(param)
	{
	// TODO Auto-generated constructor stub

}

Servo::~Servo() {
	// TODO Auto-generated destructor stub
}

/** @brief switch On */
void Servo::switchOn()
{
	PwmDevice::switchOn();
	moveTo(0.);
}

/** @brief switch Off */
void Servo::switchOff()
{
	moveTo(0.);
	PwmDevice::switchOff();
}

void Servo::moveTo(float angle)
{
	/* Saturated angle */
	float angleSaturated = math_max(math_min(angle, _param.angleMax), _param.angleMin);

	/* Convert and write to channel */
	_channel = (uint16_t)(((int16_t)(angleSaturated * _param.scale)) + _param.offset);
//
//	Serial.printf("angle = %.4f\n", angle);
//	Serial.printf("angleSaturated = %.4f\n", angleSaturated);
//	Serial.printf("channel = %d\n", _channel);
//
}

} /* namespace hw */
