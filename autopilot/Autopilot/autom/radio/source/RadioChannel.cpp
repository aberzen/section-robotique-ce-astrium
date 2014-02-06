/*
 * RadioChannel.cpp
 *
 *  Created on: 30 janv. 2014
 *      Author: Robotique
 */

#include <autom/radio/include/RadioChannel.hpp>
#include <math/include/MathMacro.hpp>

namespace autom {

RadioChannel::RadioChannel(
		/* Inputs */
		const uint16_t& pwmRawVal,
		/* Parameters */
		const Param& param
		)
: _pwmVal(pwmRawVal),
  _param(param)
{
}

RadioChannel::~RadioChannel()
{
}

/** @brief Read the channel value ensuring null value when pwm equals zero,
 * saturating pwm between min and max, and using scale to place pwm between [-1 and 1]*/
void RadioChannel::readChannel(float& val)
{
	int16_t pwmValSat = (int16_t)(math_max(_param.min, math_min(_param.max, _pwmVal)));

	/* Trig the zero val */
	int16_t signedPwmVal = pwmValSat - _param.trim;

	/* Convert using scale */
	val = ((float)(signedPwmVal)) * _param.scale;
}

} /* namespace autom */
