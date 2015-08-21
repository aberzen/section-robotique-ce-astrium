/*
 * RadioChannel.cpp
 *
 *  Created on: 30 janv. 2014
 *      Author: Robotique
 */

#include <autom/radio/RadioChannel.hpp>
#include <math/MathMacro.hpp>

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
void RadioChannel::readChannel(int16_t& signedPwmVal)
{
	int16_t pwmValSat = math_max(_param.min, math_min(_param.max, (int16_t)_pwmVal));

	/* Trig the zero val */
	signedPwmVal = pwmValSat - _param.trim;
}


/** @brief Get min value computed as trim-min*/
void RadioChannel::getMin(int16_t& min)
{
	min = _param.min-_param.trim;
}

/** @brief Get max value computed as trim+max*/
void RadioChannel::getMax(int16_t& max)
{
	max = _param.max-_param.trim;
}


} /* namespace autom */
