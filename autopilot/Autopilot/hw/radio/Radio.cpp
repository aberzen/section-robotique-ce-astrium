/*
 * Radio.cpp
 *
 *  Created on: 25 août 2015
 *      Author: AdministrateurLocal
 */

#include <system/system/System.hpp>
#include <hw/radio/Radio.hpp>
#include <math/MathUtils.hpp>

namespace hw {

Radio::Radio(const Parameter& param)
: _param(param)
{
}


int16_t Radio::getSigned(uint8_t iChannel)
{
	if (iChannel>=CNF_PWM_NUM_FROM_DEVICE)
		return 0;

	uint16_t tmp = math_min(
			math_max(
					system::system.dataPool.pwm_inputs[iChannel],
					_param.pwmMin[iChannel]),
			_param.pwmMax[iChannel]);

	if (isReversed(iChannel))
		return (int16_t) (_param.pwmZero[iChannel]-tmp);
	else
		return (int16_t) (tmp-_param.pwmZero[iChannel]);
}

uint16_t Radio::getUnsigned(uint8_t iChannel)
{
	if (iChannel>=CNF_PWM_NUM_FROM_DEVICE)
		return 0;

	uint16_t tmp = math_min(
			math_max(
					system::system.dataPool.pwm_inputs[iChannel],
					_param.pwmMin[iChannel]),
			_param.pwmMax[iChannel]);

	if (isReversed(iChannel))
		return (_param.pwmMax[iChannel]-tmp);
	else
		return (tmp-_param.pwmMin[iChannel]);
}



Radio::~Radio() {
	// TODO Auto-generated destructor stub
}

} /* namespace hw */
