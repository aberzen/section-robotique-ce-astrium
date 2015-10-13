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


int16_t Radio::getSigned(Channel channel)
{
	if (channel>=CNF_PWM_NUM_FROM_DEVICE)
		return 0;

	uint16_t tmp = math_min(
			math_max(
					system::system.dataPool.pwm_inputs[channel],
					_param.pwmMin[channel]),
			_param.pwmMax[channel]);

	if (isReversed(channel))
		return (int16_t) (_param.pwmZero[channel]-tmp);
	else
		return (int16_t) (tmp-_param.pwmZero[channel]);
}

uint16_t Radio::getUnsigned(Channel channel)
{
	if (channel>=CNF_PWM_NUM_FROM_DEVICE)
		return 0;

	uint16_t tmp = math_min(
			math_max(
					system::system.dataPool.pwm_inputs[channel],
					_param.pwmMin[channel]),
			_param.pwmMax[channel]);

	if (isReversed(channel))
		return (_param.pwmMax[channel]-tmp);
	else
		return (tmp-_param.pwmMin[channel]);
}



Radio::~Radio() {
	// TODO Auto-generated destructor stub
}

bool Radio::isReversed(uint8_t idx)
{
	uint8_t idx2 = idx>>4;
	uint8_t idx3 = idx - (idx2<<4);
	return ((_param.reversed[idx2] & (1<<idx3)) != 0);
}

int16_t Radio::getSignedMaxVal(Channel channel)
{
	if (channel < CNF_PWM_NUM_FROM_DEVICE)
		return _param.pwmMax[channel]-_param.pwmZero[channel];

	return 0;
}
int16_t Radio::getSignedMinVal(Channel channel)
{
	if (channel < CNF_PWM_NUM_FROM_DEVICE)
		return _param.pwmMin[channel]-_param.pwmZero[channel];

	return 0;
}
uint16_t Radio::getUnsignedMaxVal(Channel channel)
{
	if (channel < CNF_PWM_NUM_FROM_DEVICE)
		return (_param.pwmMax[channel]-_param.pwmMin[channel]);

	return 0;
}


} /* namespace hw */
