/*
 * Radio.hpp
 *
 *  Created on: 25 août 2015
 *      Author: AdministrateurLocal
 */

#ifndef HW_RADIO_RADIO_HPP_
#define HW_RADIO_RADIO_HPP_

#include <stdint.h>
#include <system/params/Nrd.hpp>

namespace hw {

class Radio {
public:
	typedef struct
	{
		uint16_t reversed[(CNF_PWM_NUM_FROM_DEVICE+15)>>4];
		uint16_t pwmZero[CNF_PWM_NUM_FROM_DEVICE];
		uint16_t pwmMin[CNF_PWM_NUM_FROM_DEVICE];
		uint16_t pwmMax[CNF_PWM_NUM_FROM_DEVICE];
	} Parameter ;

	typedef enum
	{
		E_RADIO_CHANNEL_ROLL = 0,
		E_RADIO_CHANNEL_PITCH,
		E_RADIO_CHANNEL_THRUST,
		E_RADIO_CHANNEL_YAW,
		E_RADIO_CHANNEL_OPT0,
		E_RADIO_CHANNEL_OPT1,
		E_RADIO_CHANNEL_OPT2,
		E_RADIO_CHANNEL_OPT3
	} Channel;

public:
	Radio(const Parameter& param);
	virtual ~Radio();

	int16_t getSigned(Channel channel);
	int16_t getSignedMaxVal(Channel channel);
	int16_t getSignedMinVal(Channel channel);

	uint16_t getUnsigned(Channel channel);
	uint16_t getUnsignedMaxVal(Channel channel);

protected:

	bool isReversed(uint8_t idx);

	const Parameter& _param;
};


} /* namespace hw */

#endif /* HW_RADIO_RADIO_HPP_ */
