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

#define RADIO_IDX_ROLL   	(0)
#define RADIO_IDX_PITCH  	(1)
#define RADIO_IDX_THRUST 	(2)
#define RADIO_IDX_YAW    	(3)
#define RADIO_IDX_OPT0	   	(4)
#define RADIO_IDX_OPT1 	  	(5)
#define RADIO_IDX_OPT2 	  	(6)
#define RADIO_IDX_OPT3   	(7)

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

public:
	Radio(const Parameter& param);
	virtual ~Radio();

	int16_t getSigned(uint8_t iChannel);
	inline int16_t getSignedMaxVal(uint8_t iChannel);
	inline int16_t getSignedMinVal(uint8_t iChannel);

	uint16_t getUnsigned(uint8_t iChannel);
	inline uint16_t getUnsignedMaxVal(uint8_t iChannel);

protected:

	inline bool isReversed(uint8_t idx);

	const Parameter& _param;
};

bool Radio::isReversed(uint8_t idx)
{
	uint8_t idx2 = idx>>4;
	uint8_t idx3 = idx - (idx2<<4);
	return ((_param.reversed[idx2] & (1<<idx3)) != 0);
}

int16_t Radio::getSignedMaxVal(uint8_t iChannel)
{
	if (iChannel < CNF_PWM_NUM_FROM_DEVICE)
		return _param.pwmMax[iChannel]-_param.pwmZero[iChannel];

	return 0;
}
int16_t Radio::getSignedMinVal(uint8_t iChannel)
{
	if (iChannel < CNF_PWM_NUM_FROM_DEVICE)
		return _param.pwmMin[iChannel]-_param.pwmZero[iChannel];

	return 0;
}
uint16_t Radio::getUnsignedMaxVal(uint8_t iChannel)
{
	if (iChannel < CNF_PWM_NUM_FROM_DEVICE)
		return (_param.pwmMax[iChannel]-_param.pwmMin[iChannel]);

	return 0;
}


} /* namespace hw */

#endif /* HW_RADIO_RADIO_HPP_ */
