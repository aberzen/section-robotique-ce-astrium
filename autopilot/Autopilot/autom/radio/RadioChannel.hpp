/*
 * RadioChannel.hpp
 *
 *  Created on: 30 janv. 2014
 *      Author: Robotique
 */

#ifndef RADIOCHANNEL_HPP_
#define RADIOCHANNEL_HPP_

#include <hw/pwm/Pwm.hpp>

#define RC_CHANNEL_ROLL 	0
#define RC_CHANNEL_PITCH 	1
#define RC_CHANNEL_THRUST 	2
#define RC_CHANNEL_YAW 		3
#define RC_CHANNEL_EXTRA0 	4
#define RC_CHANNEL_EXTRA1 	5
#define RC_CHANNEL_EXTRA2 	6
#define RC_CHANNEL_EXTRA3 	7

namespace autom {

class RadioChannel {
public:
	typedef struct
	{
		int16_t min;
		int16_t trim;
		int16_t max;
	} Param;
public:
	RadioChannel(
			/* Inputs */
			const uint16_t& pwmRawVal,
			/* Parameters */
			const Param& param
			);
	virtual ~RadioChannel();

	/** @brief Read the channel value ensuring null value when pwm equals trim
	 * and limited to min / max values*/
	void readChannel(int16_t& signedPwmVal);

	/** @brief Get min value computed as trim-min*/
	void getMin(int16_t& min);

	/** @brief Get max value computed as trim+max*/
	void getMax(int16_t& max);

protected:
	/** @brief Pwm raw val */
	const uint16_t& _pwmVal;

	/** @brief Parameter */
	const Param& _param;
};

} /* namespace autom */

#endif /* RADIOCHANNEL_HPP_ */
