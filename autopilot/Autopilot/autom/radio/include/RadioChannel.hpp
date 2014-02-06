/*
 * RadioChannel.hpp
 *
 *  Created on: 30 janv. 2014
 *      Author: Robotique
 */

#ifndef RADIOCHANNEL_HPP_
#define RADIOCHANNEL_HPP_

#include <hw/pwm/include/Pwm.hpp>

namespace autom {

class RadioChannel {
public:
	typedef struct
	{
		uint16_t min;
		int16_t trim;
		uint16_t max;
		float scale;
	} Param;
public:
	RadioChannel(
			/* Inputs */
			const uint16_t& pwmRawVal,
			/* Parameters */
			const Param& param
			);
	virtual ~RadioChannel();

	/** @brief Read the channel value ensuring null value when pwm equals zero,
	 * saturating pwm between min and max, and using scale to place pwm between [-1 and 1]*/
	void readChannel(float& val);

protected:
	/** @brief Pwm raw val */
	const uint16_t& _pwmVal;

	/** @brief Parameter */
	const Param& _param;
};

} /* namespace autom */

#endif /* RADIOCHANNEL_HPP_ */
