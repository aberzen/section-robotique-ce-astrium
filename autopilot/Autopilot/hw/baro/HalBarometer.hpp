/*
 * HalBarometer.hpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#ifndef HALBAROMETER_HPP_
#define HALBAROMETER_HPP_

#include <hw/common/Driver.hpp>
#include <stdint.h>

namespace hw {

class HalBarometer : public Driver {
public:
	HalBarometer();
	virtual ~HalBarometer();

	/** @brief Get measurement */
	virtual bool sample(
			int32_t& pressure,
			int16_t& temperature) = 0;
};

} /* namespace hw */
#endif /* HALBAROMETER_HPP_ */
