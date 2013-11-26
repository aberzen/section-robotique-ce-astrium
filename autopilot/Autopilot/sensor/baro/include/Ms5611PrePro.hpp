/*
 * Ms5611PrePro.hpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#ifndef MS5611PREPRO_HPP_
#define MS5611PREPRO_HPP_

#include <hw/baro/include/HalBarometerMs5611OverSpi.hpp>

namespace sensor {

class Ms5611PrePro {
public:
	Ms5611PrePro(hw::HalBarometerMs5611OverSpi& hal);
	virtual ~Ms5611PrePro();

	void preProcess(
			const uint32_t& rawPressure,
			const uint32_t& rawTemperature,
			float& pressure,
			int16_t& temperature);

protected:
	hw::HalBarometerMs5611OverSpi& _hal;
};

} /* namespace sensor */
#endif /* MS5611PREPRO_HPP_ */
