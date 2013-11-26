/*
 * HalBarometer.hpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#ifndef HALBAROMETER_HPP_
#define HALBAROMETER_HPP_

#include <hw/common/include/SensorDriver.hpp>

namespace hw {

class HalBarometer : public SensorDriver {
public:
	HalBarometer();
	virtual ~HalBarometer();

	/** @brief Raw pressure */
	inline uint32_t& readRawPressure() ;

	/** @brief Raw temperature */
	inline uint32_t& readRawTemperature() ;

protected:

	/** @brief Raw pressure */
	uint32_t _rawPressure;

	/** @brief Raw temperature */
	uint32_t _rawTemperature;

};

/** @brief Raw pressure */
inline uint32_t& HalBarometer::readRawPressure()
{
	return _rawPressure;
}

/** @brief Raw temperature */
inline uint32_t& HalBarometer::readRawTemperature()
{
	return _rawTemperature;
}

} /* namespace hw */
#endif /* HALBAROMETER_HPP_ */
