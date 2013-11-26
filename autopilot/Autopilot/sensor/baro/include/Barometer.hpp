/*
 * Barometer.hpp
 *
 *  Created on: 24 mars 2013
 *      Author: Aberzen
 */

#ifndef BAROMETER_HPP_
#define BAROMETER_HPP_

#include <sensor/common/include/Sensor.hpp>
#include <arch/include/Process.hpp>
#include <hw/baro/include/HalBarometer.hpp>

namespace sensor {

class Barometer : public Sensor {
public:
	Barometer(hw::HalBarometer& hal);
	virtual ~Barometer();

	/** @brief Read temperature value (in deci degree i.e.  deg/100) */
	inline const int16_t& readTemperature();

	/** @brief Read pressure value (in mBar) */
	inline const float& readPressure();

	/** @brief Get Hal */
	inline hw::HalBarometer& getHal() ;

protected:
	/** @brief Pressure in mBar */
	float _pressure;

	/** @brief Temperature in cDeg  */
	int16_t _temperature;

	/** @brief Hal interface  */
	hw::HalBarometer& _hal;
};

/** @brief Read temperature value (in deci degree i.e.  deg/100) */
inline const int16_t& Barometer::readTemperature()
{
	return _temperature;
}

/** @brief Read pressure value (in mBar) */
inline const float& Barometer::readPressure()
{
	return _pressure;
}

/** @brief Get Hal */
inline hw::HalBarometer& Barometer::getHal()
{
	return _hal;
}

} /* namespace sensor */
#endif /* BAROMETER_HPP_ */
