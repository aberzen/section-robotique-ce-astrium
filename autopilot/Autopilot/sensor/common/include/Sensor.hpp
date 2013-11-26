/*
 * Sensor.hpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#ifndef SENSOR_HPP_
#define SENSOR_HPP_

#include <arch/include/Process.hpp>

namespace sensor {

class Sensor : public arch::Process {
public:
	Sensor();
	virtual ~Sensor();

	/** @brief Is data available */
	inline bool isAvailable() ;

	/** @brief Init the process */
	virtual status initialize();

	/** @brief Execute the process */
	virtual status execute() = 0;

protected:
	/** @brief Flag to know if data is received from IMU */
	bool _isAvailable;

};

/** @brief Is data available */
inline bool Sensor::isAvailable()
{
	return _isAvailable;
}

} /* namespace sensor */
#endif /* SENSOR_HPP_ */
