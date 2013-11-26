/*
 * SensorDriver.hpp
 *
 *  Created on: 8 août 2013
 *      Author: Aberzen
 */

#ifndef SENSORDRIVER_HPP_
#define SENSORDRIVER_HPP_

#include "Driver.hpp"

namespace hw {

class SensorDriver: public hw::Driver {
public:
	SensorDriver();
	~SensorDriver();

	/** @brief Initialize the HW */
	virtual status initialize();

	/** @brief Reset the HW */
	virtual status reset();

	/** @brief Execute the driver */
	virtual status execute() = 0;

	/** @brief Is data available? */
	inline bool isAvailable();

protected:

	/** @brief Is data available? */
	bool _isAvailable;

};

/** @brief Is data available? */
inline bool SensorDriver::isAvailable()
{
	return _isAvailable;
}

} /* namespace hw */
#endif /* SENSORDRIVER_HPP_ */
