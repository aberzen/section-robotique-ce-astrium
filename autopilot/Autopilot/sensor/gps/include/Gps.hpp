/*
 * Gps.hpp
 *
 *  Created on: 7 juil. 2013
 *      Author: Aberzen
 */

#ifndef GPS_HPP_
#define GPS_HPP_

#include <sensor/common/include/Sensor.hpp>

namespace sensor {

class Gps : Sensor {
public:
	Gps();
	virtual ~Gps();

	/** @brief Init the process */
	virtual status initialize();

	/** @brief Execute the process */
	virtual status execute();

};

} /* namespace sensor */
#endif /* GPS_HPP_ */
