/*
 * Sonar.hpp
 *
 *  Created on: 7 juil. 2013
 *      Author: Aberzen
 */

#ifndef SONAR_HPP_
#define SONAR_HPP_

#include <sensor/common/include/Sensor.hpp>

namespace sensor {

class Sonar : Sensor {
public:
	Sonar();
	virtual ~Sonar() ;


	/** @brief Init the process */
	virtual status initialize();

	/** @brief Execute the process */
	virtual status execute();

};

} /* namespace sensor */
#endif /* SONAR_HPP_ */
