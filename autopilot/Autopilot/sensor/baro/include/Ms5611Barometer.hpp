/*
 * Ms5611Barometer.hpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#ifndef MS5611BAROMETER_HPP_
#define MS5611BAROMETER_HPP_

#include "../include/Barometer.hpp"
#include "../include/Ms5611PrePro.hpp"

namespace sensor {

class Ms5611Barometer : public Barometer {
public:
	Ms5611Barometer(hw::HalBarometerMs5611OverSpi& hal);
	virtual ~Ms5611Barometer();

	/** @brief Get the pre-processing */
	inline Ms5611PrePro& getPrePro();

	/** @brief Init the process */
	virtual status initialize();

	/** @brief Execute the process */
	virtual status execute();

protected:
	Ms5611PrePro _prePro;
};

/** @brief Get the pre-processing */
inline Ms5611PrePro& Ms5611Barometer::getPrePro()
{
	return _prePro;
}
} /* namespace sensor */
#endif /* MS5611BAROMETER_HPP_ */
