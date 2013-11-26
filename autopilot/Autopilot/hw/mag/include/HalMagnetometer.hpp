/*
 * HalMagnetometer.hpp
 *
 *  Created on: 31 juil. 2013
 *      Author: Aberzen
 */

#ifndef HALMAGNETOMETER_HPP_
#define HALMAGNETOMETER_HPP_

#include <hw/common/include/SensorDriver.hpp>
#include <math/include/Vector3.hpp>

namespace hw {

class HalMagnetometer : public SensorDriver {
public:
	HalMagnetometer();
	virtual ~HalMagnetometer();

	/** @brief Read magnetic field (in lsb) */
	const math::Vector3i& readRawMagField();

	/** @brief Read magnetic field (in Gauss) */
	const math::Vector3f& readMagField();

protected:

	/** @brief Raw mag field measurement (lsb) */
	math::Vector3i _rawMagFieldMeas;

	/** @brief Mag field measurement (TBD) */
	math::Vector3f _magFieldMeas;
};

/** @brief Read magnetic field */
inline const math::Vector3i& HalMagnetometer::readRawMagField()
{
	return _rawMagFieldMeas;
}

/** @brief Read magnetic field */
inline const math::Vector3f& HalMagnetometer::readMagField()
{
	return _magFieldMeas;
}

} /* namespace hw */
#endif /* HALMAGNETOMETER_HPP_ */
