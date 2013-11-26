/*
 * Magnetometer.hpp
 *
 *  Created on: 7 juil. 2013
 *      Author: Aberzen
 */

#ifndef MAGNETOMETER_HPP_
#define MAGNETOMETER_HPP_

#include <sensor/common/include/Sensor.hpp>
#include <hw/mag/include/HalMagnetometer.hpp>
#include <math/include/Matrix3.hpp>

namespace sensor {

class Magnetometer : Sensor {
public:
	Magnetometer(hw::HalMagnetometer& hal,
			math::Matrix3f& mat_UB,
			math::Vector3f& bias_B);
	virtual ~Magnetometer() ;


	/** @brief Init the process */
	virtual status initialize();

	/** @brief Execute the process */
	virtual status execute();

	/** @brief Read magnetic field */
	inline const math::Vector3f& readMagField();

protected:
	/** @brief Hal */
	hw::HalMagnetometer& _hal;

	/** @brief Mag field measurement (Gauss) */
	math::Vector3f _magFieldMeas;

	/** @brief rotate from unit to body frame */
	const math::Matrix3f& _mat_UB;

	/** @brief Pre-compensate bias in body frame */
	const math::Vector3f& _bias_B;
};

/** @brief Read magnetic field */
inline const math::Vector3f& Magnetometer::readMagField()
{
	return _magFieldMeas;
}

} /* namespace sensor */
#endif /* MAGNETOMETER_HPP_ */
