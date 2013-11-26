/*
 * AhrsEstimator.hpp
 *
 *  Created on: 31 juil. 2013
 *      Author: Aberzen
 */

#ifndef AHRSESTIMATOR_HPP_
#define AHRSESTIMATOR_HPP_

#include "Estimator.hpp"
#include <sensor/imu/include/Imu.hpp>
#include <sensor/baro/include/Barometer.hpp>
#include <sensor/mag/include/Magnetometer.hpp>
#include <config/include/nrd.h>

namespace autom {

class AhrsEstimator : public Estimator  {
public:
	AhrsEstimator(
			sensor::Imu& imu,
			sensor::Barometer& baro,
			sensor::Magnetometer& _mag);
	virtual ~AhrsEstimator();

	/** @brief Init the process */
	virtual status initialize();

	/** @brief Execute the process */
	virtual status execute();

protected:
	bool _isFirst;

	/** @brief Imu sensor */
	sensor::Imu& _imu;

	/** @brief Baro sensor */
	sensor::Barometer& _baro;

	/** @brief Magnetometer */
	sensor::Magnetometer& _mag;

	/** @brief Estimation of IMU rate bias in body frame */
	math::Vector3f _rateBias_B;

	/** @brief Estimation of magnetic field in inertial frame */
	math::Vector3f _magDir_I;
};

} /* namespace autom */
#endif /* AHRSESTIMATOR_HPP_ */
