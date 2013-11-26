/*
 * HalImu.hpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#ifndef HALIMU_HPP_
#define HALIMU_HPP_

#include <math/include/Vector3.hpp>
#include <hw/common/include/SensorDriver.hpp>

namespace hw {

class HalImu : public SensorDriver {
public:
	HalImu();
	virtual ~HalImu();

	/** @brief Imu Raw Gyrometer */
	inline const math::Vector3i& readRawRate() ;

	/** @brief Imu Raw Accelerometer */
	inline const math::Vector3i& readRawAcc() ;

	/** @brief Imu Raw Temperature */
	inline const int16_t& readRawTemperature() ;

	/** @brief Imu angular rate in unit frame (rad/s) */
	inline const math::Vector3f& readRate() ;

	/** @brief Imu acceleration in unit frame (m/s^2) */
	inline const math::Vector3f& readAcc() ;

	/** @brief Imu temperature (deg) */
	inline const float& readTemperature() ;

protected:
	/** @brief Gyrometer measurement (lsb) */
	math::Vector3i _rawRateMeas;

	/** @brief Accelerometer measurement (lsb) */
	math::Vector3i _rawAccMeas;

	/** @brief Temperature measurement (lsb) */
	int16_t _rawTempMeas;

	/** @brief Gyrometer measurement (rad/s) */
	math::Vector3f _rateMeas;

	/** @brief Accelerometer measurement (m/s^2) */
	math::Vector3f _accMeas;

	/** @brief Temperature measurement (deg) */
	float _tempMeas;
};

/** @brief Imu Raw Gyrometer */
inline const math::Vector3i& HalImu::readRawRate()
{
	return _rawRateMeas;
}

/** @brief Imu Raw Accelerometer */
inline const math::Vector3i& HalImu::readRawAcc()
{
	return _rawAccMeas;
}

/** @brief Imu angular rate in unit frame (rad/s) */
inline const math::Vector3f& HalImu::readRate()
{
	return _rateMeas;
}

/** @brief Imu acceleration in unit frame (m/s^2) */
inline const math::Vector3f& HalImu::readAcc()
{
	return _accMeas;
}

/** @brief Imu Raw Accelerometer */
inline const int16_t& HalImu::readRawTemperature()
{
	return _rawTempMeas;
}

/** @brief Imu Raw Accelerometer */
inline const float& HalImu::readTemperature()
{
	return _tempMeas;
}

} /* namespace hw */
#endif /* HALIMU_HPP_ */
