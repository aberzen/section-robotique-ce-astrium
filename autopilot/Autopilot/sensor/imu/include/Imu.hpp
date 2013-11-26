/*
 * Imu.hpp
 *
 *  Created on: 20 mars 2013
 *      Author: Aberzen
 */

#ifndef IMU_HPP_
#define IMU_HPP_

#include <sensor/common/include/Sensor.hpp>
#include <sensor/imu/include/ImuPrePro.hpp>
#include <hw/imu/include/HalImu.hpp>
#include <arch/include/Process.hpp>
#include <math/include/Vector3.hpp>
#include <math/include/Matrix3.hpp>

namespace sensor {

class Imu : public Sensor {
public:
	Imu(
			hw::HalImu& halImu,
			math::Matrix3f& rateMat_UB,
			math::Vector3f& rateBias_B,
			math::Matrix3f& accMat_UB,
			math::Vector3f& accBias_B);
	virtual ~Imu();


	/** @brief Init the process */
	virtual status initialize();

	/** @brief Execute the process */
	virtual status execute();

	/** @brief Imu Raw Gyrometer */
	inline math::Vector3f& readAngleRate() ;

	/** @brief Imu Raw Accelerometer */
	inline math::Vector3f& readAcceleration() ;

	/** @brief Get acceleration pre-processing */
	inline ImuPrePro& getAccPrePro() ;

	/** @brief Get angle rante pre-processing */
	inline ImuPrePro& getRatePrePro() ;

	/** @brief Get Hal */
	inline hw::HalImu& getHal() ;

protected:
	/** @brief Hal of the IMU sensor */
	hw::HalImu& _halImu;

	/** @brief IMU rate pre-processing */
	ImuPrePro _ratePrePro;

	/** @brief IMU acceleration pre-processing */
	ImuPrePro _accPrePro;

	/** @brief Gyrometer measurement */
	math::Vector3f _rate_B;

	/** @brief Accelerometer measurement */
	math::Vector3f _acc_B;
};

/** @brief Imu Raw Gyrometer */
inline math::Vector3f& Imu::readAngleRate()
{
	return _rate_B;
}

/** @brief Imu Raw Accelerometer */
inline math::Vector3f& Imu::readAcceleration()
{
	return _acc_B;
}

/** @brief Get acceleration pre-processing */
inline ImuPrePro& Imu::getAccPrePro()
{
	return _accPrePro;
}

/** @brief Get angle rante pre-processing */
inline ImuPrePro& Imu::getRatePrePro()
{
	return _ratePrePro;
}

/** @brief Get Hal */
inline hw::HalImu& Imu::getHal()
{
	return _halImu;
}

} /* namespace sensor */
#endif /* IMU_HPP_ */
