/*
 * HalImu.hpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#ifndef HALIMU_HPP_
#define HALIMU_HPP_

#include <math/include/Vector3.hpp>
#include <hw/common/include/Driver.hpp>

namespace hw {

class HalImu : public hw::Driver {
public:
	typedef struct {
		/** @brief Is measurement available */
		bool isAvailable;

		/** @brief Gyrometer measurement (rad/s) */
		math::Vector3f gyroMeas_B;

		/** @brief Accelerometer measurement (m/s^2) */
		math::Vector3f accoMeas_B;

		/** @brief Temperature in cDeg  */
		int32_t temperature;

	} Output;

	typedef struct {
		/** @brief Gyrometer measurement (lsb) */
		math::Vector3i gyroMeas_B;

		/** @brief Accelerometer measurement (lsb) */
		math::Vector3i accoMeas_B;

		/** @brief Temperature measurement (lsb) */
		int16_t temperature;
	} RawOutput;

public:
	HalImu(Output& out, RawOutput& rawOut);
	virtual ~HalImu();

protected:

	/** @brief Raw measurement */
	RawOutput& _rawOut;

	/** @brief Measurement */
	Output& _out;
};

} /* namespace hw */
#endif /* HALIMU_HPP_ */
