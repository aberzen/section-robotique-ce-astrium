/*
 * Imu.cpp
 *
 *  Created on: 20 mars 2013
 *      Author: Aberzen
 */

#include "../include/Imu.hpp"

namespace sensor {

Imu::Imu(
		hw::HalImu& halImu,
		math::Matrix3f& rateMat_UB,
		math::Vector3f& rateBias_B,
		math::Matrix3f& accMat_UB,
		math::Vector3f& accBias_B) :
		Sensor(),
		_halImu(halImu),
		_ratePrePro(rateMat_UB, rateBias_B),
		_accPrePro(accMat_UB, accBias_B),
		_rate_B(0.,0.,0.),
		_acc_B(0.,0.,0.)
{
}

Imu::~Imu() {
}


/** @brief Init the process */
status Imu::initialize()
{
	/* Super */
	return Sensor::initialize();
}

/** @brief Execute the process */
status Imu::execute()
{
	// Check if available
	_isAvailable = _halImu.isAvailable();

	if (_isAvailable)
	{
		{ // RATE
			// First read the Hal / rate
			const math::Vector3f& rawRate_U = _halImu.readRate();

			// Pre-process
			_ratePrePro.preProcess(rawRate_U, _rate_B);
		}

		{ // ACC
			// First read the Hal / rate
			const math::Vector3f& rawAcc_U = _halImu.readAcc();

			// Pre-process
			_accPrePro.preProcess(rawAcc_U, _acc_B);
		}
	}
	return 0;
}


} /* namespace sensor */
