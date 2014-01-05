/*
 * RawSensorStream.hpp
 *
 *  Created on: 22 juil. 2013
 *      Author: Aberzen
 */

#ifndef RAWSENSORSTREAM_HPP_
#define RAWSENSORSTREAM_HPP_

#include <gcs/include/DataStream.hpp>
#include <math/include/Vector3.hpp>
#include <board/gen/include/Board.hpp>

namespace mavlink {

class RawSensorStream : public DataStream{
public:
	RawSensorStream(mavlink_channel_t port,
			const board::Board::Measurements& meas,
	const board::Board::RawMeasurements& rawMeas);
	virtual ~RawSensorStream();

protected:
	/** @brief Sample the data */
	virtual void sampleData();

	/** @brief Send data */
	virtual void sendData();

protected:
	/** @brief Raw measurements */
	const board::Board::RawMeasurements& _rawMeas;

	/** @brief measurements */
	const board::Board::Measurements& _meas;

	/** @brief Raw acceleration */
	math::Vector3i _rawImuAcc;

	/** @brief Raw angular rate */
	math::Vector3i _rawImuRate;

	/** @brief Raw magnetic field */
	math::Vector3i _rawMag;

	/** @brief Raw atmospheric pressure */
	float _rawBaroPressure;

	/** @brief Raw atmospheric pressure at ground zero */
	float _rawBaroPressure0;

	/** @brief Raw atmospheric pressure at ground zero */
	int16_t _rawBaroTemperature;

};

} /* namespace mavlink */
#endif /* RAWSENSORSTREAM_HPP_ */
