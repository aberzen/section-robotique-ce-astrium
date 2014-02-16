/*
 * RawSensorStream.cpp
 *
 *  Created on: 22 juil. 2013
 *      Author: Aberzen
 */

#include <gcs/include/Channel.hpp>
#include "../include/RawSensorStream.hpp"
#include <system/system/include/System.hpp>
#include <Arduino.h>

namespace mavlink {

RawSensorStream::RawSensorStream(mavlink_channel_t port,
		const board::Board::Measurements& meas,
		const board::Board::RawMeasurements& rawMeas) :
	DataStream(port),
	_meas(meas),
	_rawMeas(rawMeas),
	_rawImuAcc(0, 0, 0),
	_rawImuRate(0, 0, 0),
	_rawMag(0, 0, 0),
	_rawBaroPressure(0.),
	_rawBaroPressure0(0.),
	_rawBaroTemperature(0)

{
}

RawSensorStream::~RawSensorStream() {
}


/** @brief Sample the data */
void RawSensorStream::sampleData()
{
	/* Sample rate */
	_rawImuRate = _rawMeas.imu.gyroMeas_B;

	/* Sample acceleration */
	_rawImuAcc = _rawMeas.imu.accoMeas_B;

	/* Sample magnetic */
	_rawMag = _rawMeas.compass.magMeas_B;

	/* Sample baro pressure */
	_rawBaroPressure = _meas.baro.pressure;

	/* Sample baro pressure at ground zero */
	_rawBaroPressure0 = 0.;

	/* Sample baro temperature */
	_rawBaroTemperature = _meas.baro.temperature;
}

/** @brief Send data */
void RawSensorStream::sendData()
{

//	Serial.printf("_rawImuRate={%d,%d,%d}\n",
//			_rawImuRate.x,
//			_rawImuRate.y,
//			_rawImuRate.z);

//	Serial.printf("Raw: %ld\n",_date);
//	Serial.printf("Raw (avail): %d\n",_meas.imu.isAvailable);
//	return ;

	/* Send raw Imu data */
	mavlink_msg_raw_imu_send(
			(mavlink_channel_t) _port,
			(uint64_t) _date,
			(int16_t) (_rawImuAcc.x),
			(int16_t) (_rawImuAcc.y),
			(int16_t) (_rawImuAcc.z),
			(int16_t) (_rawImuRate.x),
			(int16_t) (_rawImuRate.y),
			(int16_t) (_rawImuRate.z),
			(int16_t) (_rawMag.x),
			(int16_t) (_rawMag.y),
			(int16_t) (_rawMag.z)
	);

	/* Send raw baro data */
	mavlink_msg_raw_pressure_send(
			(mavlink_channel_t) _port,
			(uint64_t) _date,
			(int16_t) (_rawBaroPressure*10.),
			(int16_t) 0,
			(int16_t) 0,
			(int16_t) _rawBaroTemperature);

//	Serial.print("- rawBaroPressure = "); Serial.print(_rawBaroPressure); Serial.println();
//	Serial.print("- rawBaroTemperature = "); Serial.print(_rawBaroTemperature); Serial.println();
//	Serial.printf("rateRaw=%d %d %d\n", _rawImuRate.x, _rawImuRate.y, _rawImuRate.z);
//	Serial.printf("accRaw=%d %d %d\n", _rawImuAcc.x, _rawImuAcc.y, _rawImuAcc.z);
//	Serial.printf("magRaw=%d %d %d\n", _rawMag.x, _rawMag.y, _rawMag.z);
}

} /* namespace mavlink */
