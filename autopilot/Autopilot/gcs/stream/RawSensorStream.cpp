/*
 * RawSensorStream.cpp
 *
 *  Created on: 22 juil. 2013
 *      Author: Aberzen
 */

#include <gcs/include/Channel.hpp>
#include "../include/RawSensorStream.hpp"
#include <system/system/include/System.hpp>
#include <hw/serial/include/FastSerial.hpp>


namespace mavlink {

RawSensorStream::RawSensorStream(mavlink_channel_t port) :
	DataStream(port),
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
	/* Board */
	board::Board& board = system::System::system.getBoard();

	/* Sample rate */
	_rawImuRate = board.getHalImu().readRawRate();

	/* Sample acceleration */
	_rawImuAcc = board.getHalImu().readRawAcc();

	/* Sample magnetic */
	_rawMag = board.getHalMagnetometer().readRawMagField();

	/* Sample baro pressure */
	_rawBaroPressure = board.getBarometer().readPressure();

	/* Sample baro pressure at ground zero */
	_rawBaroPressure0 = 0.;

	/* Sample baro temperature */
	_rawBaroTemperature = board.getBarometer().readTemperature();
}

/** @brief Send data */
void RawSensorStream::sendData()
{

//	Serial.printf("_rawImuRate={%d,%d,%d}\n",
//			_rawImuRate.x,
//			_rawImuRate.y,
//			_rawImuRate.z);

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

//	Serial.print("RawSensorStream::sendData\n");
//	Serial.print("- rawBaroPressure = "); Serial.print(_rawBaroPressure); Serial.println();
//	Serial.print("- rawBaroTemperature = "); Serial.print(_rawBaroTemperature); Serial.println();
//	Serial.print("- rawImuRate = ");
//	Serial.print(_rawImuRate.x);Serial.print(" ");
//	Serial.print(_rawImuRate.y);Serial.print(" ");
//	Serial.print(_rawImuRate.z);Serial.println();
//	Serial.print("- rawImuAcc = ");
//	Serial.print(_rawImuAcc.x);Serial.print(" ");
//	Serial.print(_rawImuAcc.y);Serial.print(" ");
//	Serial.print(_rawImuAcc.z);Serial.println();
}

} /* namespace mavlink */
