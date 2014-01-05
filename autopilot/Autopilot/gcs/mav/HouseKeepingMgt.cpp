/*
 * HouseKeepingMgt.cpp
 *
 *  Created on: 7 juin 2013
 *      Author: Aberzen
 */

#include "../include/HouseKeepingMgt.hpp"

namespace mavlink {

HouseKeepingMgt::HouseKeepingMgt(mavlink_channel_t port,
		const autom::Estimator::Estimations& est,
		const board::Board::Measurements& meas,
		const board::Board::RawMeasurements& rawMeas) :
		_port(port),
		_rawSensorsStream(port, meas, rawMeas),
		_estStream(port, est)
{
}

HouseKeepingMgt::~HouseKeepingMgt() {
}

void HouseKeepingMgt::setDataStream(
		uint16_t req_message_rate,
		uint8_t req_stream_id,
		uint8_t start_stop
		)
{
	switch (req_stream_id)
	{
	case MAV_DATA_STREAM_ALL:
		/* Enable all data streams | */
		if (start_stop)
		{
			_rawSensorsStream.start(req_message_rate);
			_estStream.start(req_message_rate);
		}
		else
		{
			_rawSensorsStream.stop();
			_estStream.stop();
		}
		break;
	case MAV_DATA_STREAM_RAW_SENSORS:
		/* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets. | */
		if (start_stop)
		{
			_rawSensorsStream.start(req_message_rate);
		}
		else
		{
			_rawSensorsStream.stop();
		}
		break;
	case MAV_DATA_STREAM_EXTENDED_STATUS:
		/* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS | */
		break;
	case MAV_DATA_STREAM_RC_CHANNELS:
		/* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW | */
		break;
	case MAV_DATA_STREAM_RAW_CONTROLLER:
		/* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT. | */
		break;
	case MAV_DATA_STREAM_POSITION:
		/* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages. | */
		break;
	case MAV_DATA_STREAM_EXTRA1:
		/* Dependent on the autopilot | */
		if (start_stop)
		{
			_estStream.start(req_message_rate);
		}
		else
		{
			_estStream.stop();
		}
		break;
	case MAV_DATA_STREAM_EXTRA2:
		/* Dependent on the autopilot | */
		break;
	case MAV_DATA_STREAM_EXTRA3:
		/* Dependent on the autopilot | */
		break;
	}

}

//void HouseKeepingMgt::send_raw_imu1(mavlink_channel_t chan)
//{
//    Vector3f accel;
//    Vector3f gyro;
//    Vector3f mag;
//
//    mavlink_msg_raw_imu_send(
//        chan,
//        micros(),
//        accel.x,
//        accel.y,
//        accel.z,
//        gyro.x,
//        gyro.y,
//        gyro.z,
//        mag.x,
//        mag.y,
//        mag.z);
//}
//
//void HouseKeepingMgt::send_raw_imu2(mavlink_channel_t chan)
//{
//	float baroPress;
//	float baroPress0;
//	float baroTemp;
//
//    mavlink_msg_scaled_pressure_send(
//        chan,
//        millis(),
//        baroPress/,
//        baroPress - baroPress0,
//        (int16_t)(baroTemp*100.));
//}
//
//void HouseKeepingMgt::send_raw_imu3(mavlink_channel_t chan)
//{
//    Vector3f mag_offsets = compass.get_offsets();
//    Vector3f accel_offsets = ins.get_accel_offsets();
//    Vector3f gyro_offsets = ins.get_gyro_offsets();
//
//    mavlink_msg_sensor_offsets_send(chan,
//                                    mag_offsets.x,
//                                    mag_offsets.y,
//                                    mag_offsets.z,
//                                    compass.get_declination(),
//                                    barometer.get_raw_pressure(),
//                                    barometer.get_raw_temp(),
//                                    gyro_offsets.x,
//                                    gyro_offsets.y,
//                                    gyro_offsets.z,
//                                    accel_offsets.x,
//                                    accel_offsets.y,
//                                    accel_offsets.z);
//}
//
//void HouseKeepingMgt::send_gps_raw(mavlink_channel_t chan)
//{
//    uint8_t fix = g_gps->status();
//    if (fix == GPS::GPS_OK) {
//        fix = 3;
//    }
//
//    mavlink_msg_gps_raw_int_send(
//        chan,
//        g_gps->last_fix_time*(uint64_t)1000,
//        fix,
//        g_gps->latitude,      // in 1E7 degrees
//        g_gps->longitude,     // in 1E7 degrees
//        g_gps->altitude * 10, // in mm
//        g_gps->hdop,
//        65535,
//        g_gps->ground_speed,  // cm/s
//        g_gps->ground_course, // 1/100 degrees,
//        g_gps->num_sats);
//
//}


/** @brief Update periodic */
void HouseKeepingMgt::update()
{
	/* Update Raw sensors */
	_rawSensorsStream.update();
	/* Update Estimator */
	_estStream.update();
}

/** @brief Init the process */
infra::status HouseKeepingMgt::initialize()
{
	/* Initialize raw sensors */
	_rawSensorsStream.initialize();
	/* Update Estimator */
	_estStream.initialize();

	return 0;
}

/** @brief Execute the process */
infra::status HouseKeepingMgt::execute()
{
	/* Execute raw sensors data stream */
	_rawSensorsStream.execute();
	/* Update Estimator */
	_estStream.execute();

	return 0;
}

} /* namespace mavlink */
