/*
 * EstimatorStream.cpp
 *
 *  Created on: 2 août 2013
 *      Author: Aberzen
 */

#include <infra/include/Task.hpp>
#include <hw/serial/include/FastSerial.hpp>
#include <gcs/include/Channel.hpp>
#include "../include/EstimatorStream.hpp"
#include <system/system/include/System.hpp>
#include <Arduino.h>

namespace mavlink {

EstimatorStream::EstimatorStream(mavlink_channel_t port, const autom::Estimator::Estimations& est) :
		DataStream(port),
		_est(est)
{
	// TODO Auto-generated constructor stub

}

EstimatorStream::~EstimatorStream() {
	// TODO Auto-generated constructor stub
}

/** @brief Sample the data */
void EstimatorStream::sampleData()
{
	/* Sample attitude */
	_q_BI = _est.attitude_IB;

	/* Sample rate */
	_rate_B = _est.rate_B;
}

/** @brief Send data */
void EstimatorStream::sendData()
{
//	Serial.printf("_q_BI={%f,%f,%f,%f}\n",
//			_q_BI.getScalar(),
//			_q_BI.getVector().x,
//			_q_BI.getVector().y,
//			_q_BI.getVector().z);
//
//	Serial.printf("_rate_B={%f,%f,%f}\n",
//			_rate_B.x,
//			_rate_B.y,
//			_rate_B.z);

#if 0
	mavlink_msg_attitude_quaternion_send(
			_port,
			_date,
			_q_BI.getScalar(),
			_q_BI.getVector().x,
			_q_BI.getVector().y,
			_q_BI.getVector().z,
			_rate_B.x,
			_rate_B.y,
			_rate_B.z);
#else
	float roll, pitch, yaw;
//	math::Matrix3f dcm;
	math::Quaternion qTmp_BI;
	if (qTmp_BI.scalar<0)
		qTmp_BI(-_q_BI.scalar, _q_BI.vector * (-1.));
	else
		qTmp_BI(_q_BI.scalar,_q_BI.vector);

	roll = atan2(
			2.*(qTmp_BI.scalar*qTmp_BI.vector.x + qTmp_BI.vector.y*qTmp_BI.vector.z),
			1.-2.*(qTmp_BI.vector.x*qTmp_BI.vector.x+qTmp_BI.vector.y*qTmp_BI.vector.y));
	pitch = -asin(2.*(qTmp_BI.scalar*qTmp_BI.vector.y - qTmp_BI.vector.x*qTmp_BI.vector.z));
	yaw = -atan2(
			2.*(qTmp_BI.scalar*qTmp_BI.vector.z + qTmp_BI.vector.x*qTmp_BI.vector.y),
			1.-2.*(qTmp_BI.vector.y*qTmp_BI.vector.y+qTmp_BI.vector.z*qTmp_BI.vector.z));

//	qTmp_BI.rotation_matrix(dcm);
//	_q_BI.to_dcm(dcm);
//	qTmp_BI.to_dcm(dcm);
//	dcm.to_euler(&roll, &pitch, &yaw);
//	yaw = -yaw;
//	roll = -roll;
//	pitch = -pitch;

//	Serial.printf("Est: %ld\n",millis());
//	Serial.printf("Est: %.5f %.5f %.5f %.5f\n",_est.attitude_IB.scalar, _est.attitude_IB.vector.x, _est.attitude_IB.vector.y, _est.attitude_IB.vector.z);
//	Serial.printf("Est: %.5f %.5f %.5f %.5f\n",_q_BI.scalar, _q_BI.vector.x, _q_BI.vector.y, _q_BI.vector.z);
//	return ;
//	_q_BI.to_euler(roll, pitch, yaw);
//	_q_BI.to_euler(roll, pitch, yaw);
	mavlink_msg_attitude_send(
			_port,
			millis(),
			roll,
			pitch,
			yaw,
			_rate_B.x,
			_rate_B.y,
			_rate_B.z
	);
#endif

}


} /* namespace mavlink */
