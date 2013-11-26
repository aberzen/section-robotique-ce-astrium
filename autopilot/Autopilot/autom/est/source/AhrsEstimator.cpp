/*
 * AhrsEstimator.cpp
 *
 *  Created on: 31 juil. 2013
 *      Author: Aberzen
 */

#include <hw/serial/include/FastSerial.hpp>
#include "../include/AhrsEstimator.hpp"

namespace autom {

AhrsEstimator::AhrsEstimator(
		sensor::Imu& imu,
		sensor::Barometer& baro,
		sensor::Magnetometer& mag) :
		Estimator(),
		_isFirst(true),
		_imu(imu),
		_baro(baro),
		_mag(mag),
		_rateBias_B(0.,0.,0.)
{
}

AhrsEstimator::~AhrsEstimator() {
}

/** @brief Init the process */
status AhrsEstimator::initialize()
{
	_position_I(0., 0., 0.);
	_velocity_I(0., 0., 0.);
	_rate_B(0., 0., 0.);
	_q_BI(1.,0.,0.,0.);
	/* TODO: initialize estimator (i.e. launch initialization procedure?) */
	_isFirst = true;
	return 0;
}



/** @brief Execute the process */
status AhrsEstimator::execute()
{
	math::Quaternion qPred_BI;
	math::Vector3f accPred_I;
	math::Vector3f velPred_I;
	math::Vector3f posPred_I;
	const math::Vector3f rateMeas_B = _imu.readAngleRate() + _rateBias_B;
	const math::Vector3f& accMeas_B = _imu.readAcceleration();
	const math::Vector3f& magMeas_B = _mag.readMagField();

	if (_isFirst)
	{
//		Serial.printf("accMeas_B={%f,%f,%f}\n",accMeas_B.x,accMeas_B.y,accMeas_B.z);
//		Serial.printf("magMeas_B={%f,%f,%f}\n",magMeas_B.x,magMeas_B.y,magMeas_B.z);

		/* Assume first step is performed on ground */
		/* acceleration measures -g */
		math::Vector3f z_B = accMeas_B.normalized();
		/* North */
		math::Vector3f x_B = z_B % (magMeas_B.normalized() % z_B);
		x_B.normalize();
		/* Third, to have a direct normalized frame */
		math::Vector3f y_B = z_B % x_B;

//		Serial.printf("x_B={%f,%f,%f}\n",x_B.x,x_B.y,x_B.z);
//		Serial.printf("y_B={%f,%f,%f}\n",y_B.x,y_B.y,y_B.z);
//		Serial.printf("z_B={%f,%f,%f}\n",z_B.x,z_B.y,z_B.z);


		/* Build DCM */
		math::Matrix3f dcm_BI(x_B, y_B, z_B);

		/* Convert DCM to quaternion */
		_q_BI.from_dcm(dcm_BI);
		math::Quaternion::normalize(_q_BI);

		/* store mag dir in inertial frame */
		_magDir_I = _q_BI.rotateQVQconj(magMeas_B.normalized());

		_velocity_I(0.,0.,0.);
		_position_I(0.,0.,0.);
		_rate_B(0.,0.,0.);
		_rateBias_B(0.,0.,0.);
//		_rateBias_B = rateMeas_B * (-1.);

		_isFirst = false;
	}
	else
	{

		{ /* 1) Compute prediction */
			{ /* Attitude */
				/*
				 * Limited development around zero
				 * cos(theta/2) = 1 - theta^2 / 8
				 * sin(theta/2) = theta/2 - theta^3/48
				 */
				float rateNorm = rateMeas_B.norm();
				math::Vector3f rateDir_B = rateMeas_B / rateNorm;
				float angle = rateNorm * FSW_TASK_CTRL_PERIOD_SEC;
				float angleSqrt = angle*angle;
				math::Quaternion dQ(
						1. - angleSqrt / 8.,
						rateDir_B * (angle*(0.5 - angleSqrt / 48.))
				);

				/* Prediction is attitude updated from rate */
				qPred_BI = _q_BI * dQ;

	//			Serial.printf("rateMeas_B={%f,%f,%f}\n",rateMeas_B.x,rateMeas_B.y,rateMeas_B.z);
	//			Serial.printf("rateNorm=%f\n",rateNorm);
	//			Serial.printf("rateDir_B={%f,%f,%f}\n",rateDir_B.x,rateDir_B.y,rateDir_B.z);
	//			Serial.printf("angle=%f\n",angle);
	//			Serial.printf("dQ={%f,%f,%f,%f}\n",dQ.getScalar(),dQ.getVector().x,dQ.getVector().y,dQ.getVector().z);
	//			Serial.printf("q_BI={%f,%f,%f,%f}\n",_q_BI.getScalar(),_q_BI.getVector().x,_q_BI.getVector().y,_q_BI.getVector().z);
	//			Serial.printf("qPred_BI={%f,%f,%f,%f}\n",qPred_BI.getScalar(),qPred_BI.getVector().x,qPred_BI.getVector().y,qPred_BI.getVector().z);

			} /* Attitude */
			{ /* Velocity / position */
				math::Vector3f accMeas_I;
				accMeas_I = qPred_BI.rotateQVQconj(accMeas_B);

				/* Predict acceleration (add the gravity) */
				accPred_I = accMeas_I + g_I;

				/* Predict velocity () */
				velPred_I = _velocity_I + accPred_I * FSW_TASK_CTRL_PERIOD_SEC;

				/* Predict velocity () */
				posPred_I = _position_I + _velocity_I * FSW_TASK_CTRL_PERIOD_SEC + accPred_I * FSW_TASK_CTRL_PERIOD_SEC * FSW_TASK_CTRL_PERIOD_SEC;
			} /* Velocity / position */
		} /* 1) Compute prediction */

		{ /* 2) Compute innovation */
			// TODO implements innovations (require more sensors to be implemented)
			// GPS for vel / pos
			// magneto for attitude
//			math::Vector3f magDirPred_B = qPred_BI.rotateQconjVQ(_magDir_I);


			/* Compare direction */
//			math::Vector3f dirErr_B = magDirPred_B % magMeas_B.normalized();;
//			_rateBias_B -= dirErr_B * (0.00001 / 0.01);
//			float errAngle = dirErr_B.norm();
//			dirErr_B.normalize();
//			float errAngleSqrt = errAngle*errAngle;
//			math::Quaternion dQ(
//					1. - errAngleSqrt / 8.,
//					dirErr_B * (errAngle*(0.5 - errAngleSqrt / 48.))
//			);

			/* Correction of the attitude */
			//qPred_BI = (~dQ) * qPred_BI ;

			math::Matrix3f dcm_BI;
			{
				/* Assume first step is performed on ground */
				/* acceleration measures -g */
				math::Vector3f z_B = accMeas_B.normalized();
				/* North */
				math::Vector3f x_B = z_B % (magMeas_B.normalized() % z_B);
				x_B.normalize();
				/* Third, to have a direct normalized frame */
				math::Vector3f y_B = z_B % x_B;
				/* Build DCM */
				dcm_BI(x_B, y_B, z_B);
			}

			/* Convert DCM to quaternion */
			{
				math::Quaternion qMeas_BI;
				qMeas_BI.from_dcm(dcm_BI);
				math::Quaternion::normalize(qMeas_BI);
				math::Quaternion dQ = (~qPred_BI) * qMeas_BI;

				math::Vector3f dirErr_B = dQ.getVector();
				float errAngle = dirErr_B.norm();
				if (errAngle < 0.0872664) /* 5° */
				{
					_rateBias_B += dirErr_B * (0.00005/0.01);
					dirErr_B /= errAngle;
					errAngle *= 0.1;

					float errAngleSqrt = errAngle*errAngle;
					dQ(
							1. - errAngleSqrt / 8.,
							dirErr_B * (errAngle*(0.5 - errAngleSqrt / 48.))
					);
					qPred_BI = qPred_BI * dQ;
				}
				else
				{
//					_rateBias_B += dirErr_B * (0.00005/0.01);
					dirErr_B /= errAngle;
					errAngle *= 0.02;

					float errAngleSqrt = errAngle*errAngle;
					dQ(
							1. - errAngleSqrt / 8.,
							dirErr_B * (errAngle*(0.5 - errAngleSqrt / 48.))
					);
					qPred_BI = qPred_BI * dQ;
				}
			}
		} /* 2) Compute innovation */

		{ /* 3) Correct and compute estimation */
			// No innovation, no correction
			// TODO implement correction
			_velocity_I = velPred_I;
			_position_I = posPred_I;
	//		Serial.printf("qPred_BI={%f,%f,%f,%f}\n",qPred_BI.getScalar(),qPred_BI.getVector().x,qPred_BI.getVector().y,qPred_BI.getVector().z);
			math::Quaternion::normalize(qPred_BI);
	//		Serial.printf("qPredNorm_BI={%f,%f,%f,%f}\n",qPred_BI.getScalar(),qPred_BI.getVector().x,qPred_BI.getVector().y,qPred_BI.getVector().z);
			_q_BI = qPred_BI;
			_rate_B = rateMeas_B;
		} /* 3) Correct and compute estimation */
	}
	return 0;
}


} /* namespace autom */
