/*
 * ProcCompassDeclination.cpp
 *
 *  Created on: 30 déc. 2013
 *      Author: Robotique
 */

#include <autom/proc/include/ProcCompassDeclination.hpp>
#include <hw/serial/include/FastSerial.hpp>

namespace autom {

ProcCompassDeclination::ProcCompassDeclination(
		const board::Board::Measurements& meas,
		autom::Estimator::Estimations& est,
		ProcCompassDeclination::Output& out,
		const ProcCompassDeclination::Param& param
		)
: _state(E_STATE_OFF),
  _meas(meas),
  _est(est),
  _out(out),
  _param(param),
  _filtX(_param.filt),
  _count(param.nbMeas)
{
}

ProcCompassDeclination::~ProcCompassDeclination()
{
}

/** @brief Start the procedure */
void ProcCompassDeclination::start()
{
	_state = E_STATE_INIT;
	_count = _param.nbMeas;
	_filtX.reset();
}

/** @brief Stop procedure */
void ProcCompassDeclination::stop()
{
	_state = E_STATE_OFF;
}

/** @brief Reset procedure */
void ProcCompassDeclination::reset()
{
	stop();
	start();
}


/** @brief Execute current procedure step */
void ProcCompassDeclination::onTick()
{
	switch (_state)
	{
	case E_STATE_INIT:
		if (_meas.imu.isAvailable && _meas.compass.isAvailable)
		{
			float X[2];
			::math::Vector3f accoMeasDir_B = (_meas.imu.accoMeas_B - _est.imuAccoBias_B);
//			Serial.printf("accoMeas_B=%.3f %.3f %.3f\n", _meas.imu.accoMeas_B.x, _meas.imu.accoMeas_B.y, _meas.imu.accoMeas_B.z);
//			Serial.printf("imuAccoBias_B=%.3f %.3f %.3f\n", _est.imuAccoBias_B, _est.imuAccoBias_B, _est.imuAccoBias_B);
//			Serial.printf("accoMeasDir_B=%.3f %.3f %.3f\n", accoMeasDir_B.x, accoMeasDir_B.y, accoMeasDir_B.z);
			accoMeasDir_B.normalize(3,1./9.81);
//			Serial.printf("accoMeasDir_B=%.3f %.3f %.3f\n", accoMeasDir_B.x, accoMeasDir_B.y, accoMeasDir_B.z);
			::math::Vector3f tmp = (accoMeasDir_B % (_meas.compass.magMeas_B % accoMeasDir_B));
//			Serial.printf("magMeas_B=%.3f %.3f %.3f\n", _meas.compass.magMeas_B.x, _meas.compass.magMeas_B.y, _meas.compass.magMeas_B.z);
//			Serial.printf("tmp=%.3f %.3f %.3f\n", tmp.x, tmp.y, tmp.z);
			X[0] = tmp.norm();
			X[1] = X[0];

			_filtX.reset(X, X);
			_count = _param.nbMeas;
			_state = E_STATE_RUNNING;
		}
		else
		{
			Serial.printf("No measurement\n");
		}
		break;
	case E_STATE_RUNNING:
		if (_meas.imu.isAvailable && _meas.compass.isAvailable)
		{
			float yaw = 0.;
			Serial.printf("filt=%.3f %.3f / %.3f %.3f\n",
					_param.filt.coeffNum[0],
					_param.filt.coeffNum[1],
					_param.filt.coeffDen[0],
					_param.filt.coeffDen[1]);

			/* Only consider when both measurements are available */
			::math::Vector3f accoMeasDir_B = (_meas.imu.accoMeas_B - _est.imuAccoBias_B);
//			Serial.printf("accoMeasDir_B=%.3f %.3f %.3f\n", accoMeasDir_B.x, accoMeasDir_B.y, accoMeasDir_B.z);
			accoMeasDir_B.normalize(3,1./9.81);
//			Serial.printf("accoMeasDir_B=%.3f %.3f %.3f\n", accoMeasDir_B.x, accoMeasDir_B.y, accoMeasDir_B.z);
			::math::Vector3f tmp = (accoMeasDir_B % (_meas.compass.magMeas_B % accoMeasDir_B));
//			Serial.printf("magMeas_B=%.3f %.3f %.3f\n", _meas.compass.magMeas_B.x, _meas.compass.magMeas_B.y, _meas.compass.magMeas_B.z);
//			Serial.printf("tmp=%.3f %.3f %.3f\n", tmp.x, tmp.y, tmp.z);
			_out.invNrm = 1./_filtX.apply(tmp.norm());
//			Serial.printf("invNrm=%.3f\n",_out.invNrm);
			tmp *= _out.invNrm;
			::math::Vector3f x_I(1.,0.,0.);
			yaw = atan2((tmp%x_I).norm(),tmp*x_I);
			Serial.printf("yaw=%.3f\n",yaw);
			if (_count != 0)
			{
				_count --;
			}
			else
			{
				/* Use yaw to initialize the attitude */
				::math::Vector3f z(0.,0.,1.);
				z *= sin(yaw*0.5);
				_est.attitude_IB(cos(yaw*0.5),z);
//				Serial.printf("q_IB=%.3f %.3f %.3f %.3f\n",
//						_est.attitude_IB.scalar,
//						_est.attitude_IB.vector.x,
//						_est.attitude_IB.vector.y,
//						_est.attitude_IB.vector.z
//					);

				/* Procedure is over */
				_state = E_STATE_ENDED;
			}
		}
		break;
	case E_STATE_ENDED:
	case E_STATE_OFF:
	default:
		break;
	}
}


} /* namespace autom */
