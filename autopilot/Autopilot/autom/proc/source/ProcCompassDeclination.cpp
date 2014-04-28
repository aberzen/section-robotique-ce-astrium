/*
 * ProcCompassDeclination.cpp
 *
 *  Created on: 30 déc. 2013
 *      Author: Robotique
 */

#include <autom/proc/include/ProcCompassDeclination.hpp>

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
  _filtZ(_param.filt),
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
	_filtZ.reset();
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
			accoMeasDir_B /= accoMeasDir_B.norm();
			::math::Vector3f tmp = (accoMeasDir_B % (_meas.compass.magMeas_B % accoMeasDir_B));

			X[0] = tmp.norm();
			X[1] = X[0];
			_out.magDir_I.x = X[0];
			_filtX.reset(X, X);

			_out.magDir_I.y = 0.;

			X[0] = _meas.compass.magMeas_B * accoMeasDir_B;
			X[1] = X[0];
			_out.magDir_I.z = X[0];
			_filtZ.reset(X, X);

			_count = _param.nbMeas;
			_state = E_STATE_RUNNING;
		}
		break;
	case E_STATE_RUNNING:
		if (_meas.imu.isAvailable && _meas.compass.isAvailable)
		{
			/* Only consider when both measurements are available */
			::math::Vector3f accoMeasDir_B = (_meas.imu.accoMeas_B - _est.imuAccoBias_B);
			accoMeasDir_B /= accoMeasDir_B.norm();

			::math::Vector3f tmp = (accoMeasDir_B % (_meas.compass.magMeas_B % accoMeasDir_B));
			_out.magDir_I.x = _filtX.apply(tmp.norm());
			_out.magDir_I.z = _filtZ.apply(_meas.compass.magMeas_B * accoMeasDir_B);

			_out.invNrm = 1./_out.magDir_I.norm();
			_out.magDir_I *= _out.invNrm;

			if (_count != 0)
			{
				_count --;
			}
			else
			{
				/* Normalize */
				math::Matrix3f dcm_IB;
				dcm_IB.a = tmp / tmp.norm();
				dcm_IB.c = accoMeasDir_B;
				dcm_IB.b = dcm_IB.c % dcm_IB.a;

				_est.attitude_IB.from_dcm(dcm_IB);

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
