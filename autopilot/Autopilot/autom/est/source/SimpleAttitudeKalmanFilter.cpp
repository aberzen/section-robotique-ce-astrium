/*
 * SimpleAttitudeKalmanFilter.cpp
 *
 *  Created on: 9 déc. 2013
 *      Author: Robotique
 */

#include <autom/est/include/SimpleAttitudeKalmanFilter.hpp>

namespace autom {


SimpleAttitudeKalmanFilter::SimpleAttitudeKalmanFilter(
		/* Inputs */
		const board::Board::Measurements& meas,
		/* Outputs */
		Estimator::Estimations& est,
		/* Parameters */
		const float& dt,
		const SimpleAttitudeKalmanFilter::Param& param)
: KalmanFilter(meas, est, dt),
  _param(param),
  _ratePred_B(0,0,0),
  _attitudePred_IB(1.,0.,0.,0.),
  _attitudeInnov_B(0.,0.,0.),
  _attEstInvNrmPrev(1.),
  _invNrmMagProjPrev(_param.declination.invNrm)
{
}

SimpleAttitudeKalmanFilter::~SimpleAttitudeKalmanFilter() {
}

/** @brief Init the process */
void SimpleAttitudeKalmanFilter::initialize()
{
	KalmanFilter::initialize();

	_attitudeInnov_B(0.,0.,0.);
	_attEstInvNrmPrev = 1.;
	_invNrmMagProjPrev = _param.declination.invNrm;
}

/** @brief Predict the state knowing the past */
infra::status SimpleAttitudeKalmanFilter::predict()
{
	if (_meas.imu.isAvailable)
	{
		/* Calc rate prediction */
		_ratePred_B = _meas.imu.gyroMeas_B - _est.imuGyroBias_B;

		math::Vector3f angInc = (_est.rate_B+_ratePred_B) * 0.25 * _dt;

		math::Quaternion dQ(
				1 - (angInc*angInc) * 0.5,
				angInc.x - angInc.x*angInc.x*angInc.x/6,
				angInc.y - angInc.y*angInc.y*angInc.y/6,
				angInc.z - angInc.z*angInc.z*angInc.z/6);
		_attitudePred_IB = _est.attitude_IB * dQ;
		_attEstInvNrmPrev = _attitudePred_IB.normalize(1,_attEstInvNrmPrev);

		math::Vector3f acc_B = _meas.imu.accoMeas_B - _est.imuAccoBias_B;
		math::Vector3f acc_I = _est.attitude_IB.rotateQVQconj(acc_B);
		acc_I.z -= 9.81;
		_velPred_I = _est.velocity_I + (acc_I * _dt);
		_posPred_I = _est.position_I + (_velPred_I * _dt);
	}
	return 0;
}

/** @brief Compute innovation */
infra::status SimpleAttitudeKalmanFilter::computeInnov()
{
	::math::Vector3f innovCompass_B(0., 0., 0.);
	::math::Vector3f innovAcco_B(0., 0., 0.);

	if (_meas.imu.isAvailable)
	{
		::math::Vector3f acc_B = _meas.imu.accoMeas_B - _est.imuAccoBias_B;
		if ((math_abs(acc_B.norm_squared() - 9.81*9.81)) < 10.)
		{
			acc_B.normalize(3,1./9.81);
			{
				::math::Vector3f z_I(0., 0., 1.);
				::math::Vector3f z_B = _attitudePred_IB.rotateQconjVQ(z_I);
				innovAcco_B = (acc_B % z_B) * (0.5 * _param.gainAcco);
			}
		    if (_meas.compass.isAvailable)
		    {
				::math::Vector3f magDirTh_B = _attitudePred_IB.rotateQconjVQ(_param.declination.magDir_I);
				::math::Vector3f magDir_B = _meas.compass.magMeas_B * _param.declination.invNrm;
				innovCompass_B = (magDir_B % magDirTh_B) * (0.5 * _param.gainCompass);
		    }
		}
	}

	_attitudeInnov_B = innovCompass_B + innovAcco_B;
	return 0;
}

/** @brief Update the state knowing */
infra::status SimpleAttitudeKalmanFilter::update()
{
	::math::Quaternion qCorr(1., _attitudeInnov_B);
	_est.attitude_IB = _attitudePred_IB*qCorr;
	_attEstInvNrmPrev = _est.attitude_IB.normalize(1,_attEstInvNrmPrev);
	_est.rate_B = _ratePred_B;

	_est.position_I = _posPred_I;
	_est.velocity_I = _velPred_I;

	_est.isConverged = true;
	return 0;
}

} /* namespace autom */
