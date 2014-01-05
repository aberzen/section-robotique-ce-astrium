/*
 * ModeStabilized.cpp
 *
 *  Created on: 2 janv. 2014
 *      Author: Robotique
 */

//#include <hw/serial/include/FastSerial.hpp>
#include <autom/mgt/include/ModeStabilized.hpp>

#define MODE_STABILIZED_RC_PITCH	0
#define MODE_STABILIZED_RC_YAWRATE	1
#define MODE_STABILIZED_RC_THRUST	2
#define MODE_STABILIZED_RC_ROLL		3

namespace autom {

ModeStabilized::ModeStabilized(
		/* Input */
		const Estimator::Estimations& est,
		const ::math::Vector3f& force_I,
		/* Outputs */
		AttGuid::Output& attCtrlIn,
		NavGuid::Output& navCtrlIn,
		::math::Vector3f& force_B,
		/* Parameters */
		const float& dt,
		const ::autom::ModeStabilized::Param& param,
		const ControllerPid3Axes::Param& paramAttCtrl,
		const ControllerPid3Axes::Param& paramNavCtrl,
		/* Dependencies */
		AttitudeController& attCtrl,
		NavigationController& navCtrl
		)
: Mode(est, force_I, attCtrlIn, navCtrlIn, force_B, paramAttCtrl, paramNavCtrl, attCtrl, navCtrl),
  _dt(dt),
  _param(param),
  _rotZ(1.,0.,0.,0.),
  _prevInvNormRotZ(1.),
  _prevInvNormGuid(1.),
  _angleRollPrev(0.),
  _anglePitchPrev(0.),
  _thrustPrev(0.)
{
}

ModeStabilized::~ModeStabilized() {
	// TODO Auto-generated destructor stub
}

/** @brief Init the process */
::infra::status ModeStabilized::initialize()
{
	::infra::status res = Mode::initialize();
	if (res < 0)
		return res;

//	Serial.printf("qEst_IB = {%.5f %.5f %.5f %.5f}\n", _est.attitude_IB.scalar, _est.attitude_IB.vector.x, _est.attitude_IB.vector.y, _est.attitude_IB.vector.z);

	/* Compute the current seat and yaw rotations */
	math::Vector3f z_I(0.,0.,1.);
	math::Vector3f z_B = _est.attitude_IB.rotateQconjVQ(z_I);
	math::Vector3f cross = (z_I % z_B) * (-0.5);
	math::Quaternion qCorr(1-cross*cross, cross);
	qCorr.normalize(2, 1.);

	/* Save previous roll and pitch angle */
	_angleRollPrev = qCorr.vector.x*2.;
	_anglePitchPrev = qCorr.vector.y*2.;

	/* Save yaw rotation */
	_rotZ = _est.attitude_IB * qCorr;
	_rotZ.vector.x = 0.;
	_rotZ.vector.y = 0.;
	_prevInvNormRotZ = _rotZ.normalize(2, 1.);
	_prevInvNormGuid = _prevInvNormRotZ;

	return 0;
}

/** @brief Execute the process */
::infra::status ModeStabilized::execute()
{
	float tmp;
	float angleRoll = 0.;
	float anglePitch = 0.;
	float rateYaw = 0.;
	float thrust = _thrustPrev;

	hw::Pwm::Output& rc = board::Board::board.radio;
	if (rc.isAvailable)
	{
		/* Compute roll and pitch from user inputs */
		tmp = (float) (((int16_t)rc.channels[MODE_STABILIZED_RC_ROLL])-((MAX_PULSEWIDTH+MIN_PULSEWIDTH) >> 1));
		angleRoll = ldexpf(tmp, _param.rollPwmScale);

		tmp = (float) (((int16_t)rc.channels[MODE_STABILIZED_RC_PITCH])-((MAX_PULSEWIDTH+MIN_PULSEWIDTH) >> 1));
		anglePitch = ldexpf(tmp, _param.pitchPwmScale);

		/* Compute yaw rate from user inputs */
		tmp = (float) (((int16_t)rc.channels[MODE_STABILIZED_RC_YAWRATE])-((MAX_PULSEWIDTH+MIN_PULSEWIDTH) >> 1));
		rateYaw = ldexpf(tmp, _param.yawRatePwmScale);

		/* Compute thrust from user inputs */
		tmp = (float) (((int16_t)rc.channels[MODE_STABILIZED_RC_THRUST])-MIN_PULSEWIDTH);
		thrust = ldexpf(tmp, _param.thrustPwmScale);
	}

	/* Compute seat quaternion */
	math::Quaternion dQSeat(1., angleRoll*0.5, anglePitch*0.5, 0.);
//	dQSeat.normalize(1,1.);
//	Serial.printf("dQSeat = {%.5f %.5f %.5f %.5f}\n", dQSeat.scalar, dQSeat.vector.x, dQSeat.vector.y, dQSeat.vector.z);

	/* Compute yaw increment quaternion from rate */
	math::Quaternion dQYaw(1., 0., 0., rateYaw*_dt*0.5);
//	dQYaw.normalize(1,1.);
//	Serial.printf("dQYaw = {%.5f %.5f %.5f %.5f}\n", dQYaw.scalar, dQYaw.vector.x, dQYaw.vector.y, dQYaw.vector.z);

	/* Compute Yaw rotation */
//	Serial.printf("_rotZ (prev) = {%.5f %.5f %.5f %.5f}\n", _rotZ.scalar, _rotZ.vector.x, _rotZ.vector.y, _rotZ.vector.z);
	_rotZ *= dQYaw;
//	Serial.printf("_rotZ = {%.5f %.5f %.5f %.5f}\n", _rotZ.scalar, _rotZ.vector.x, _rotZ.vector.y, _rotZ.vector.z);
//	Serial.printf("_prevInvNormRotZ (prev) = %.5f\n", _prevInvNormRotZ);
	_prevInvNormRotZ = _rotZ.normalize(1,_prevInvNormRotZ);
//	Serial.printf("_prevInvNormRotZ = %.5f\n", _prevInvNormRotZ);

	/* Compute commanded attitude */
	_attCtrlIn.qDem_IB = _rotZ * dQSeat;
//	Serial.printf("_prevInvNormGuid (prev) = %.5f\n", _prevInvNormGuid);
	_prevInvNormGuid = _attCtrlIn.qDem_IB.normalize(1,_prevInvNormGuid);
//	Serial.printf("_prevInvNormGuid = %.5f\n", _prevInvNormGuid);

	/* Update acceleration and rate */
	_attCtrlIn.angRateDem_B((angleRoll-_angleRollPrev)/_dt, (anglePitch-_anglePitchPrev)/_dt, rateYaw);
	_angleRollPrev = angleRoll;
	_anglePitchPrev = anglePitch;


//	Serial.printf("radio{%d} = [%d %d %d %d]\n", rc.isAvailable, rc.channels[0], rc.channels[1], rc.channels[2], rc.channels[3]);
//	Serial.printf("anglePitch = %.5f\n", anglePitch);
//	Serial.printf("angleRoll = %.5f\n", angleRoll);
//	Serial.printf("rateYaw = %.5f\n", rateYaw);
//	Serial.printf("qDem_IB = {%.5f %.5f %.5f %.5f}\n", _attCtrlIn.qDem_IB.scalar, _attCtrlIn.qDem_IB.vector.x, _attCtrlIn.qDem_IB.vector.y, _attCtrlIn.qDem_IB.vector.z);
//	Serial.printf("rateDem_B = {%.5f %.5f %.5f}\n", _attCtrlIn.angRateDem_B.x, _attCtrlIn.angRateDem_B.y, _attCtrlIn.angRateDem_B.z);


	/* Compute demanded force */
	_force_B = _param.thrustDir_B * (_param.mass*thrust);
	_thrustPrev = thrust;

	/* Execute attitude control */
	return _attCtrl.execute();
}

} /* namespace autom */
