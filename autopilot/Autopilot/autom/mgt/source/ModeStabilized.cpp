/*
 * ModeStabilized.cpp
 *
 *  Created on: 2 janv. 2014
 *      Author: Robotique
 */

#include <hw/serial/include/FastSerial.hpp>
#include <autom/mgt/include/ModeStabilized.hpp>

#define MODE_STABILIZED_RC_ROLL		0
#define MODE_STABILIZED_RC_PITCH	1
#define MODE_STABILIZED_RC_THRUST	2
#define MODE_STABILIZED_RC_YAWRATE	3

namespace autom {


ModeStabilized::ModeStabilized(
		/* Input */
		const Estimator::Estimations& est,
		/* Outputs */
		::math::Vector3f& torque_B,
		::math::Vector3f& force_B,
		/* Parameters */
		const float& dt,
		const ::autom::ModeStabilized::Param& param,
		const GenericParam& paramGen
		)
: Mode(est, torque_B, force_B),
  _dt(dt),
  _param(param),
  _paramGen(paramGen),
  _attCtrl(_guidAtt, _est, _torque_B, _dt, _param.attCtrl),
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
	::infra::status res;
	/* Reinitialize the attitude controller */
//	_attCtrl.setParam(_param.attCtrl);
	res = _attCtrl.initialize();
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
	float angleRoll = this->_angleRollPrev;
	float anglePitch = this->_anglePitchPrev;
	float rateYaw = this->_guidAtt.angRateDem_B.z;
	float thrust = _thrustPrev;

	hw::Pwm::Output& rc = board::Board::board.radio;
	if (rc.isAvailable)
	{
		/* Compute roll and pitch from user inputs */
		tmp = (float) ((((int16_t)rc.channels[MODE_STABILIZED_RC_ROLL])-((MAX_PULSEWIDTH+MIN_PULSEWIDTH) >> 1)) * _param.rollPwmScale);
		angleRoll = ldexpf(tmp, _param.rollPwmScaleExp);

		tmp = (float) ((((int16_t)rc.channels[MODE_STABILIZED_RC_PITCH])-((MAX_PULSEWIDTH+MIN_PULSEWIDTH) >> 1)) * _param.pitchPwmScale);
		anglePitch = ldexpf(tmp, _param.pitchPwmScaleExp);

		/* Compute yaw rate from user inputs */
		tmp = (float) ((((int16_t)rc.channels[MODE_STABILIZED_RC_YAWRATE])-((MAX_PULSEWIDTH+MIN_PULSEWIDTH) >> 1)) * _param.yawRatePwmScale);
		if (math_abs(tmp) > 10)
			rateYaw = ldexpf(tmp, _param.yawRatePwmScaleExp);
		else
			rateYaw = 0.;

		/* Compute thrust from user inputs */
		tmp = (float) ((((int16_t)rc.channels[MODE_STABILIZED_RC_THRUST])-MIN_PULSEWIDTH - 300) * _param.thrustPwmScale);
		thrust = ldexpf(tmp, _param.thrustPwmScaleExp);


//		Serial.printf("radio{%d} = [%d %d %d %d]\n", rc.isAvailable, rc.channels[0], rc.channels[1], rc.channels[2], rc.channels[3]);
//		Serial.printf("anglePitch = %.5f\n", anglePitch);
//		Serial.printf("angleRoll = %.5f\n", angleRoll);
//		Serial.printf("rateYaw = %.5f\n", rateYaw);
//		Serial.printf("thrust= %.5f\n", thrust);
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
	_guidAtt.qDem_IB = _rotZ * dQSeat;
//	Serial.printf("_prevInvNormGuid (prev) = %.5f\n", _prevInvNormGuid);
	_prevInvNormGuid = _guidAtt.qDem_IB.normalize(1,_prevInvNormGuid);
//	Serial.printf("_prevInvNormGuid = %.5f\n", _prevInvNormGuid);

	/* Update acceleration and rate */
	_guidAtt.angRateDem_B((angleRoll-_angleRollPrev)/_dt, (anglePitch-_anglePitchPrev)/_dt, rateYaw);
	_angleRollPrev = angleRoll;
	_anglePitchPrev = anglePitch;


//	Serial.printf("radio{%d} = [%d %d %d %d]\n", rc.isAvailable, rc.channels[0], rc.channels[1], rc.channels[2], rc.channels[3]);
//	Serial.printf("anglePitch = %.5f\n", anglePitch);
//	Serial.printf("angleRoll = %.5f\n", angleRoll);
//	Serial.printf("rateYaw = %.5f\n", rateYaw);
//	Serial.printf("qDem_IB = {%.5f %.5f %.5f %.5f}\n", _guidAtt.qDem_IB.scalar, _guidAtt.qDem_IB.vector.x, _guidAtt.qDem_IB.vector.y, _guidAtt.qDem_IB.vector.z);
//	Serial.printf("rateDem_B = {%.5f %.5f %.5f}\n", _guidAtt.angRateDem_B.x, _guidAtt.angRateDem_B.y, _guidAtt.angRateDem_B.z);


	/* Compute demanded force */
	if (thrust < 0.)
	{
		thrust = 0.;
		_thrustPrev = thrust;
		this->_force_B(0.,0.,0.);
		this->_torque_B(0.,0.,0.);
		return 0;
	}

	tmp = _paramGen.mass*thrust;
	this->_force_B(_param.thrustDir_B_x * tmp, _param.thrustDir_B_y * tmp, _param.thrustDir_B_z * tmp);
	_thrustPrev = thrust;
//	Serial.printf("_force_B = %.5f %.5f %.5f\n", this->_force_B.x, this->_force_B.y, this->_force_B.z);
//	Serial.printf("_param.thrustPwm = %d %d\n", _param.thrustPwmScale, _param.thrustPwmScaleExp);
//	Serial.printf("_param.thrustDir_B = %.5f %.5f %.5f\n", this->_param.thrustDir_B_x, this->_param.thrustDir_B_y, this->_param.thrustDir_B_z);
//	Serial.printf("_param.mass = %.5f\n", _param.mass);

	/* Execute attitude control */
	return _attCtrl.execute();
}

} /* namespace autom */
