/*
 * ModeStabilized.cpp
 *
 *  Created on: 2 janv. 2014
 *      Author: Robotique
 */

#include <math.h>
#include <autom/mgt/include/ModeStabilized.hpp>
#include <autom/ctrl/include/AttitudeControlError.hpp>
#include <system/system/include/System.hpp>


#define MODE_STABILIZED_RC_ROLL 	0
#define MODE_STABILIZED_RC_PITCH 	1
#define MODE_STABILIZED_RC_THRUST 	2
#define MODE_STABILIZED_RC_YAWRATE 	3

namespace autom {


ModeStabilized::ModeStabilized(
		/* Parameters */
		const float& dt,
		const ::autom::ModeStabilized::Param& param,
		const GenericParam& paramGen
		)
: Process(),
  _dt(dt),
  _param(param),
  _paramGen(paramGen),
  _rotZ(1.,0.,0.,0.),
  _prevInvNormRotZ(1.),
  _prevInvNormGuid(1.),
  _angleRollPrev(0.),
  _anglePitchPrev(0.)
{
}

ModeStabilized::~ModeStabilized() {
}

/** @brief Init the process */
void ModeStabilized::initialize()
{
	initializeFromEstimations();
}

/** @brief Initialize internal state from estimations */
void ModeStabilized::initializeFromEstimations()
{
	/* Compute the current seat and yaw rotations */
	math::Vector3f z_I(0.,0.,1.);
	math::Vector3f z_B = system::System::system.ancs.estimations.attitude_IB.rotateQconjVQ(z_I);
	math::Vector3f cross = (z_I % z_B) * (-0.5);
	math::Quaternion qCorr(1-cross*cross, cross);
	qCorr.normalize(2, 1.);

	/* Save previous roll and pitch angle */
	_angleRollPrev = qCorr.vector.x*2.;
	_anglePitchPrev = qCorr.vector.y*2.;

	/* Save yaw rotation */
	_rotZ = system::System::system.ancs.estimations.attitude_IB * qCorr;
	_rotZ.vector.x = 0.;
	_rotZ.vector.y = 0.;
	_prevInvNormRotZ = _rotZ.normalize(2, 1.);
	_prevInvNormGuid = _prevInvNormRotZ;

	system::System::system.ancs.attDem_IB = system::System::system.ancs.estimations.attitude_IB;
	system::System::system.ancs.rateDem_B = system::System::system.ancs.estimations.rate_B;
}

/** @brief Execute the process */
void ModeStabilized::execute()
{
	if (system::System::system.ancs.smGroundContact.getState() == GroundContactState::E_STATE_FLYING)
	{
		/* Flying */
		executeFlying();
	}
	else
	{
		/* On ground or similar */
		executeFlying();
//		executeOnGround();
	}
}

/** @brief Execute the process when on ground */
void ModeStabilized::executeOnGround()
{
	float thrust, roll, pitch, yawRate;

	/* Get commands */
	computeCommandsFromRc(thrust, roll, pitch, yawRate);

	/* Compute demanded attitude to minimize control errors */
	initializeFromEstimations();

	/* Compute demanded force */
	computeDemandedForce(thrust);

	/* Update control errors and torque */
	system::System::system.ancs.rateCtrlError(0., 0., 0.);
	system::System::system.ancs.attCtrlError(0., 0., 0.);

	/* Reset filter */
	system::System::system.ancs.filtX.reset();
	system::System::system.ancs.filtY.reset();
	system::System::system.ancs.filtZ.reset();

	system::System::system.ancs.torque_B(0., 0., 0.);
}

/** @brief Execute the process when flying */
void ModeStabilized::executeFlying()
{
	float thrust, roll, pitch, yawRate;

	/* Get commands */
	computeCommandsFromRc(thrust, roll, pitch, yawRate);

	/* Compute demanded force */
	computeDemandedForce(thrust);

	if (thrust != 0)
	{
		/* Compute demanded attitude */
		computeDemandedAttGuid(roll, pitch, yawRate);

		/* Compute control errors */
		system::System::system.ancs.rateCtrlError = system::System::system.ancs.rateDem_B - system::System::system.ancs.estimations.rate_B;
		AttitudeControlError::compControlError(system::System::system.ancs.attDem_IB, system::System::system.ancs.estimations.attitude_IB, system::System::system.ancs.attCtrlError);

		/* Apply Controller */
		system::System::system.ancs.attCtrl.compCommand(
				system::System::system.ancs.attCtrlError,
				system::System::system.ancs.rateCtrlError,
				system::System::system.ancs.torquePid_B);

		/* Apply filter */
		system::System::system.ancs.torque_B.x = system::System::system.ancs.filtX.apply(system::System::system.ancs.torquePid_B.x);
		system::System::system.ancs.torque_B.y = system::System::system.ancs.filtY.apply(system::System::system.ancs.torquePid_B.y);
		system::System::system.ancs.torque_B.z = system::System::system.ancs.filtZ.apply(system::System::system.ancs.torquePid_B.z);
	}
	else
	{
		/* Compute demanded attitude to minimize control errors */
		initializeFromEstimations();

		/* Update control errors and torque */
		system::System::system.ancs.rateCtrlError(0., 0., 0.);
		system::System::system.ancs.attCtrlError(0., 0., 0.);

		/* Reset filter */
		system::System::system.ancs.filtX.reset();
		system::System::system.ancs.filtY.reset();
		system::System::system.ancs.filtZ.reset();

		system::System::system.ancs.torque_B(0., 0., 0.);

	}
}

/** @brief Compute demanded attitude */
void ModeStabilized::computeDemandedAttGuid(float& roll, float& pitch, float& yawRate)
{
	/* Compute seat quaternion */
	math::Quaternion dQSeat(1., roll*0.5, pitch*0.5, 0.);

	/* Compute yaw increment quaternion from rate */
	math::Quaternion dQYaw(1., 0., 0., yawRate*_dt*0.5);

	/* Compute Yaw rotation */
	_rotZ *= dQYaw;
	_prevInvNormRotZ = _rotZ.normalize(1,_prevInvNormRotZ);

	/* Compute commanded attitude */
	system::System::system.ancs.attDem_IB(_rotZ * dQSeat);
	_prevInvNormGuid = system::System::system.ancs.attDem_IB.normalize(1,_prevInvNormGuid);

	/* Update acceleration and rate */
	system::System::system.ancs.rateDem_B((roll-_angleRollPrev)/_dt, (pitch-_anglePitchPrev)/_dt, yawRate);
	_angleRollPrev = roll;
	_anglePitchPrev = pitch;
}


/** @brief Compute commands from RC */
void ModeStabilized::computeCommandsFromRc(float& thrust, float& roll, float& pitch, float& yawRate)
{
	float tmp;
	int16_t pwm, limit;
	RadioChannel** rc = &system::System::system.ancs.radioChannels[0];

	/* Compute roll and pitch from user inputs */
	rc[MODE_STABILIZED_RC_ROLL]->readChannel(pwm);
	tmp = (float) (pwm * _param.rollPwmScale);
	roll = ldexpf(tmp, _param.rollPwmScaleExp);

	rc[MODE_STABILIZED_RC_PITCH]->readChannel(pwm);
	tmp = (float) (pwm * _param.pitchPwmScale);
	pitch = ldexpf(tmp, _param.pitchPwmScaleExp);

	/* Compute yaw rate from user inputs */
	yawRate = 0.;
	if (system::System::system.ancs.smGroundContact.getState() == GroundContactState::E_STATE_FLYING)
	{
		rc[MODE_STABILIZED_RC_YAWRATE]->readChannel(pwm);
		if (tmp > _param.deadzone)
		{
			tmp -= _param.deadzone;
		}
		else if (tmp < -_param.deadzone)
		{
			tmp += _param.deadzone;
		}
		else
		{
			tmp = 0;
		}
		tmp = (float) (pwm * _param.yawRatePwmScale);
		yawRate = ldexpf(tmp, _param.yawRatePwmScaleExp);
	}

	/* Compute thrust from user inputs */
	rc[MODE_STABILIZED_RC_THRUST]->readChannel(pwm);
	rc[MODE_STABILIZED_RC_THRUST]->getMin(limit);
	pwm = math_max(0, pwm-limit-_param.deadzone);
	tmp = (float) (pwm * _param.thrustPwmScale);
	thrust = ldexpf(tmp, _param.thrustPwmScaleExp);
}

/** @brief Compute demanded force*/
void ModeStabilized::computeDemandedForce(float& thrust)
{
	/* Compute demanded force */
	float forceNorm = _paramGen.mass*thrust;
	system::System::system.ancs.force_B(_param.thrustDir_B_x * forceNorm, _param.thrustDir_B_y * forceNorm, _param.thrustDir_B_z * forceNorm);
}

} /* namespace autom */
