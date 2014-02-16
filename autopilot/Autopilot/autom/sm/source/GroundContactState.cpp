/*
 * GroundContactState.cpp
 *
 *  Created on: 10 janv. 2014
 *      Author: Robotique
 */

#include <autom/proc/include/GroundContactState.hpp>
#include <math/include/MathMacro.hpp>
#include <system/system/include/System.hpp>

namespace autom {

GroundContactState::GroundContactState(
		/* Input */
		/* Output */
		/* Param */
		const Param& param,
		const GenericParam& paramGen
		)
: Process(),
  _state(E_STATE_OFF),
  _count(0),
  _param(param),
  _paramGen(paramGen)
{

}

GroundContactState::~GroundContactState() {
}


/** @brief Init the process */
void GroundContactState::initialize()
{
	stop();
}

/** @brief Start the state machine */
void GroundContactState::start()
{
	_state = E_STATE_UNKNOWN;
}

/** @brief Sop the state machine */
void GroundContactState::stop()
{
	_state = E_STATE_OFF;
}

/** @brief Reset the state machine */
void GroundContactState::reset()
{
	stop();
	start();
}

/** @brief On tick */
void GroundContactState::execute()
{
	switch(_state)
	{
	case E_STATE_OFF:
		/* Nothing to do */
		break;
	case E_STATE_UNKNOWN:
		processUnknown();
		break;
	case E_STATE_ON_GROUND:
		processOnGround();
		break;
	case E_STATE_FLYING:
		processFlying();
		break;
	}
}


/** @brief Process unknown state */
void GroundContactState::processUnknown()
{
	float frcGrav_I_z = -9.81*_paramGen.mass;
	const math::Quaternion& attitude_IB = system::System::system.ancs.estimations.attitude_IB;
	const math::Vector3f& forceReal_B = system::System::system.ancs.forceReal_B;
	const math::Vector3f forceMeas_B = system::System::system.board.meas.imu.accoMeas_B * _paramGen.mass;

	math::Vector3f forceExtMeas_I = attitude_IB.rotateQVQconj(forceMeas_B - forceReal_B);

	/* Check if there is a contact with ground */
	if (   (math_abs(forceExtMeas_I.x)<_param.detectThdX)
		&& (math_abs(forceExtMeas_I.y)<_param.detectThdY)
		&& (math_abs(forceExtMeas_I.z + frcGrav_I_z)<_param.detectThdZ) )
	{
		_count = 0;
		_state = E_STATE_ON_GROUND;
	}
	/* Check if the contact with ground is lost */
	else if (  (math_abs(forceExtMeas_I.x)<_param.detectThdX)
			&& (math_abs(forceExtMeas_I.y)<_param.detectThdY)
			&& (math_abs(forceExtMeas_I.z)<_param.detectThdZ) )
	{
		_count = 0;
		_state = E_STATE_FLYING;
	}

}

/** @brief Process on ground state */
void GroundContactState::processOnGround()
{
	const math::Quaternion& attitude_IB = system::System::system.ancs.estimations.attitude_IB;
	const math::Vector3f& forceReal_B = system::System::system.ancs.forceReal_B;
	const math::Vector3f forceMeas_B = system::System::system.board.meas.imu.accoMeas_B * _paramGen.mass;

	if (system::System::system.board.meas.imu.isAvailable)
	{
		math::Vector3f forceExtMeas_I = attitude_IB.rotateQVQconj(forceMeas_B - forceReal_B);

		/* Check if the contact with ground is lost */
		if (   (math_abs(forceExtMeas_I.x)<_param.detectThdX)
			&& (math_abs(forceExtMeas_I.y)<_param.detectThdY)
			&& (math_abs(forceExtMeas_I.z)<_param.detectThdZ) )
		{
			if (++_count>=_param.filtDur)
			{
				_state = E_STATE_FLYING;
				_count = 0;
			}
		}
		else
		{
			_count = 0;
		}
	}
	else
	{
		_count = 0;
	}
}

/** @brief Process flying state */
void GroundContactState::processFlying()
{
	float frcGrav_I_z = -9.81*_paramGen.mass;
	const math::Quaternion& attitude_IB = system::System::system.ancs.estimations.attitude_IB;
	const math::Vector3f& forceReal_B = system::System::system.ancs.forceReal_B;
	const math::Vector3f forceMeas_B = system::System::system.board.meas.imu.accoMeas_B * _paramGen.mass;

	if (system::System::system.board.meas.imu.isAvailable)
	{

		math::Vector3f forceExtMeas_I = attitude_IB.rotateQVQconj(forceMeas_B - forceReal_B);

		/* Check if there is a contact with ground */
		if (   (math_abs(forceExtMeas_I.x)<_param.detectThdX)
			&& (math_abs(forceExtMeas_I.y)<_param.detectThdY)
			&& (math_abs(forceExtMeas_I.z + frcGrav_I_z)<_param.detectThdZ) )
		{
			if (++_count>=_param.filtDur)
			{
				_count = 0;
				_state = E_STATE_ON_GROUND;
			}
		}
		else
		{
			_count = 0;
		}
	}
	else
	{
		_count = 0;
	}
}

} /* namespace autom */
