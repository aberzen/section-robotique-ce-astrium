/*
 * FlyingState.cpp
 *
 *  Created on: 3 févr. 2014
 *      Author: Robotique
 */

#include <autom/sm/include/FlyingState.hpp>
#include <system/system/include/System.hpp>

namespace autom {

FlyingState::FlyingState(
		const Param& param
		)
: _param(param),
  _state(E_STATE_OFF),
  _armingTimer(0)
{
}

FlyingState::~FlyingState() {
}

/** @brief Start the state machine */
void FlyingState::start()
{
	/* Set to disarmed */
	_state = E_STATE_DISARMED;
}

/** @brief Sop the state machine */
void FlyingState::stop()
{
	/* Set to off */
	_state = E_STATE_OFF;
}

/** @brief Reset the state machine */
void FlyingState::reset()
{
	stop();
	start();
}

/** @brief On tick */
void FlyingState::onTick()
{
	switch(_state)
	{
	case E_STATE_OFF:
		/* Nothing to do */
		break;
	case E_STATE_DISARMED:
		processDisarmed();
		break;
	case E_STATE_ARMING:
		processArming();
		break;
	case E_STATE_ARMED_AND_LANDED:
		processArmedLanded();
		break;
	case E_STATE_ARMED_AND_FLYING:
		processArmedFlying();
		break;
	case E_STATE_DISARMING:
		processDisarming();
		break;
	}
}

/** @brief Process disarmed state */
void FlyingState::processDisarmed()
{
	/* If thrust is minimal and yaw command on right, arming procedure started */
	if (areThrottlesInArmingPosition())
	{
		_armingTimer = _param.armingTimer;
		_state = E_STATE_ARMING;
	}
}

/** @brief Process arming state */
void FlyingState::processArming()
{
	/* If thrust not minimal or yaw command not on right, arming procedure cancelled */
	if (!areThrottlesInArmingPosition())
	{
		_state = E_STATE_DISARMED;
	}

	/* If timer duration run out, system armed */
	if ((++_armingTimer) >= _param.armingTimer)
	{
		system::System::system.ancs.modulator.arm();
		_state = E_STATE_ARMED_AND_LANDED;
	}
}

/** @brief Process armed and landed state */
void FlyingState::processArmedLanded()
{
	GroundContactState::State state = system::System::system.ancs.smGroundContact.getState();
	/* If thrust is minimal and yaw command on right, arming procedure started */
	if (state == GroundContactState::E_STATE_FLYING)
	{
		_state = E_STATE_ARMED_AND_FLYING;
	}

	if ((state == GroundContactState::E_STATE_ON_GROUND) && areThrottlesInDisarmingPosition())
	{
		_armingTimer = _param.armingTimer;
		_state = E_STATE_DISARMING;
	}
}

/** @brief Process armed and flying state */
void FlyingState::processArmedFlying()
{
	GroundContactState::State state = system::System::system.ancs.smGroundContact.getState();
	/* If thrust is minimal and yaw command on right, arming procedure started */
	if (state == GroundContactState::E_STATE_ON_GROUND)
	{
		_state = E_STATE_ARMED_AND_LANDED;
	}
}

/** @brief Process disarming state */
void FlyingState::processDisarming()
{
	/* If thrust not minimal or yaw command not on right, arming procedure cancelled */
	if (!areThrottlesInDisarmingPosition())
	{
		_state = E_STATE_ARMED_AND_LANDED;
	}

	/* If timer duration run out, system armed */
	if ((++_armingTimer) >= _param.armingTimer)
	{
		system::System::system.ancs.modulator.disarm();
		_state = E_STATE_DISARMED;
	}
}

/** @brief Check if throttles are in arming position */
bool FlyingState::areThrottlesInArmingPosition()
{
	// TODO to be implemented
	return false;
}

/** @brief Check if throttles are in disarming position */
bool FlyingState::areThrottlesInDisarmingPosition()
{
	// TODO to be implemented
	return true;
}



} /* namespace autom */
