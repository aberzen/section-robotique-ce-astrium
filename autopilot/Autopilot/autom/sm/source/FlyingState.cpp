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
: Process(),
  _param(param),
  _state(E_STATE_DISARMED),
  _armingTimer(0),
  _safeTimer(0)
{
}

FlyingState::~FlyingState() {
}

/** @brief Initialize the state machine */
void FlyingState::initialize()
{
	stop();
}

/** @brief Start the state machine */
void FlyingState::start()
{
	/* Set to disarmed */
	disarm();
}

/** @brief Sop the state machine */
void FlyingState::stop()
{
	/* Set to disarmed */
	disarm();
}

/** @brief Reset the state machine */
void FlyingState::reset()
{
	/* Set to disarmed */
	disarm();
}

/** @brief Disarm the system */
void FlyingState::disarm()
{
	_armingTimer = 0;
	system::System::system.ancs.modulator.disarm();
	_state = E_STATE_DISARMED;
}

/** @brief Arm the system */
void FlyingState::arm()
{
	system::System::system.ancs.modulator.arm();
	_armingTimer = 0;
	_safeTimer = 0;
	_state = E_STATE_ARMED;
}

/** @brief On tick */
void FlyingState::execute()
{
	switch(_state)
	{
	case E_STATE_DISARMED:
		processDisarmed();
		break;
	case E_STATE_ARMED:
		processArmed();
		break;
	}
}

/** @brief Process disarmed state */
void FlyingState::processDisarmed()
{
	/* If thrust is minimal and yaw command on right, arming procedure started */
	if (areThrottlesInArmingPosition())
	{
		if (++_armingTimer >= _param.armingTimer)
		{
			arm();
		}
	}
	else
	{
		_armingTimer = 0;
	}
}

/** @brief Process armed state */
void FlyingState::processArmed()
{
	GroundContactState::State state = system::System::system.ancs.smGroundContact.getState();
	/* If thrust is minimal and yaw command on right, arming procedure started */
	switch(state)
	{
	case GroundContactState::E_STATE_ON_GROUND:
		if (areThrottlesInNeutralPosition())
		{
			if (++_safeTimer >= _param.safeTimer)
			{
				disarm();
			}
		}
		else
		{
			_safeTimer = 0;
		}
		if (areThrottlesInDisarmingPosition())
		{
			if (++_armingTimer >= _param.armingTimer)
			{
				disarm();
			}
		}
		else
		{
			_armingTimer = 0;
		}
		break;
	case GroundContactState::E_STATE_OFF:
	case GroundContactState::E_STATE_UNKNOWN:
	case GroundContactState::E_STATE_FLYING:
	default:
		_safeTimer = 0;
		_armingTimer = 0;
		break;
	}
}

bool FlyingState::areThrottlesInNeutralPosition()
{
	RadioChannel* chanRoll = system::System::system.ancs.radioChannels[RC_CHANNEL_ROLL];
	RadioChannel* chanPitch = system::System::system.ancs.radioChannels[RC_CHANNEL_PITCH];
	RadioChannel* chanThrust = system::System::system.ancs.radioChannels[RC_CHANNEL_THRUST];
	RadioChannel* chanYaw = system::System::system.ancs.radioChannels[RC_CHANNEL_YAW];

	int16_t pwm = 0, target = 0;

	/* Thrust */
	chanThrust->getMin(target);
	chanThrust->readChannel(pwm);

	if (pwm - target > _param.deadzone)
	{
		return false;
	}

	/* Yaw */
	chanYaw->readChannel(pwm);
	if (pwm > _param.deadzone)
	{
		return false;
	}

	/* Roll */
	chanRoll->readChannel(pwm);
	if (pwm > _param.deadzone)
	{
		return false;
	}

	/* Pitch */
	chanPitch->readChannel(pwm);
	if (pwm > _param.deadzone)
	{
		return false;
	}

	return true;
}

/** @brief Check if throttles are in arming position */
bool FlyingState::areThrottlesInArmingPosition()
{
	RadioChannel* chanYaw = system::System::system.ancs.radioChannels[RC_CHANNEL_YAW];
	RadioChannel* chanThrust = system::System::system.ancs.radioChannels[RC_CHANNEL_THRUST];

	int16_t pwm = 0, target = 0;

	/* Thrust */
	chanThrust->getMin(target);
	chanThrust->readChannel(pwm);

	if (pwm - target > _param.deadzone)
	{
		return false;
	}

	/* Yaw */
	chanYaw->getMax(target);
	chanYaw->readChannel(pwm);

	if (target - pwm > _param.deadzone)
	{
		return false;
	}

	return true;
}

/** @brief Check if throttles are in disarming position */
bool FlyingState::areThrottlesInDisarmingPosition()
{
	RadioChannel* chanYaw = system::System::system.ancs.radioChannels[RC_CHANNEL_YAW];
	RadioChannel* chanThrust = system::System::system.ancs.radioChannels[RC_CHANNEL_THRUST];

	int16_t pwm = 0, target = 0;

	/* Thrust */
	chanThrust->getMin(target);
	chanThrust->readChannel(pwm);

	if (pwm - target > _param.deadzone)
	{
		return false;
	}

	/* Yaw */
	chanYaw->getMin(target);
	chanYaw->readChannel(pwm);

	if (pwm - target > _param.deadzone)
	{
		return false;
	}

	return true;
}

} /* namespace autom */
