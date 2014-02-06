/*
 * FlyingState.hpp
 *
 *  Created on: 3 févr. 2014
 *      Author: Robotique
 */

#ifndef FLYINGSTATE_HPP_
#define FLYINGSTATE_HPP_

#include <stdint.h>

namespace autom {

class FlyingState {
public:
	typedef enum
	{
		E_STATE_OFF = 0,
		E_STATE_DISARMED,
		E_STATE_ARMING,
		E_STATE_ARMED_AND_LANDED,
		E_STATE_ARMED_AND_FLYING,
		E_STATE_DISARMING
	} State;

	typedef struct
	{
		uint16_t armingTimer;
	} Param;
public:
	FlyingState(
			const Param& param
			);
	virtual ~FlyingState();

	/** @brief Getter method for state */
	inline State getState();

	/** @brief Start the state machine */
	virtual void start();

	/** @brief Sop the state machine */
	virtual void stop();

	/** @brief Reset the state machine */
	virtual void reset();

	/** @brief On tick */
	virtual void onTick();

protected:

	/** @brief Process disarmed state */
	void processDisarmed();

	/** @brief Process arming state */
	void processArming();

	/** @brief Process armed and landed state */
	void processArmedLanded();

	/** @brief Process armed and flying state */
	void processArmedFlying();

	/** @brief Process arming state */
	void processDisarming();

	/** @brief Check if throttles are in arming position */
	bool areThrottlesInArmingPosition();

	/** @brief Check if throttles are in disarming position */
	bool areThrottlesInDisarmingPosition();

protected:
	/** @brief Parameters */
	const Param& _param;

	/** @brief Current state */
	State _state;

	/** @brief Arming / Disarming timer */
	uint16_t _armingTimer;
};

/** @brief Getter method for state */
inline FlyingState::State FlyingState::getState()
{
	return _state;
}

} /* namespace autom */

#endif /* FLYINGSTATE_HPP_ */
