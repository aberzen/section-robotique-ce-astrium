/*
 * FlyingState.hpp
 *
 *  Created on: 3 févr. 2014
 *      Author: Robotique
 */

#ifndef FLYINGSTATE_HPP_
#define FLYINGSTATE_HPP_

#include <stdint.h>
#include <infra/app/include/Process.hpp>

namespace autom {

class FlyingState : public infra::Process {
public:
	typedef enum
	{
		E_STATE_DISARMED = 0,
		E_STATE_ARMED
	} State;

	typedef struct
	{
		uint16_t armingTimer;
		uint16_t safeTimer;
		int16_t deadzone;
	} Param;
public:
	FlyingState(
			const Param& param
			);
	virtual ~FlyingState();

	/** @brief Initialize the state machine */
	virtual void initialize();

	/** @brief Getter method for state */
	inline State getState();

	/** @brief Start the state machine */
	virtual void start();

	/** @brief Sop the state machine */
	virtual void stop();

	/** @brief Reset the state machine */
	virtual void reset();

	/** @brief On tick */
	virtual void execute();

	/** @brief Disarm the system */
	virtual void disarm();

	/** @brief Arm the system */
	virtual void arm();

protected:

	/** @brief Process disarmed state */
	void processDisarmed();

	/** @brief Process armed state */
	void processArmed();

	/** @brief Check if throttles are in arming position */
	bool areThrottlesInArmingPosition();

	/** @brief Check if throttles are in disarming position */
	bool areThrottlesInDisarmingPosition();

	/** @brief Check if all commands are in neutral position */
	bool areThrottlesInNeutralPosition();

protected:
	/** @brief Parameters */
	const Param& _param;

	/** @brief Current state */
	State _state;

	/** @brief Arming / Disarming timer */
	uint16_t _armingTimer;

	/** @brief Safe timer ensure fall back in disarmed mode when on ground for a certain time */
	uint16_t _safeTimer;
};

/** @brief Getter method for state */
inline FlyingState::State FlyingState::getState()
{
	return _state;
}

} /* namespace autom */

#endif /* FLYINGSTATE_HPP_ */
