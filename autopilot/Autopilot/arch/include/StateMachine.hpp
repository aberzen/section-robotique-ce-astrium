/*
 * StateMachine.hpp
 *
 *  Created on: 17 janv. 2013
 *      Author: Aberzen
 */

#ifndef STATEMACHINE_HPP_
#define STATEMACHINE_HPP_

#include <stddef.h>
#include "Procedure.hpp"
#include "State.hpp"

namespace arch {

class StateMachine : public Procedure {
public:
	StateMachine(State* initState);
	virtual ~StateMachine();

	/** @brief Getter method for internal current state */
	inline State* getCurrentState();

	/** @brief Step the state machine
	 *
	 * State machine step method consists in two main actions:
	 * 1) Execute procedure of the current state
	 * 2) Evaluate transition
	 *   a) If crossable transition exists:
	 *   		- process the attached procedure
	 *   		- execute state initialization procedure
	 *   b) Else do nothing
	 */
	status step();

	/** @brief Reset the state machine to its initial state */
	status reset();

protected:
	/** @brief Initial state */
	State* _initState;

	/** @brief Current state */
	State* _currState;

};

/** @brief Getter method for internal current state */
State* StateMachine::getCurrentState()
{
	return this->_currState;
}

} /* namespace arch */
#endif /* STATEMACHINE_HPP_ */
