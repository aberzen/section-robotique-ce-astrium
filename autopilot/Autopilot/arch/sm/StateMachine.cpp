/*
 * StateMachine.cpp
 *
 *  Created on: 17 janv. 2013
 *      Author: Aberzen
 */

#include "../include/StateMachine.hpp"
#include "../include/State.hpp"

namespace arch {

StateMachine::StateMachine(State* initState) :
	_initState(initState),
	_currState(initState) {
	/* Reset */
	this->reset();
}

StateMachine::~StateMachine() {
}


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
status StateMachine::step(){
	Transition* transition;

	if (_currState == NULL)
	{
		return -1;
	}

	/* Execute current step */
	_currState->execute();

	/* Get crossable transition */
	transition = _currState->getCrossableTransition();
	if (transition != NULL)
	{
		/* Cross transition */
		transition->cross();
		_currState = transition->getNextState();
		/* Initialize new state */
		if (_currState != NULL)
			_currState->initialize();
	}
	return 0;
}

/** @brief Reset the state machine to its initial state */
status StateMachine::reset() {
	/* Update the state */
	_currState = _initState;

	/* Execute the state procedure */
	if (_currState !=NULL)
		_currState->initialize();

	return 0;
}

} /* namespace arch */
