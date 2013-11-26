/*
 * State.cpp
 *
 *  Created on: 17 janv. 2013
 *      Author: Aberzen
 */

#include "../include/State.hpp"
#include "../include/StateMachine.hpp"

namespace arch {

State::State(Transition** transitions, uint8_t nbTransitions) :
	_transitions(transitions),
	_nbTransitions(nbTransitions)
	{
}

State::~State() {
}


/** @brief Get crossable transition */
Transition* State::getCrossableTransition()
{
	uint8_t iTrans;
	Transition* result = NULL;
	Transition* trans;
	for (iTrans=0 ; iTrans<_nbTransitions ; iTrans++)
	{
		trans = this->_transitions[iTrans];
		if (trans != NULL)
		{
			if (trans->isCrossable())
			{
				result = trans;
				break;
			}
		}
	}
	return result;
}

/** @brief Initialization method */
void State::initialize() {
	uint8_t iTrans;
	Transition* trans;

	for (iTrans=0 ; iTrans<_nbTransitions ; iTrans++)
	{
		trans = this->_transitions[iTrans];

		/* Initialize the transition */
		if (trans != NULL)
			trans->initialize();
	}
}


} /* namespace arch */
