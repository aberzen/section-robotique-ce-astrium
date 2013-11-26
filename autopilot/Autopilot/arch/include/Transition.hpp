/*
 * Transition.hpp
 *
 *  Created on: 17 janv. 2013
 *      Author: Aberzen
 */

#ifndef TRANSITION_HPP_
#define TRANSITION_HPP_

#include <stddef.h>
#include "Procedure.hpp"


namespace arch {

class State;

class Transition {
public:
	Transition();
	virtual ~Transition();

	/** @brief Next state setter method */
	inline void setNextState(State* nextState);

	/** @brief Next state setter method */
	inline State* getNextState();

	/** @brief Verify if the transition is crossable */
	virtual bool isCrossable() = 0;

	/** @brief Initialize the transition when entering the sink state */
	virtual void initialize() = 0;

	/** @brief Cross the transition and return next state */
	virtual void cross() = 0;

private:
	/** @brief Next state */
	State* _nextState;
};

/** @brief Next state setter method */
inline void Transition::setNextState(State* nextState) {
	_nextState = nextState;
}

/** @brief Next state setter method */
inline State* Transition::getNextState() {
	return _nextState;
}



} /* namespace arch */
#endif /* TRANSITION_HPP_ */
