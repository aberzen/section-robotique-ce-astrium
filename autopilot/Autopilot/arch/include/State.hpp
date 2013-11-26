/*
 * State.hpp
 *
 *  Created on: 17 janv. 2013
 *      Author: Aberzen
 */

#ifndef STATE_HPP_
#define STATE_HPP_

#include <stdint.h>
#include <stddef.h>
#include "Procedure.hpp"
#include "Transition.hpp"

namespace arch {

class State {
public:
	State(Transition** transitions = NULL, uint8_t nbTransitions = 0);
	virtual ~State();

	/** @brief Initialize by executing the initialization procedure */
	virtual void initialize();

	/** @brief Execute the internal procedure */
	virtual void execute() = 0;

	/** @brief Get crossable transition */
	Transition* getCrossableTransition();

protected:

	/** @brief List of the transition from this state */
	Transition** _transitions;

	/** @brief Number of transition from this state */
	uint8_t _nbTransitions;

};

} /* namespace arch */
#endif /* STATE_HPP_ */
