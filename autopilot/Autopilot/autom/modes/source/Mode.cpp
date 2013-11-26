/*
 * Mode.cpp
 *
 *  Created on: 10 juin 2013
 *      Author: Aberzen
 */

#include "../include/Mode.hpp"

namespace autom {

Mode::Mode(
		autom::AttGuid* attGuid,
		autom::AttCtrl* attCtrl,
		autom::TrajGuid* trajGuid,
		autom::TrajCtrl* trajCtrl,
		autom::Estimator* est,
		arch::Transition** transitions,
		uint8_t nbTransitions
		) :
		arch::State(transitions, nbTransitions),
		_attGuid(attGuid),
		_attCtrl(attCtrl),
		_trajGuid(trajGuid),
		_trajCtrl(trajCtrl),
		_est(est)
{
}


/** @brief Initialize by executing the initialization procedure */
void Mode::initialize()
{
	arch::State::initialize();
}

/** @brief Execute the internal procedure */
void Mode::execute()
{
	/* Process estimation function */
	_est->execute();
	/* Process attitude guidance function */
	_attGuid->execute();
	/* Process attitude control function */
	_attCtrl->execute();
	/* Process trajectory guidance function */
	_trajGuid->execute();
	/* Process trajectory control function */
	_trajCtrl->execute();
}


} /* namespace autom */
