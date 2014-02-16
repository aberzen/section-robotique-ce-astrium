/*
 * GroundContactState.hpp
 *
 *  Created on: 10 janv. 2014
 *      Author: Robotique
 */

#ifndef GROUNDCONTACTSTATE_HPP_
#define GROUNDCONTACTSTATE_HPP_

#include <autom/gen/include/GenericParameters.hpp>
#include <infra/app/include/Process.hpp>

namespace autom {

class GroundContactState : public infra::Process {
public:
	typedef enum
	{
		E_STATE_OFF = 0,
		E_STATE_UNKNOWN,
		E_STATE_ON_GROUND,
		E_STATE_FLYING
	} State;
	typedef struct
	{
		float detectThdX;
		float detectThdY;
		float detectThdZ;
		uint8_t filtDur;
	} Param;
public:
	GroundContactState(
			/* Input */
			/* Output */
			/* Param */
			const Param& param,
			const GenericParam& paramGen
			);
	virtual ~GroundContactState();

	/** @brief Init the process */
	virtual void initialize() ;


	/** @brief Getter method for state */
	inline State getState();

	/** @brief Start the state machine */
	virtual void start();

	/** @brief Sop the state machine */
	virtual void stop();

	/** @brief Reset the state machine */
	virtual void reset();

	/** @brief Execute on tick */
	virtual void execute();


protected:

	/** @brief Process unknown state */
	void processUnknown();

	/** @brief Process on ground state */
	void processOnGround();

	/** @brief Process flying state */
	void processFlying();


protected:

	/** @brief Current state */
	State _state;

	/** @brief Cycle count for which the threshold was crossed */
	uint8_t _count;

	/** @brief Parameters */
	const Param& _param;

	/** @brief Generic parameters */
	const GenericParam& _paramGen;
};

/** @brief Getter method for state */
inline GroundContactState::State GroundContactState::getState()
{
	return _state;
}

} /* namespace autom */

#endif /* GROUNDCONTACTSTATE_HPP_ */
