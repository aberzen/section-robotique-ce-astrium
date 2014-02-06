/*
 * GlobalState.hpp
 *
 *  Created on: 3 févr. 2014
 *      Author: Robotique
 */

#ifndef GLOBALSTATE_HPP_
#define GLOBALSTATE_HPP_

#include <infra/app/include/Process.hpp>

namespace autom {

class GlobalState : public infra::Process {
public:
	typedef enum
	{
		E_STATE_OFF = 0,
		E_STATE_INITIALIZING_IMU,
		E_STATE_INITIALIZING_COMPASS,
		E_STATE_READY
	} State;
public:
	GlobalState();
	virtual ~GlobalState();

	/** @brief Getter method for current state */
	inline State getState();

	/** @brief Init the process */
	virtual void initialize();

	/** @brief Execute the process */
	virtual void execute();

protected:

	/** @brief Process the off state */
	void processOff();

	/** @brief Process the off state */
	void processInitImu();

	/** @brief Process the off state */
	void processInitCompass();

	/** @brief Process the off state */
	void processReady();

protected:
	/** @brief */
	State _state;
};

/** @brief Getter method for current state */
inline GlobalState::State GlobalState::getState()
{
	return _state;
}

} /* namespace autom */

#endif /* GLOBALSTATE_HPP_ */
