/*
 * GlobalState.hpp
 *
 *  Created on: 3 févr. 2014
 *      Author: Robotique
 */

#ifndef GLOBALSTATE_HPP_
#define GLOBALSTATE_HPP_

#include <infra/sched/include/FrameScheduling.hpp>

#define SCHED_NB_FRAME			 2
#define SCHED_FRAME0_NB_PROC	 9
#define SCHED_FRAME1_NB_PROC	 9

namespace autom {

class GlobalState : public infra::FrameScheduling {
public:
	typedef enum
	{
		E_STATE_OFF = 0,
		E_STATE_INITIALIZING_IMU,
		E_STATE_INITIALIZING_COMPASS,
		E_STATE_READY,
		E_STATE_FAILSAFE
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

	/** @brief Command failsafe */
	void cmdFailsafe();

protected:

	/** @brief Process the off state */
	void processOff();

	/** @brief Process the off state */
	void processInitImu();

	/** @brief Process the off state */
	void processInitCompass();

	/** @brief Process the off state */
	void processReady();

	/** @brief Process the failsafe state */
	void processFailsafe();

protected:
	/** @brief Current state */
	State _state;

	/** @brief Frames */
	infra::FrameScheduling::Frame _frames[SCHED_NB_FRAME];

	/** @brief Processes list for frame1 */
	infra::Process* _frame0Processes[SCHED_FRAME0_NB_PROC];

	/** @brief Processes list for frame2 */
	infra::Process* _frame1Processes[SCHED_FRAME1_NB_PROC];

};

/** @brief Getter method for current state */
inline GlobalState::State GlobalState::getState()
{
	return _state;
}

} /* namespace autom */

#endif /* GLOBALSTATE_HPP_ */
