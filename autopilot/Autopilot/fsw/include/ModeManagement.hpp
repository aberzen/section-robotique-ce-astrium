/*
 * ModeManagement.hpp
 *
 *  Created on: 17 janv. 2013
 *      Author: Aberzen
 */

#ifndef MODEMANAGEMENT_HPP_
#define MODEMANAGEMENT_HPP_

#include <StateMachine.hpp>

namespace fsw {

class ModeManagement : public arch::StateMachine {
public:
	enum {
		E_MODE_ACCRO = 0,
		E_MODE_STABILIZED,
		E_MODE_ALT_HOLD,
		E_MODE_POS_HOLD,
		E_MODE_TAKE_OFF,
		E_MODE_LANDING,
		E_MODE_INERT_VELOCITY,
		E_MODE_INERT_POSITION,
		E_MODE_WP
	} T_MODE;

public:
	ModeManagement();
	virtual ~ModeManagement();

	/** @brief Getter method for current mode */
	inline T_MODE getCurrMode();

	/** @brief Change the current mode */
	inline void changeMode(T_MODE newMode);

protected:
	T_MODE _currMode;
};

/** @brief Getter method for current mode */
inline T_MODE ModeManagement::getCurrMode(){
	return _currMode;
}

/** @brief Change the current mode */
inline void ModeManagement::changeMode(T_MODE newMode) {
	_currMode = newMode;
}


} /* namespace fsw */
#endif /* MODEMANAGEMENT_HPP_ */
