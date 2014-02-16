/*
 * ModeManagement.hpp
 *
 *  Created on: 17 janv. 2013
 *      Author: Aberzen
 */

#ifndef MODEMANAGEMENT_HPP_
#define MODEMANAGEMENT_HPP_

#include <infra/app/include/Process.hpp>
#include <autom/mgt/include/ModeStabilized.hpp>

namespace autom {

class ModeManagement : public infra::Process {
public:
	typedef enum {
		E_MODE_STABILIZED = 0,
		E_MODE_ACCRO,
		E_MODE_ALT_HOLD,
		E_MODE_POS_HOLD,
		E_MODE_TAKE_OFF,
		E_MODE_LANDING,
		E_MODE_INERT_VELOCITY,
		E_MODE_INERT_POSITION,
		E_MODE_WP,
		E_MODE_NB
	} T_MODE;

	typedef struct
	{
		ModeStabilized::Param modeStabilized;

	} Param;
public:
	ModeManagement(
			/* Parameters */
			const float& dt,
			const Param& param,
			const GenericParam& paramGen
			);
	virtual ~ModeManagement();

	/** @brief Getter method for current mode */
	inline T_MODE getCurrMode();

	/** @brief Change the current mode */
	inline void changeMode(T_MODE newMode);

	/** @brief Init the process */
	virtual void initialize();

	/** @brief Execute the process */
	virtual void execute();

protected:

	/** @brief Execute current mode*/
	virtual void executeMode();

protected:
	T_MODE _currMode;

	/** @brief Mode auto-stabilized */
	ModeStabilized _modeStabilitized;
};

/** @brief Getter method for current mode */
inline ModeManagement::T_MODE ModeManagement::getCurrMode(){
	return _currMode;
}

/** @brief Change the current mode */
inline void ModeManagement::changeMode(ModeManagement::T_MODE newMode) {
	_currMode = newMode;
}


} /* namespace autom */
#endif /* MODEMANAGEMENT_HPP_ */
