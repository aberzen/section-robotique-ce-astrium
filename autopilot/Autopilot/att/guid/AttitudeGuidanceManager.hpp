/*
 * AttitudeGuidanceManager.hpp
 *
 *  Created on: 27 août 2015
 *      Author: AdministrateurLocal
 */

#ifndef ATT_GUID_ATTITUDEGUIDANCEMANAGER_HPP_
#define ATT_GUID_ATTITUDEGUIDANCEMANAGER_HPP_

#include "AttitudeGuidanceAutoStab.hpp"
#include "AttitudeGuidanceAccro.hpp"

namespace attitude {

class AttitudeGuidanceManager {
public:
	typedef enum {
		E_MODE_NONE = 0,
		E_MODE_AUTOSTAB,
		E_MODE_AUTOSTAB_NOYAW,
		E_MODE_ACCRO
	} Mode;

public:
	AttitudeGuidanceManager();
	virtual ~AttitudeGuidanceManager();

	/** @brief Set mode */
	bool setMode(Mode mode);

	/** @brief Get mode */
	inline Mode getMode();

	/** @brief Execute attitude guidance mode manager */
	void execute();

protected:
	/** @brief Switch to Auto Stab */
	bool switchToAutoStab();

	/** @brief Switch to Accro */
	bool switchToAccro();

protected:

	/** @brief Attitude guidance mode */
	Mode _mode;

	/** @brief Auto Stab guidance */
	attitude::AttitudeGuidanceAutoStab _guidAutoStab;

	/** @brief Accro guidance */
	attitude::AttitudeGuidanceAccro _guidAccro;

};

AttitudeGuidanceManager::Mode AttitudeGuidanceManager::getMode()
{
	return _mode;
}


} /* namespace attitude */

#endif /* ATT_GUID_ATTITUDEGUIDANCEMANAGER_HPP_ */
