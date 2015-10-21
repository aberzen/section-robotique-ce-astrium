/*
 * AttitudeManager.hpp
 *
 *  Created on: 31 août 2015
 *      Author: AdministrateurLocal
 */

#ifndef ATT_MGT_ATTITUDEMANAGER_HPP_
#define ATT_MGT_ATTITUDEMANAGER_HPP_

#include <att/guid/AttitudeGuidanceManager.hpp>
#include <att/ctrl/AttitudeController.hpp>

namespace attitude {

class AttitudeManager {
public:

	typedef enum
	{
		E_ATT_MODE_NONE = 0,
		E_ATT_MODE_STABNOYAW,
	} Mode;

	typedef struct {
		AttitudeController::Parameter ctrl;
	} Parameter;

public:
	AttitudeManager(const Parameter& param);
	virtual ~AttitudeManager();

	/** @brief periodic execution */
	void execute();

	/** @brief Set mode */
	bool setMode(Mode mode);

	/** @brief Get mode */
	inline Mode getMode();

protected:
	/** @brief Switch to auto stab */
	bool switchToAutoStab();

	/** @brief Switch to none */
	bool switchToNone();

protected:
	/** @brief Guidance manager */
	AttitudeGuidanceManager _guidMgt;

	/** @brief Controller manager */
	AttitudeController _ctrl;

	/** @brief Current mode */
	Mode _mode;
};

/** @brief Get current mode */
AttitudeManager::Mode AttitudeManager::getMode()
{
	return _mode;
}


} /* namespace attitude */

#endif /* ATT_MGT_ATTITUDEMANAGER_HPP_ */
