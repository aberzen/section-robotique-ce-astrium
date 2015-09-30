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

	typedef struct {
		AttitudeController::Parameter ctrl;
	} Parameter;

public:
	AttitudeManager(const Parameter& param);
	virtual ~AttitudeManager();

	/** @brief periodic execution */
	void execute();

	/** @brief Getter method for Guidance Manager */
	inline AttitudeGuidanceManager& getGuidanceManager();

	/** @brief Getter method for Controller */
	inline AttitudeController& getController();

protected:

	/** @brief Guidance manager */
	AttitudeGuidanceManager _guidMgt;

	/** @brief Controller manager */
	AttitudeController _ctrl;

};

/** @brief Getter method for Guidance Manager */
AttitudeGuidanceManager& AttitudeManager::getGuidanceManager()
{
	return _guidMgt;
}

/** @brief Getter method for Controller */
AttitudeController& AttitudeManager::getController()
{
	return _ctrl;
}


} /* namespace attitude */

#endif /* ATT_MGT_ATTITUDEMANAGER_HPP_ */
