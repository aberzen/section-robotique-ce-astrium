/*
 * NavigationManager.hpp
 *
 *  Created on: 15 sept. 2015
 *      Author: AdministrateurLocal
 */

#ifndef NAV_MGT_NAVIGATIONMANAGER_HPP_
#define NAV_MGT_NAVIGATIONMANAGER_HPP_

#include <nav/guid/NavigationGuidanceManager.hpp>
#include <nav/ctrl/NavigationController.hpp>
#include <nav/mgt/NavigationDirectThrust.hpp>

namespace navigation {

class NavigationManager {
public:

	typedef enum {
		E_NAV_MODE_NONE = 0,
		E_NAV_MODE_DIRECTTHRUST,
		E_NAV_MODE_LOITER,
		E_NAV_MODE_WAYPOINT
	} Mode;

	typedef struct {
		NavigationController::Parameter paramCtrl;
		NavigationDirectThrust::Parameter paramDirectThrust;
	} Parameter ;

public:
	NavigationManager(const Parameter& param);
	virtual ~NavigationManager();

	/** @brief Get current mode */
	inline Mode getMode();

	/** @brief Set mode */
	bool setMode(Mode mode);

	/** @brief Execute the service */
	void execute();

	/** @brief Getter method for Guidance Manager */
	inline NavigationGuidanceManager& getGuidanceManager();

	/** @brief Getter method for Controller */
	inline NavigationController& getController();

protected:

	/** @brief Process direct thrust mode */
	void executeDirectThrustMode();

	/** @brief Process loiter mode */
	void executeAutoNavMode();

	/** @brief Switch to direct thrust */
	bool switchToModeDirectThrust();

	/** @brief Switch to loiter*/
	bool switchToModeLoiter();

	/** @brief Switch to way point */
	bool switchToModeWaypoint();

protected:
	/** @brief Current mode */
	Mode _mode;

	/** @brief Guidance manager */
	NavigationGuidanceManager _guidMgr;

	/** @brief Guidance manager */
	NavigationDirectThrust _guidDirectThrust;

	/** @brief Controller */
	NavigationController _ctrl;
};

/** @brief Get current mode */
NavigationManager::Mode NavigationManager::getMode()
{
	return _mode;
}

/** @brief Getter method for Guidance Manager */
NavigationGuidanceManager& NavigationManager::getGuidanceManager()
{
	return _guidMgr;
}

/** @brief Getter method for Controller */
NavigationController& NavigationManager::getController()
{
	return _ctrl;
}



} /* namespace navigation */

#endif /* NAV_MGT_NAVIGATIONMANAGER_HPP_ */
