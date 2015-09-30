/*
 * NavigationGuidanceManager.hpp
 *
 *  Created on: 7 sept. 2015
 *      Author: AdministrateurLocal
 */

#ifndef NAVIGATION_GUID_NAVIGATIONGUIDANCEMANAGER_HPP_
#define NAVIGATION_GUID_NAVIGATIONGUIDANCEMANAGER_HPP_

namespace navigation {

class NavigationGuidanceManager {
public:
	typedef enum {
		E_NAV_GUID_MODE_NONE = 0,
		E_NAV_GUID_MODE_LOITER,
		E_NAV_GUID_MODE_WAYPOINT
	} Mode;

public:
	NavigationGuidanceManager();
	virtual ~NavigationGuidanceManager();

	/** @brief Set mode */
	bool setMode(Mode mode);

	/** @brief Get mode */
	inline Mode getMode();

	/** @brief Execute attitude guidance mode manager */
	void execute();

protected:

	/** @brief Switch to Loiter */
	bool switchToLoiter();

	/** @brief Switch to Waypoint */
	bool switchToWaypoint();

	/** @brief Execute Loiter */
	void executeLoiter();

	/** @brief Execute Waypoint */
	void executeWaypoint();

protected:

	/** @brief Attitude guidance mode */
	Mode _mode;

};

/** @brief Get mode */
NavigationGuidanceManager::Mode NavigationGuidanceManager::getMode()
{
	return _mode;
}

} /* namespace navigation */

#endif /* NAVIGATION_GUID_NAVIGATIONGUIDANCEMANAGER_HPP_ */
