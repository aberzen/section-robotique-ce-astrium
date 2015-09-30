/*
 * NavigationGuidanceManager.cpp
 *
 *  Created on: 7 sept. 2015
 *      Author: AdministrateurLocal
 */

#include <nav/guid/NavigationGuidanceManager.hpp>

namespace navigation {

NavigationGuidanceManager::NavigationGuidanceManager() {
	// TODO Auto-generated constructor stub

}

NavigationGuidanceManager::~NavigationGuidanceManager() {
	// TODO Auto-generated destructor stub
}


/** @brief Set mode */
bool NavigationGuidanceManager::setMode(Mode mode)
{
	bool result = false;

	if (_mode == mode)
		return true;

	switch (mode)
	{
	case E_NAV_GUID_MODE_LOITER:
		// TODO
		break;
	case E_NAV_GUID_MODE_WAYPOINT:
		// TODO
		break;
	case E_NAV_GUID_MODE_NONE:
	default:
		break;
	}

	if (result)
		_mode = mode;

	return result;
}

/** @brief Execute attitude guidance mode manager */
void NavigationGuidanceManager::execute()
{
	switch (_mode)
	{
	case E_NAV_GUID_MODE_LOITER:
		executeLoiter();
		break;
	case E_NAV_GUID_MODE_WAYPOINT:
		executeWaypoint();
		break;
	case E_NAV_GUID_MODE_NONE:
	default:
		break;
	}
}


/** @brief Switch to Loiter */
bool NavigationGuidanceManager::switchToLoiter()
{
	return false;
}

/** @brief Switch to Waypoint */
bool NavigationGuidanceManager::switchToWaypoint()
{
	return false;
}

/** @brief Execute Loiter */
void NavigationGuidanceManager::executeLoiter()
{
}

/** @brief Execute Waypoint */
void NavigationGuidanceManager::executeWaypoint()
{
}


} /* namespace navigation */
