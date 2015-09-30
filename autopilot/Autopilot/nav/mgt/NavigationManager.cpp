/*
 * NavigationManager.cpp
 *
 *  Created on: 15 sept. 2015
 *      Author: AdministrateurLocal
 */

#include <nav/mgt/NavigationManager.hpp>
#include <system/system/System.hpp>

namespace navigation {

NavigationManager::NavigationManager(const Parameter& param)
: _mode(E_NAV_MODE_NONE),
  _guidMgr(),
  _guidDirectThrust(param.paramDirectThrust),
  _ctrl(param.paramCtrl)
{
	// TODO Auto-generated constructor stub

}

NavigationManager::~NavigationManager() {
	// TODO Auto-generated destructor stub
}

/** @brief Execute the service */
void NavigationManager::execute()
{
	switch (_mode)
	{
	case E_NAV_MODE_DIRECTTHRUST:
		executeDirectThrustMode();
		break;
	case E_NAV_MODE_LOITER:
	case E_NAV_MODE_WAYPOINT:
		executeAutoNavMode();
		break;
	case E_NAV_MODE_NONE:
	default:
		/* Nothing to do */
		break;
	}
}

/** @brief Set mode */
bool NavigationManager::setMode(Mode mode)
{
	bool result = false;

	if (_mode == mode)
		return true;

	switch (mode)
	{
	case E_NAV_MODE_DIRECTTHRUST:
		result = switchToModeDirectThrust();
		break;
	case E_NAV_MODE_LOITER:
		result = switchToModeLoiter();
		break;
	case E_NAV_MODE_WAYPOINT:
		result = switchToModeWaypoint();
		break;
	case E_NAV_MODE_NONE:
	default:
		/* Nothing to do */
		break;
	}

	if (result)
		_mode = mode;

	return result;
}


/** @brief Process direct thrust mode */
void NavigationManager::executeDirectThrustMode()
{
	_guidDirectThrust.calcGuidance(system::system.getRadio());
}

/** @brief Process mode with auto navigation guidance */
void NavigationManager::executeAutoNavMode()
{
	/* Execute guidance */
	_guidMgr.execute();

	/* Compute the command */
	_ctrl.execute();

	/* Move the force into body frame */
	math::Vector3f ctrlFrcDemB =
			system::system.dataPool.estAtt_IB.rotateQconjVQ(
					system::system.dataPool.ctrlFrcDemI);

	/* Scale the force and convert is to integer */
	system::system.dataPool.ctrlFrcDemB(
			ldexpf(ctrlFrcDemB.x, SCALE_TORSOR),
			ldexpf(ctrlFrcDemB.y, SCALE_TORSOR),
			ldexpf(ctrlFrcDemB.z, SCALE_TORSOR));

}

/** @brief Switch to direct thrust */
bool NavigationManager::switchToModeDirectThrust()
{
	return _guidMgr.setMode(NavigationGuidanceManager::E_NAV_GUID_MODE_NONE);
}


/** @brief Switch to loiter*/
bool NavigationManager::switchToModeLoiter()
{
	return _guidMgr.setMode(NavigationGuidanceManager::E_NAV_GUID_MODE_LOITER);
}

/** @brief Switch to way point */
bool NavigationManager::switchToModeWaypoint()
{
	return _guidMgr.setMode(NavigationGuidanceManager::E_NAV_GUID_MODE_WAYPOINT);
}


} /* namespace navigation */

