/*
 * NavigationController.cpp
 *
 *  Created on: 8 sept. 2015
 *      Author: AdministrateurLocal
 */

#include <nav/ctrl/NavigationController.hpp>
#include <system/params/Nrd.hpp>
#include <system/system/System.hpp>

namespace navigation {

NavigationController::NavigationController(const Parameter& param)
: _param(param),
  _ctrl(param.paramCtrl)
{
}

NavigationController::~NavigationController() {
	// TODO Auto-generated destructor stub
}

/** @brief Exeute the service */
void NavigationController::execute()
{
	/* Compute control errors */
	system::system.dataPool.ctrlNavPosErrI =
			system::system.dataPool.guidPos_I - system::system.dataPool.estPos_I;
	system::system.dataPool.ctrlNavVelErrI =
			system::system.dataPool.guidVel_I - system::system.dataPool.estVel_I;

	/* Compute control force */
	_ctrl.computeCommand(
			system::system.dataPool.ctrlNavPosErrI,
			system::system.dataPool.ctrlNavVelErrI,
			system::system.dataPool.ctrlFrcDemI);

	/* Add feedforward guidance force to the force */
	system::system.dataPool.ctrlFrcDemI += (system::system.dataPool.guidVel_I * _param.mass);

	/* Add feedforward gravity force to the force */
	system::system.dataPool.ctrlFrcDemI += (math::Vector3f(0.,0.,-PHYSICS_GRAVITY) * _param.mass);
}


} /* namespace navigation */
