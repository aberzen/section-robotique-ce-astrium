/*
 * NavigationController.cpp
 *
 *  Created on: 13 déc. 2013
 *      Author: Robotique
 */

#include <autom/ctrl/include/NavigationController.hpp>

namespace autom {

NavigationController::NavigationController(
		/* Inputs */
		const NavGuid::Output& guid,
		const Estimator::Estimations& est,
		/* Outputs */
		::math::Vector3f& force_I,
		/* Parameters */
		 const float& dt,
		 const ControllerPid3Axes::Param& param
		) :
		ControllerPid3Axes(force_I, dt, param),
		_guid(guid),
		_est(est)
{
}

NavigationController::~NavigationController() {
}

/** @brief Update the control error */
void NavigationController::updateCtrlErr()
{
	/* Position error */
	this->_ctrlErr = _guid.posDem_I - _est.position_I;

	/* Velocity error */
	this->_ctrlErrDeriv = _guid.velDem_I - _est.velocity_I;
}

} /* namespace autom */


