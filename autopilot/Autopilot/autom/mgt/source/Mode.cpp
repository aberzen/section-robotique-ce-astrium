/*
 * Mode.cpp
 *
 *  Created on: 2 janv. 2014
 *      Author: Robotique
 */

#include <autom/mgt/include/Mode.hpp>

namespace autom {

Mode::Mode(
		/* Input */
		const Estimator::Estimations& est,
		const ::math::Vector3f& force_I,
		/* Outputs */
		AttGuid::Output& attCtrlIn,
		NavGuid::Output& navCtrlIn,
		::math::Vector3f& force_B,
		/* Parameters */
		const ControllerPid3Axes::Param& paramAttCtrl,
		const ControllerPid3Axes::Param& paramNavCtrl,
		/* Dependencies */
		AttitudeController& attCtrl,
		NavigationController& navCtrl
		)
: _est(est),
  _force_I(force_I),
  _navCtrlIn(navCtrlIn),
  _attCtrlIn(attCtrlIn),
  _force_B(force_B),
  _paramAttCtrl(paramAttCtrl),
  _paramNavCtrl(paramNavCtrl),
  _attCtrl(attCtrl),
  _navCtrl(navCtrl)
{
}

Mode::~Mode() {
	// TODO Auto-generated destructor stub
}

/** @brief Init the process */
::infra::status Mode::initialize()
{
	::infra::status res;

	/* Reinitialize the attitude controller */
	_attCtrl.setParam(_paramAttCtrl);
	res = _attCtrl.initialize();
	if (res < 0)
		return res;

	/* Reinitialize the navigation controller */
	_navCtrl.setParam(_paramNavCtrl);
	res = _navCtrl.initialize();
	if (res < 0)
		return res;

	return 0;
}

} /* namespace autom */
