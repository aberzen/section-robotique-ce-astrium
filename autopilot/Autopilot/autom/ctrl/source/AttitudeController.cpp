/*
 * AttitudeController.cpp
 *
 *  Created on: 13 déc. 2013
 *      Author: Robotique
 */

#include <autom/ctrl/include/AttitudeController.hpp>

namespace autom {

AttitudeController::AttitudeController (
		/* Inputs */
		const Input& guid,
		const Estimator::Estimations& est,
		/* Outputs */
		::math::Vector3f& torque_B,
		/* Parameters */
		const float& dt,
		const ControllerPid3Axes::Param& param)
: ControllerPid3Axes::ControllerPid3Axes(torque_B, dt, param),
  _guid(guid),
  _est(est)
{
}


AttitudeController::~AttitudeController() {
}

/** @brief Execute the process */
void AttitudeController::updateCtrlErr()
{
	/* Attitude error computed as the conj(qEst)*qDem */
	::math::Quaternion dQ = (~_est.attitude_IB) * _guid.qDem_IB;
	this->_ctrlErr = dQ.vector * (2.*math_sign(dQ.scalar));

	/* Rate error computed */
	this->_ctrlErrDeriv = _guid.angRateDem_B - _est.rate_B;
}

} /* namespace autom */

