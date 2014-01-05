/*
 * AttitudeController.cpp
 *
 *  Created on: 13 déc. 2013
 *      Author: Robotique
 */

#include <hw/serial/include/FastSerial.hpp>
#include <autom/ctrl/include/AttitudeController.hpp>

namespace autom {

AttitudeController::AttitudeController (
		/* Inputs */
		const AttGuid::Output& guid,
		const Estimator::Estimations& est,
		/* Outputs */
		::math::Vector3f& torque_B,
		/* Parameters */
		const float& dt,
		const ControllerPid3Axes::Param& param) :
			ControllerPid3Axes::ControllerPid3Axes(torque_B, dt, param),
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
//	Serial.printf("qEst_IB = {%.5f %.5f %.5f %.5f}\n", _est.attitude_IB.scalar, _est.attitude_IB.vector.x, _est.attitude_IB.vector.y, _est.attitude_IB.vector.z);
//	Serial.printf("qDem_IB = {%.5f %.5f %.5f %.5f}\n", _guid.qDem_IB.scalar, _guid.qDem_IB.vector.x, _guid.qDem_IB.vector.y, _guid.qDem_IB.vector.z);
//	Serial.printf("%.5f;%.5f;%.5f;%.5f\n", _guid.qDem_IB.scalar, _guid.qDem_IB.vector.x, _guid.qDem_IB.vector.y, _guid.qDem_IB.vector.z);
	::math::Quaternion dQ = (~_est.attitude_IB) * _guid.qDem_IB;
	this->_ctrlErr = dQ.vector * (2.*math_sign(dQ.scalar));

	/* Rate error computed */
	this->_ctrlErrDeriv = _guid.angRateDem_B - _est.rate_B;

//	Serial.printf("ctrlErr = {%.5f %.5f %.5f}\n", this->_ctrlErr.x, this->_ctrlErr.y, this->_ctrlErr.z);
//	Serial.printf("ctrlErrDeriv = {%.5f %.5f %.5f}\n", this->_ctrlErrDeriv.x, this->_ctrlErrDeriv.y, this->_ctrlErrDeriv.z);
}

} /* namespace autom */

