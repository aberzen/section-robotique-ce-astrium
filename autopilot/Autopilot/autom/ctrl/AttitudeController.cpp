/*
 * AttitudeController.cpp
 *
 *  Created on: 19 août 2015
 *      Author: AdministrateurLocal
 */

#include <system/system/System.hpp>
#include <autom/ctrl/AttitudeController.hpp>

namespace autom {

AttitudeController::AttitudeController(
		const math::Vector3<int32_t>& Kp,
		const math::Vector3<int32_t>& Kd,
		const math::Vector3<int32_t>& Ki,
		const math::Vector3<int32_t>& maxI)
: _ctrl(Kp, Kd, Ki, maxI)
{
}

AttitudeController::~AttitudeController() {
}

/** @brief Compute torque */
void AttitudeController::computeTorque(
		math::Quaternion qDem_IB,
		math::Quaternion qEst_IB,
		math::Vector3f rateDem_B,
		math::Vector3f rateEst_B)
{

	/* Attitude error computed as the conj(qEst)*qDem */
	::math::Quaternion dQ = (~qEst_IB) * qDem_IB;
	system::system.dataPool.ctrlErrAttB(
			((int32_t) ldexpf(dQ.vector.x,11)) * math_sign(dQ.scalar),
			((int32_t) ldexpf(dQ.vector.y,11)) * math_sign(dQ.scalar),
			((int32_t) ldexpf(dQ.vector.z,11)) * math_sign(dQ.scalar));

	math::Vector3f rateErrorFloat_B = rateDem_B - rateEst_B;
	system::system.dataPool.ctrlErrRateB(
			ldexpf(rateErrorFloat_B.x,10),
			ldexpf(rateErrorFloat_B.y,10),
			ldexpf(rateErrorFloat_B.z,10));

	_ctrl.computeCommand(
			system::system.dataPool.ctrlErrAttB,
			system::system.dataPool.ctrlErrRateB,
			system::system.dataPool.ctrlTrqDemB);

	system::system.dataPool.ctrlTrqDemB(
			system::system.dataPool.ctrlTrqDemB.x >> SCALE_ATT_CTRL_GAIN,
			system::system.dataPool.ctrlTrqDemB.y >> SCALE_ATT_CTRL_GAIN,
			system::system.dataPool.ctrlTrqDemB.z >> SCALE_ATT_CTRL_GAIN);
}

} /* namespace autom */
