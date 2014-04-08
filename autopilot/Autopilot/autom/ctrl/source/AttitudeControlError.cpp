/*
 * AttitudeControlError.cpp
 *
 *  Created on: 13 déc. 2013
 *      Author: Robotique
 */

#include <autom/ctrl/include/AttitudeControlError.hpp>

namespace autom {

/** @brief Update the control error */
void AttitudeControlError::compControlError(
		const math::Quaternion& qDem_IB,
		const math::Quaternion& qEst_IB,
		math::Vector3f& error)
{
	/* Attitude error computed as the conj(qEst)*qDem */
	::math::Quaternion dQ = (~qEst_IB) * qDem_IB;
	error(dQ.vector * (2.*math_sign(dQ.scalar)));
}

} /* namespace autom */

