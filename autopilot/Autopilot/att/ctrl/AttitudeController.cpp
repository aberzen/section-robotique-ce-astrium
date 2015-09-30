/*
 * AttitudeController.cpp
 *
 *  Created on: 19 août 2015
 *      Author: AdministrateurLocal
 */

#include <system/system/System.hpp>
#include "AttitudeController.hpp"

namespace attitude {

AttitudeController::AttitudeController(const AttitudeController::Parameter& param)
: _ctrl(param.ctrl),
  _param(param)
{
}

AttitudeController::~AttitudeController() {
}

/** @brief Compute torque */
void AttitudeController::execute()
{
	const math::Quaternion& qDem_IB = system::system.dataPool.guidAtt_IB;
	const math::Quaternion& qEst_IB = system::system.dataPool.estAtt_IB;
	const math::Vector3f& rateDem_B = system::system.dataPool.guidRate_B;
	const math::Vector3f& rateEst_B = system::system.dataPool.estRate_B;

	math::Vector3f rateDemSaturated_B(rateDem_B);

	/* Attitude error computed as the conj(qEst)*qDem */
	system::system.dataPool.ctrlAttAngErrB = (~qEst_IB) * qDem_IB;
	system::system.dataPool.ctrlAttAngErrB *= (float) math_sign(system::system.dataPool.ctrlAttAngErrB.scalar);
//	math::Quaternion ctrlErrAttSatB(system::system.dataPool.ctrlAttAngErrB);

	/* Check and saturate if necessary */
//	if (ctrlErrAttSatB.scalar < _param.maxCosAngOverTwoErr)
//	{
//		ctrlErrAttSatB(
//				_param.maxCosAngOverTwoErr,
//				ctrlErrAttSatB.vector * (_param.maxSinAngOverTwoErr / ctrlErrAttSatB.vector.norm()));
//
//		math::Quaternion qDem_BBsaturated = (~system::system.dataPool.ctrlAttAngErrB) * ctrlErrAttSatB;
//		rateDemSaturated_B = qDem_BBsaturated.rotateQconjVQ(rateDem_B);
//	}

	/* Set angular control error */
	math::Vector3l ctrlErrAttB(
			((int32_t) ldexpf(system::system.dataPool.ctrlAttAngErrB.vector.x,SCALE_TORSOR+1)),
			((int32_t) ldexpf(system::system.dataPool.ctrlAttAngErrB.vector.y,SCALE_TORSOR+1)),
			((int32_t) ldexpf(system::system.dataPool.ctrlAttAngErrB.vector.z,SCALE_TORSOR+1)));

	/* Set angular control error */
	system::system.dataPool.ctrlAttRateErrB = rateDemSaturated_B - rateEst_B;
	math::Vector3l crlAttRateErrB(
			ldexpf(system::system.dataPool.ctrlAttRateErrB.x,SCALE_TORSOR),
			ldexpf(system::system.dataPool.ctrlAttRateErrB.y,SCALE_TORSOR),
			ldexpf(system::system.dataPool.ctrlAttRateErrB.z,SCALE_TORSOR));

	/* Compute controller */
	_ctrl.computeCommand(
			ctrlErrAttB,
			crlAttRateErrB,
			system::system.dataPool.ctrlTrqDemB);

	/* Set the commanded torque */
	system::system.dataPool.ctrlTrqDemB(
			system::system.dataPool.ctrlTrqDemB.x >> SCALE_ATT_CTRL_GAIN,
			system::system.dataPool.ctrlTrqDemB.y >> SCALE_ATT_CTRL_GAIN,
			system::system.dataPool.ctrlTrqDemB.z >> SCALE_ATT_CTRL_GAIN);
}

} /* namespace attitude */
