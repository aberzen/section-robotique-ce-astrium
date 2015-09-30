/*
 * AttitudeGuidanceAutoStab.hpp
 *
 *  Created on: 27 août 2015
 *      Author: AdministrateurLocal
 */

#ifndef ATT_GUID_ATTITUDEGUIDANCEAUTOSTAB_HPP_
#define ATT_GUID_ATTITUDEGUIDANCEAUTOSTAB_HPP_

#include <hw/radio/Radio.hpp>
#include "AttitudeGuidance.hpp"

namespace attitude {

class AttitudeGuidanceAutoStab : public AttitudeGuidance {
public:
	AttitudeGuidanceAutoStab();
	virtual ~AttitudeGuidanceAutoStab();

	/** @brief Initialize internal variable */
	virtual void initialize(
			const math::Quaternion& quat_IB,
			const math::Vector3f& rate_B);

	/** @brief Compute the guidance */
	void calcGuidance(
			hw::Radio& radio,
			math::Quaternion& guidQuat_IB,
			math::Vector3f& guidRate_B,
			bool updateYaw);

protected:
	/** @brief Scale parameter in Autostab mode */
	float _paramScalePwm[3];

	/** @brief Prev Roll angle */
	float _angRollPrev;

	/** @brief Prev Pitch angle */
	float _angPitchPrev;

	/** @brief Prev Yaw angle */
	float _angYawPrev;

	/** @brief Norm */
	float _attNormInv;
};

} /* namespace attitude */

#endif /* ATT_GUID_ATTITUDEGUIDANCEAUTOSTAB_HPP_ */
