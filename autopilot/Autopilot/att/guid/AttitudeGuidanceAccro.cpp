/*
 * AttitudeGuidanceAccro.cpp
 *
 *  Created on: 27 août 2015
 *      Author: AdministrateurLocal
 */

#include "AttitudeGuidanceAccro.hpp"

namespace attitude {

AttitudeGuidanceAccro::AttitudeGuidanceAccro()
: AttitudeGuidance(),
  _attNormInv(1.)
{
	_paramScalePwm[ATTITUDE_GUIDANCE_IDX_ROLL]  = ATTITUDE_GUIDANCE_MODE_RATE_PWM_SCALE_ROLL;
	_paramScalePwm[ATTITUDE_GUIDANCE_IDX_PITCH] = ATTITUDE_GUIDANCE_MODE_RATE_PWM_SCALE_PITCH;
	_paramScalePwm[ATTITUDE_GUIDANCE_IDX_YAW]   = ATTITUDE_GUIDANCE_MODE_RATE_PWM_SCALE_YAW;
}

AttitudeGuidanceAccro::~AttitudeGuidanceAccro() {
}

/** @brief Initialize internal variable */
void AttitudeGuidanceAccro::initialize(
		const math::Quaternion& quat_IB,
		const math::Vector3f& rate_B)
{
	_attNormInv = 1. / quat_IB.norm();
}

void AttitudeGuidanceAccro::calcGuidance(
		hw::Radio& radio,
		math::Quaternion& guidQuat_IB,
		math::Vector3f& guidRate_B)
{
	/* Build rate from pwm inputs */
	guidRate_B.x = ((float) radio.getSigned(hw::Radio::E_RADIO_CHANNEL_ROLL) )
			* _paramScalePwm[ATTITUDE_GUIDANCE_IDX_ROLL];
	guidRate_B.y = ((float) radio.getSigned(hw::Radio::E_RADIO_CHANNEL_PITCH) )
			* _paramScalePwm[ATTITUDE_GUIDANCE_IDX_PITCH];
	guidRate_B.z = ((float) radio.getSigned(hw::Radio::E_RADIO_CHANNEL_YAW) )
			*_paramScalePwm[ATTITUDE_GUIDANCE_IDX_YAW];

	/* Integrate rate */
	math::Vector3f angInc = guidRate_B * (0.5 * FSW_TASK_CTRL_PERIOD_TICK_PER_SEC);;
	math::Quaternion dQ(
			1 - (angInc*angInc) * 0.5,
			angInc.x - angInc.x*angInc.x*angInc.x * 0.1666667,
			angInc.y - angInc.y*angInc.y*angInc.y * 0.1666667,
			angInc.z - angInc.z*angInc.z*angInc.z * 0.1666667);
	guidQuat_IB = guidQuat_IB * dQ;
	_attNormInv = guidQuat_IB.normalize(1,_attNormInv);
}

} /* namespace attitude */
