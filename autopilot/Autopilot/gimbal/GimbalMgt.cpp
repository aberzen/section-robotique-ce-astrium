/*
 * GimbalMgt.cpp
 *
 *  Created on: 29 janv. 2014
 *      Author: Robotique
 */

#include <Arduino.h>
#include "GimbalMgt.hpp"
#include <hw/servo/Servo.hpp>

extern hw::Servo gimbalPitch;
extern hw::Servo gimbalRoll;


namespace autom {

GimbalMgt::GimbalMgt(
		/* Input */
		/* Output */
		/* Param */
		)
{
}

GimbalMgt::~GimbalMgt() {
}

/** @brief Init the process */
void GimbalMgt::initialize()
{
	/* Switch on */
	gimbalPitch.switchOn();
	gimbalRoll.switchOn();
}

/** @brief Execute the process */
void GimbalMgt::orientate(
		const math::Quaternion& quatAtt_IB,
		const math::Quaternion& quatAtt_BT)
{
	/* Pitch */
	math::Vector3f tmp_B(1.,0.,0.);
	math::Vector3f tmp_I = quatAtt_IB.rotateQVQconj(tmp_B);
	float angle = M_PI/2. - acos(tmp_I.z);
	gimbalPitch.moveTo(angle);

	/* Roll */
	tmp_B(0.,1.,0.);
	tmp_I = quatAtt_IB.rotateQVQconj(tmp_B);
	angle = M_PI/2. - acos(tmp_I.z);
	gimbalRoll.moveTo(angle);
}

} /* namespace autom */
