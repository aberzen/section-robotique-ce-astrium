/*
 * GimbalMgt.cpp
 *
 *  Created on: 29 janv. 2014
 *      Author: Robotique
 */

#include <autom/gimbal/include/GimbalMgt.hpp>
#include <system/system/include/System.hpp>

namespace autom {

GimbalMgt::GimbalMgt(
		/* Input */
		const autom::Estimator::Estimations& estVal,
		/* Output */
		/* Param */
		const Param& param)
: Process(),
  _estVal(estVal),
  _pitch(system::System::system.board.pwmVal.channels[param.pitch.idxChannel], param.pitch, system::System::system.board.pwm),
  _roll(system::System::system.board.pwmVal.channels[param.roll.idxChannel], param.roll, system::System::system.board.pwm)
{
}

GimbalMgt::~GimbalMgt() {
}

/** @brief Init the process */
void GimbalMgt::initialize()
{
	/* Initialize servos */
	_pitch.initialize();
	_roll.initialize();

	/* Switch on */
	_pitch.switchOn();
	_roll.switchOn();
}

/** @brief Execute the process */
void GimbalMgt::execute()
{
	/* Pitch */
	math::Vector3f tmp_B(1.,0.,0.);
	math::Vector3f tmp_I = _estVal.attitude_IB.rotateQVQconj(tmp_B);
	float angle = M_PI_2 - acos(tmp_I.z);
	_pitch.moveTo(angle);

	/* Roll */
	tmp_B(0.,1.,0.);
	tmp_I = _estVal.attitude_IB.rotateQVQconj(tmp_B);
	angle = M_PI_2 - acos(tmp_I.z);
	_roll.moveTo(angle);
}

} /* namespace autom */
