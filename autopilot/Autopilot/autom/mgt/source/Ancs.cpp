/*
 * Ancs.cpp
 *
 *  Created on: 16 déc. 2013
 *      Author: Robotique
 */

#include <autom/mgt/include/Ancs.hpp>
#include <system/system/include/System.hpp>

#include <infra/include/Task.hpp>

namespace autom {

Ancs::Ancs(
		const float& dt_HF,
		const float& dt_LF,
		const Ancs::Param& param)
: procImuCalib( ::system::System::system.board.meas.imu, estimations, param.procCalibImu),
  procCompDec(::system::System::system.board.meas, estimations, *(autom::ProcCompassDeclination::Output*)((void*)&param.est.declination), param.procCompDec),
  modulator(torque_B, force_B, system::System::system.board.pwmVal, torqueReal_B, forceReal_B, param.modGen, param.modPinv, system::System::system.board.pwm),
  torqueReal_B(0.,0.,0.),
  forceReal_B(0.,0.,0.),
  estimations(),
  smGroundContact(param.smGroundContact, param.gen),
  smFlyingState(param.smFLying),
  smGlobal(),
  radioChannels(),
  est(::system::System::system.board.meas, estimations, dt_HF, param.est),
  attCtrl(dt_LF, param.attCtrl),
  filtX(param.filtX),
  filtY(param.filtY),
  filtZ(param.filtZ),
  gimbal(estimations, param.gimbal),
  torque_B(0.,0.,0.),
  force_B(0.,0.,0.),
  attDem_IB(1.,0.,0.,0.),
  rateDem_B(0.,0.,0.),
  attCtrlError(0.,0.,0.),
  rateCtrlError(0.,0.,0.),
  modeMgt(dt_LF, param.modeMgt, param.gen),
  _param(param)
{
	uint8_t iChannel;
	for (iChannel=0 ; iChannel<PWM_OUT_NUM_CHANNELS ; iChannel++)
	{
		radioChannels[iChannel] = new RadioChannel(
				system::System::system.board.radio.channels[iChannel],
				_param.radioChannel[iChannel]);
	}
}

Ancs::~Ancs() {
}



} /* namespace autom */
