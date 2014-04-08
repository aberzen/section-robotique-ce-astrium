/*
 * ControllerPid.cpp
 *
 *  Created on: 13 déc. 2013
 *      Author: Robotique
 */

#include <autom/ctrl/include/ControllerPid.hpp>

namespace autom {

ControllerPid::ControllerPid(
		/* Parameters */
		const float& dt,
		const Param& param)
: _dt(dt),
  _param(param),
  _ctrlErrPrev(0.),
  _intCtrlErr(0.)
{
}

ControllerPid::~ControllerPid(){
}

/** @brief Init the process */
void ControllerPid::initialize()
{
	this->_intCtrlErr = 0.;
	this->_ctrlErrPrev = 0.;
}

/** @brief Compute the controller value */
void ControllerPid::compCommand(
		const float& ctrlErr,
		const float& derivCtrlErr,
		float& command)
{
	/* 1) Compute integral term */
	_intCtrlErr +=  this->_param.Ki * (ctrlErr+_ctrlErrPrev)/2 * this->_dt;
	_ctrlErrPrev = (float)ctrlErr;

	/* 2) Saturation of integral term */
	if (math_abs(_intCtrlErr)>this->_param.maxI){
		_intCtrlErr  = math_sign(_intCtrlErr) * this->_param.maxI;
	}

	/* 3) Make use of  RB when necessary */
	if (this->_param.useOfRb && (math_abs(ctrlErr) > this->_param.rbThd)) {
		/* 3b) Rate bias */
		command = this->_param.Krb * (derivCtrlErr + math_sign(ctrlErr)*this->_param.rb);
		_intCtrlErr = 0; // reset the integral term
	}
	else {
		/* 3a) No rate bias */
		command = this->_param.Kp * ctrlErr + this->_param.Kd * derivCtrlErr + this->_intCtrlErr;
	}
}

} /* namespace autom */
