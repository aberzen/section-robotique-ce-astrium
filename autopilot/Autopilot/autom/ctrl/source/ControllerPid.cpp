/*
 * ControllerPid.cpp
 *
 *  Created on: 13 déc. 2013
 *      Author: Robotique
 */

#include <hw/serial/include/FastSerial.hpp>
#include <autom/ctrl/include/ControllerPid.hpp>
namespace autom {

ControllerPid::ControllerPid(
		/* Inputs */
		const float& ctrlErr,
		const float& derivCtrlErr,
		/* Outputs */
		float& out,
		/* Parameters */
		const float& dt,
		const Param& param) :
		_ctrlErr(ctrlErr),
		_derivCtrlErr(derivCtrlErr),
		_out(out),
		_dt(dt),
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
void ControllerPid::execute(){
	/* 1) Compute integral term */
	_intCtrlErr +=  this->_param.Ki * (_ctrlErr+_ctrlErrPrev)/2 * this->_dt;
	_ctrlErrPrev = _ctrlErr;

	/* 2) Saturation of integral term */
	if (math_abs(_intCtrlErr)>this->_param.maxI){
		_intCtrlErr  = math_sign(_intCtrlErr) * this->_param.maxI;
	}

	/* 3) Make use of  RB when necessary */
	if (this->_param.useOfRb && (math_abs(_ctrlErr) > this->_param.rbThd)) {
		/* 3b) Rate bias */
		_out = this->_param.Krb * (_derivCtrlErr + math_sign(_ctrlErr)*this->_param.rb);
		_intCtrlErr = 0; // reset the integral term
	}
	else {
		/* 3a) No rate bias */
		_out = this->_param.Kp * _ctrlErr + this->_param.Kd * _derivCtrlErr + this->_intCtrlErr;
	}
}

} /* namespace autom */
