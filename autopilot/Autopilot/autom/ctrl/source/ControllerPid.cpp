/*
 * ControllerPid.cpp
 *
 *  Created on: 13 déc. 2013
 *      Author: Robotique
 */

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
		_ctrlErrPrev(0),
		_intCtrlErr(0)
		 {
}

ControllerPid::~ControllerPid(){
}

/** @brief Init the process */
infra::status ControllerPid::initialize()
{
	this->_intCtrlErr = 0.;
	this->_ctrlErrPrev = 0.;
	return 0;
}

/** @brief Compute the controller value */
infra::status ControllerPid::execute(){
	/* 1) Compute integral term */
	_intCtrlErr +=  this->_param._Ki * (_ctrlErr+_ctrlErrPrev)/2 * this->_dt;
	_ctrlErrPrev = _ctrlErr;

	/* 2) Saturation of integral term */
	if (math_abs(_intCtrlErr)>this->_param._maxI){
		_intCtrlErr  = math_sign(_intCtrlErr) * this->_param._maxI;
	}

	/* 3) Make use of  RB when necessary */
	if (this->_param._useOfRb && (math_abs(_ctrlErr) > this->_param._rbThd)) {
		/* 3b) Rate bias */
		_out = this->_param._Krb * (_derivCtrlErr + math_sign(_ctrlErr)*this->_param._rb);
		_intCtrlErr = 0; // reset the integral term
	}
	else {
		/* 3a) No rate bias */
		_out = this->_param._Kp * _ctrlErr + this->_param._Kd * _derivCtrlErr + this->_intCtrlErr;
	}
	return 0;
}

} /* namespace autom */
