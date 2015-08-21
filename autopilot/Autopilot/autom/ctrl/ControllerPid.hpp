/*
 * ControllerPid.h
 *
 *  Created on: 16 janv. 2013
 *      Author: Aberzen
 */

#ifndef CONTROLLERPID_H_
#define CONTROLLERPID_H_

#include <math/MathUtils.hpp>

namespace autom {



/**
 * @brief PID controller class
 *
 * This controller performs a proportional, derivative and integral control
 * with rate bias support and integral term saturation.
 *
 * The output value is computed in 3 steps.
 * 0) Assumption
 * ctrlErr = guid - meas
 *
 * 1) Compute the integral term
 * intCtrlErr = intCtrlErr + ctrlErr * dt
 *
 * 2) Saturate the integral term
 * intCtrlErr = sign(intCtrlErr) * max(abs(intCtrlErr), maxI);
 *
 * 3) Compute output
 *
 *   3a) Normal case: if abs(ctrlErr)<=rbThd
 *   out = Kp * ctrlErr + Kd * derivCtrlErr + Ki * integral(ctrlErr)
 *
 *   3b) Rate bias case: else abs(ctrlErr)>rbThd
 *   out = Krb * (derivCtrlErr + sign(ctrlErr)*rb)
 *
 */

template<typename T>
class ControllerPid {

public:
	ControllerPid(
			/* Parameters */
			const T& Kp,
			const T& Kd,
			const T& Ki,
			const T& maxI);
	virtual ~ControllerPid();

	/** @brief Init the process */
	virtual void initialize();

	/** @brief Compute command */
	virtual void computeCommand(
			const T& ctrlErr,
			const T& derivCtrlErr,
			T& command);

	inline void setParam(
			const T& Kp,
			const T& Kd,
			const T& Ki,
			const T& maxI);

protected:
	/** @brief Controller parameters */
	const T& _Kp;
	const T& _Kd;
	const T& _Ki;
	const T& _maxI;

	/** @brief Previous control error (used for integral term computation) */
	T _ctrlErrPrev;

	/** @brief Current integral of the control error */
	T _intCtrlErr;
};

template <typename T>
void ControllerPid<T>::setParam(
		const T& Kp,
		const T& Kd,
		const T& Ki,
		const T& maxI)
{
	_Kp = Kp;
	_Kd = Kd;
	_Ki = Ki;
	_maxI = maxI;
}

template<typename T>
ControllerPid<T>::ControllerPid(
		/* Parameters */
		const T& Kp,
		const T& Kd,
		const T& Ki,
		const T& maxI)
: _Kp(Kp),
  _Kd(Kd),
  _Ki(Ki),
  _maxI(maxI),
  _ctrlErrPrev((T)0),
  _intCtrlErr((T)0)
{
}

template<typename T>
ControllerPid<T>::~ControllerPid(){
}

/** @brief Init the process */
template<typename T>
void ControllerPid<T>::initialize()
{
	this->_intCtrlErr = 0;
	this->_ctrlErrPrev = 0;
}

/** @brief Compute the controller value */
template<typename T>
void ControllerPid<T>::computeCommand(
		const T& ctrlErr,
		const T& derivCtrlErr,
		T& command)
{

	/* 1) Compute integral term */
	_intCtrlErr +=  ((ctrlErr+_ctrlErrPrev)>>1);
	_ctrlErrPrev = ctrlErr;

	/* 2) Saturation of integral term */
	if (math_abs(_intCtrlErr)>this->_maxI){
		_intCtrlErr  = math_sign(_intCtrlErr) * this->_maxI;
	}

	/* 3) Compute the command summing the different terms */
	command = this->_Kp * ctrlErr
			+ this->_Ki * this->_intCtrlErr
			+ this->_Kd * derivCtrlErr;
}

} /* namespace autom */
#endif /* CONTROLLERPID_H_ */
