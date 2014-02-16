/*
 * ControllerPid.h
 *
 *  Created on: 16 janv. 2013
 *      Author: Aberzen
 */

#ifndef CONTROLLERPID_H_
#define CONTROLLERPID_H_

#include <math/include/MathUtils.hpp>
#include <infra/app/include/Process.hpp>

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

class ControllerPid : public infra::Process {
public:
	typedef struct
	{
		/** @brief Controller static gain */
		float Kp;
		/** @brief Controller derivative gain */
		float Kd;
		/** @brief Controller integral gain */
		float Ki;
		/** @brief Make use of Rate Bias */
		bool useOfRb;
		/** @brief Controller rate bias gain */
		float Krb;
		/** @brief Controller rate bias threshold */
		float rbThd;
		/** @brief Controller rate bias value */
		float rb;
		/** @brief Controller integral saturation */
		float maxI;
	} Param ;

public:
	ControllerPid(
			/* Inputs */
			const float& ctrlErr,
			const float& derivCtrlErr,
			/* Outputs */
			float& out,
			/* Parameters */
			const float& dt,
			const Param& param
			);
	virtual ~ControllerPid();

	/** @brief Getter method of integral term of the controller */
	inline const float& getIntErr();
	/** @brief Setter method of integral term of the controller */
	inline void setIntErr(float initIntCtrlErr);
	/** @brief Getter method of integration period */
	inline const float& getPeriod();
	/** @brief Getter method of controller parameters */
	inline const Param& getParam();
	/** @brief Setter method of controller parameters */
	inline void setParam(const Param& param);

	/** @brief Init the process */
	virtual void initialize();

	/** @brief Execute the process */
	virtual void execute();

protected:
	/* @brief Control error */
	const float& _ctrlErr;

	/* @brief Derivative of the control error */
	const float& _derivCtrlErr;

	/* Outputs (torque, force, etc) */
	float& _out;

	/** @brief Integration period */
	const float& _dt;

	/** @brief Controller parameters */
	const Param& _param;

	/** @brief Previous control error (used for integral term computation) */
	float _ctrlErrPrev;

	/** @brief Current integral of the control error */
	float _intCtrlErr;
};

/** @brief Getter method of integral term */
const float& ControllerPid::getIntErr() {
	return this->_intCtrlErr;
}

/** @brief Getter method of integral term */
void ControllerPid::setIntErr(float intCtrlErr) {
	this->_intCtrlErr = intCtrlErr;
}

/** @brief Getter method of integration period */
const float& ControllerPid::getPeriod() {
	return this->_dt;
}

/** @brief Getter method of controller parameters */
inline const ControllerPid::Param& ControllerPid::getParam()
{
	return this->_param;
}

/** @brief Setter method of controller parameters */
void ControllerPid::setParam(const ControllerPid::Param& param)
{
	((ControllerPid::Param&)_param) = param;
}

} /* namespace autom */
#endif /* CONTROLLERPID_H_ */
