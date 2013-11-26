/*
 * ControllerPid.h
 *
 *  Created on: 16 janv. 2013
 *      Author: Aberzen
 */

#ifndef CONTROLLERPID_H_
#define CONTROLLERPID_H_

namespace autom {

/**
 * @brief PID controller class
 *
 * This controller performs a proportional, derivative and integral control
 * with rate bias support and integral term saturation.
 *
 * The output value is computed in XXX steps.
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

template <typename T>
class ControllerPid {
public:
	ControllerPid(
			T dt,
			T Kp,
			T Kd,
			T Ki,
			T maxI,
			bool useOfRb=false,
			T Krb=((T)0),
			T rbThd=((T)0),
			T rb=((T)0));
	virtual ~ControllerPid();

	/** @brief Reset integral term with optional given value */
	inline void reset(T initIntCtrlErr=((T)0));
	/** @brief Getter method of integration period */
	inline T getPeriod();
	/** @brief Setter method of integration period */
	inline void setPeriod(T dt);
	/** @brief Getter method of static gain */
	inline T getKp();
	/** @brief Setter method of static gain */
	inline void setKp(T Kp);
	/** @brief Getter method of derivative gain */
	inline T getKd();
	/** @brief Setter method of derivative gain */
	inline void setKd(T Kd);
	/** @brief Getter method of integrative gain */
	inline T getKi();
	/** @brief Setter method of integrative gain */
	inline void setKi(T Ki);
	/** @brief Getter method of maximum integral value */
	inline T getMaxI();
	/** @brief Setter method of maximum integral value */
	inline void setMaxI(T maxI);
	/** @brief Activate/desactivate the rate bias */
	inline void activateRateBias(bool isRbActivated);
	/** @brief Getter method of rate bias gain */
	inline T getKrb();
	/** @brief Setter method of rate bias gain */
	inline void setKrb(T Krb);
	/** @brief Getter method of rate bias threshold */
	inline T getRbTh();
	/** @brief Setter method of rate bias threshold */
	inline void setRbTh(T rbThd);

	/** @brief Compute the controller value */
	T compute(T ctrlErr, T derivCtrlErr, T dt) ;

private:
	/** @brief Current integral of the control error */
	T _dt;
	/** @brief Current integral of the control error */
	T _intCtrlErr;
	/** @brief Controller static gain */
	T _Kp;
	/** @brief Controller derivative gain */
	T _Kd;
	/** @brief Controller integral gain */
	T _Ki;
	/** @brief Make use of Rate Bias */
	bool _useOfRb;
	/** @brief Controller rate bias gain */
	T _Krb;
	/** @brief Controller rate bias threshold */
	T _rbThd;
	/** @brief Controller rate bias value */
	T _rb;
	/** @brief Controller integral saturation */
	T _maxI;
};

/** @brief Getter method of integration period */
template <typename T>
T ControllerPid<T>::getPeriod() {
	return this->_dt;
}

/** @brief Setter method of integration period */
template <typename T>
void ControllerPid<T>::setPeriod(T dt) {
	this->_dt = dt;
}

/** @brief Reset integral term with optional given value */
template <typename T>
void ControllerPid<T>::reset(T initIntCtrlErr){
	this->_intCtrlErr = initIntCtrlErr;
}

/** @brief Getter method of static gain */
template <typename T>
T ControllerPid<T>::getKp(){
	return this->_Kp;
}

/** @brief Setter method of static gain */
template <typename T>
void ControllerPid<T>::setKp(T Kp){
	this->_Kp = Kp;
}

/** @brief Getter method of derivative gain */
template <typename T>
T ControllerPid<T>::getKd(){
	return this->_Kd;
}

/** @brief Setter method of derivative gain */
template <typename T>
void ControllerPid<T>::setKd(T Kd){
	this->_Kd = Kd;
}

/** @brief Getter method of integrative gain */
template <typename T>
T ControllerPid<T>::getKi(){
	return this->_Ki;
}

/** @brief Setter method of integrative gain */
template <typename T>
void ControllerPid<T>::setKi(T Ki){
	this->_Ki = Ki;
}

/** @brief Activate/desactivate the rate bias */
template <typename T>
void ControllerPid<T>::activateRateBias(bool isRbActivated){
	this->_useOfRb = isRbActivated;
}

/** @brief Getter method of rate bias gain */
template <typename T>
T ControllerPid<T>::getKrb(){
	return this->_Krb;
}

/** @brief Setter method of rate bias gain */
template <typename T>
void ControllerPid<T>::setKrb(T Krb){
	this->_Krb = Krb;
}

/** @brief Getter method of rate bias threshold */
template <typename T>
T ControllerPid<T>::getRbTh(){
	return this->_rbThd;
}

/** @brief Setter method of rate bias threshold */
template <typename T>
void ControllerPid<T>::setRbTh(T rbThd){
	this->_rbThd = rbThd;
}

/** @brief Getter method of maximum integral value */
template <typename T>
T ControllerPid<T>::getMaxI(){
	return this->_maxI;
}

/** @brief Setter method of maximum integral value */
template <typename T>
void ControllerPid<T>::setMaxI(T maxI){
	this->_maxI = maxI;
}

#include "../ctrl/ControllerPid.cpp_"

} /* namespace autom */
#endif /* CONTROLLERPID_H_ */
