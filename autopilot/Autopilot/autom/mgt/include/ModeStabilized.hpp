/*
 * ModeStabilized.hpp
 *
 *  Created on: 2 janv. 2014
 *      Author: Robotique
 */

#ifndef MODESTABILIZED_HPP_
#define MODESTABILIZED_HPP_

#include <autom/mgt/include/Mode.hpp>

namespace autom {

class ModeStabilized: public autom::Mode {
public:
	typedef struct
	{
		int8_t rollPwmScale; /* in log2(rad/ms), i.e. scale is 2^rollScale (typically -10 for +/-35deg or -9 for +/-70deg) */
		int8_t pitchPwmScale; /* in log2(rad/ms), i.e. scale is 2^pitchScale (typically -10 for +/-35deg or -9 for +/-70deg) */
		int8_t yawRatePwmScale; /* in log2(rad/s/ms), i.e. scale is 2^yawRateScale (typically -10 for +/-35deg/s or -9 for +/-70deg/s) */
		int8_t thrustPwmScale; /* in log2(N/ms), i.e. scale is 2^thrustScale (typically -6 for [0 2g]) */
		::math::Vector3f thrustDir_B;
		float mass;
	} Param ;
public:
	ModeStabilized(
			/* Input */
			const Estimator::Estimations& est,
			const ::math::Vector3f& force_I,
			/* Outputs */
			AttGuid::Output& attCtrlIn,
			NavGuid::Output& navCtrlIn,
			::math::Vector3f& force_B,
			/* Parameters */
			const float& dt,
			const autom::ModeStabilized::Param& param,
			const ControllerPid3Axes::Param& paramAttCtrl,
			const ControllerPid3Axes::Param& paramNavCtrl,
			/* Dependencies */
			AttitudeController& attCtrl,
			NavigationController& navCtrl
			);
	virtual ~ModeStabilized();

	/** @brief Init the process */
	virtual ::infra::status initialize();

	/** @brief Execute the process */
	virtual ::infra::status execute();

protected:

	/** @brief Parameters */
	const autom::ModeStabilized::Param& _param;

	/** @brief Time step duration */
	const float& _dt;

	/** @brief Rotation around z */
	math::Quaternion _rotZ;

	/** @brief Previous inverse of rotZ norm */
	float _prevInvNormRotZ;

	/** @brief Previous inverse of guidance norm */
	float _prevInvNormGuid;

	/** @brief Previous roll angle */
	float _angleRollPrev;

	/** @brief Previous pitch angle */
	float _anglePitchPrev;

	/** @brief Previous thrust */
	float _thrustPrev;
};

} /* namespace autom */

#endif /* MODESTABILIZED_HPP_ */
