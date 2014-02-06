/*
 * Modulator.hpp
 *
 *  Created on: 24 déc. 2013
 *      Author: Robotique
 */

#ifndef MODULATOR_HPP_
#define MODULATOR_HPP_

#include <hw/pwm/include/Pwm.hpp>
#include <infra/app/include/Process.hpp>
#include <math/include/Vector3.hpp>

namespace autom {

template <int8_t NB_MOTORS>
class Modulator : public infra::Process {
public:
	typedef struct
	{
		float infMat[6][NB_MOTORS];
	} ParamGen;

	typedef enum {
		E_STATE_DISARMED = 0,
		E_STATE_ARMED
	} State;

public:

	Modulator(
			/* Input */
			const ::math::Vector3f& torque_B,
			const ::math::Vector3f& force_B,
			/* Output */
			hw::Pwm::Input& out,
			::math::Vector3f& torqueReal_B,
			::math::Vector3f& forceReal_B,
			/* Param */
			const ParamGen& param,
			/* Dependencies */
			hw::Pwm& pwm
			);
	virtual ~Modulator();

	/** @brief Init the process */
	virtual void initialize();

	/** @brief Execute the process */
	virtual void execute() ;

	/** @brief Arm the motors */
	virtual void arm() ;

	/** @brief Disarm the motors */
	virtual void disarm() ;

protected:

	/** @brief Update the produced torsor */
	virtual void updateRealTosor() ;

	/** @brief Update the PWM value */
	virtual void updatePwm() = 0 ;

protected:

	/** @brief Realized torque in body frame */
	const ::math::Vector3f& _torque_B;

	/** @brief Realized force in body frame */
	const ::math::Vector3f& _force_B;

	/** @brief Pwm */
	hw::Pwm::Input& _out;

	/** @brief Realized torque in body frame */
	::math::Vector3f& _torqueReal_B;

	/** @brief Realized force in body frame */
	::math::Vector3f& _forceReal_B;

	/** @brief Parameter */
	const ParamGen& _paramGen;

	/** @brief Pwm */
	hw::Pwm& _pwm;

	/** @brief Parameter */
	float _sqRatios[NB_MOTORS];

	/** @brief State of internal state machine */
	State _state;
};


template <int8_t NB_MOTORS>
Modulator<NB_MOTORS>::Modulator(
		/* Input */
		const ::math::Vector3f& torque_B,
		const ::math::Vector3f& force_B,
		/* Output */
		hw::Pwm::Input& out,
		::math::Vector3f& torqueReal_B,
		::math::Vector3f& forceReal_B,
		/* Param */
		const ParamGen& param,
		/* Dependencies */
		hw::Pwm& pwm
		)
: Process(),
  _torque_B(torque_B),
  _force_B(force_B),
  _out(out),
  _torqueReal_B(torqueReal_B),
  _forceReal_B(forceReal_B),
  _paramGen(param),
  _pwm(pwm)
{
	// TODO Auto-generated constructor stub

}

template <int8_t NB_MOTORS>
Modulator<NB_MOTORS>::~Modulator() {
	// TODO Auto-generated destructor stub
}


/** @brief Init the process */
template <int8_t NB_MOTORS>
void Modulator<NB_MOTORS>::initialize()
{
	uint8_t iMotor;

	for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
	{
		_out.channels[iMotor] = MIN_PULSEWIDTH;
		_sqRatios[iMotor] = 0;
		_pwm.force_out(iMotor);
		_pwm.enable_out(iMotor);
	}

	_torqueReal_B(0.,0.,0.);
	_forceReal_B(0.,0.,0.);
}

/** @brief Init the process */
template <int8_t NB_MOTORS>
void Modulator<NB_MOTORS>::execute()
{
	if (_state == E_STATE_ARMED)
	{
		/* Compute PWM */
		updatePwm();

		/* Compute produced torsor */
		updateRealTosor();
	}
	else
	{
		uint8_t iMotor;
		for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
		{
			_out.channels[iMotor] = MIN_PULSEWIDTH;
			_sqRatios[iMotor] = 0;
		}

		/* Produced torsor is zero */
		_torqueReal_B(0.,0.,0.);
		_forceReal_B(0.,0.,0.);
	}
}

/** @brief Arm the motors */
template <int8_t NB_MOTORS>
void Modulator<NB_MOTORS>::arm()
{
	_state = E_STATE_ARMED;
}

/** @brief Disarm the motors */
template <int8_t NB_MOTORS>
void Modulator<NB_MOTORS>::disarm()
{
	uint8_t idxMotor;
	for (idxMotor=0 ; idxMotor<NB_MOTORS ; idxMotor++)
	{
		_out.channels[idxMotor] = MIN_PULSEWIDTH;
		_sqRatios[idxMotor] = 0;
		_pwm.force_out(idxMotor);
	}
	_state = E_STATE_DISARMED;
}

/** @brief Update the produced torsor */
template <int8_t NB_MOTORS>
void Modulator<NB_MOTORS>::updateRealTosor()
{
	uint8_t iMotor;

	/* Initialize to zero */
	_torqueReal_B(0.,0.,0.);
	_forceReal_B(0.,0.,0.);

	/* Compute the produced torsor */
	for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
	{
		_torqueReal_B.x += _paramGen.infMat[0][iMotor] * _sqRatios[iMotor];
		_torqueReal_B.y += _paramGen.infMat[1][iMotor] * _sqRatios[iMotor];
		_torqueReal_B.z += _paramGen.infMat[2][iMotor] * _sqRatios[iMotor];
		_forceReal_B.x += _paramGen.infMat[3][iMotor] * _sqRatios[iMotor];
		_forceReal_B.y += _paramGen.infMat[4][iMotor] * _sqRatios[iMotor];
		_forceReal_B.z += _paramGen.infMat[5][iMotor] * _sqRatios[iMotor];
	}

}

} /* namespace autom */

#endif /* MODULATOR_HPP_ */
