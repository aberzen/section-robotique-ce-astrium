/*
 * Modulator.hpp
 *
 *  Created on: 24 déc. 2013
 *      Author: Robotique
 */

#ifndef MODULATOR_HPP_
#define MODULATOR_HPP_

#include <hw/pwm/Pwm.hpp>

#include <system/params/Nrd.hpp>

#include <math/Vector3.hpp>


#define MODULATOR_MOTOR_MAX ((int32_t) (MAX_PULSEWIDTH - MIN_PULSEWIDTH))

namespace autom {

class Modulator {
public:
	typedef int32_t InfluenceMatrix[6][CNF_NB_MOTORS];

public:

	Modulator(
			const InfluenceMatrix& inflMat,
			const math::Vector3l& frcMaxPos_B,
			const math::Vector3l& frcMaxNeg_B,
			const math::Vector3l& trqMaxPos_B,
			const math::Vector3l& trqMaxNeg_B);
	virtual ~Modulator();

	/** @brief Init the process */
	virtual void initialize();

	/** @brief Execute the process */
	virtual void calcMotorCommand(
			const math::Vector3l& force_B,
			const math::Vector3l& torque_B,
			hw::pwm_t* command) ;

	/** @brief Execute the process */
	virtual void calcTorsor(
			const hw::pwm_t* command,
			math::Vector3l& force_B,
			math::Vector3l& torque_B) ;

protected:

	/** @brief Calc motor command*/
	virtual void calcMotorCommandPriv(
			const math::Vector3l& force_B,
			const math::Vector3l& torque_B,
			hw::pwm_t command[CNF_NB_MOTORS]) = 0 ;

	/** @brief Parameter */
	const InfluenceMatrix& _inflMat;

	/** @brief Max positive force */
	const math::Vector3l& _frcMaxPos_B;

	/** @brief Max negative force */
	const math::Vector3l& _frcMaxNeg_B;

	/** @brief Max positive torque */
	const math::Vector3l& _trqMaxPos_B;

	/** @brief Max negative torque */
	const math::Vector3l& _trqMaxNeg_B;
};

} /* namespace autom */

#endif /* MODULATOR_HPP_ */
