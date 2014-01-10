/*
 * Modulator.hpp
 *
 *  Created on: 24 déc. 2013
 *      Author: Robotique
 */

#ifndef MODULATOR_HPP_
#define MODULATOR_HPP_

#include <arch/app/include/Process.hpp>
#include <hw/pwm/include/Pwm.hpp>
#include <math/include/Vector3.hpp>

namespace autom {

template <int8_t NB_MOTORS>
class Modulator : public infra::Process {
public:

	Modulator(
			/* Input */
			const ::math::Vector3f& torque_B,
			const ::math::Vector3f& force_B,
			/* Output */
			hw::Pwm::Input& out);
	virtual ~Modulator();

	/** @brief Init the process */
	virtual ::infra::status initialize();

	/** @brief Execute the process */
	virtual ::infra::status execute() = 0;

protected:

	/** @brief Demanded torque */
	const ::math::Vector3f& _torque_B;

	/** @brief Demanded force */
	const ::math::Vector3f& _force_B;

	/** @brief Pwm */
	hw::Pwm::Input& _out;
};


template <int8_t NB_MOTORS>
Modulator<NB_MOTORS>::Modulator(
		/* Input */
		const ::math::Vector3f& torque_B,
		const ::math::Vector3f& force_B,
		/* Output */
		hw::Pwm::Input& out)
: Process(),
  _torque_B(torque_B),
  _force_B(force_B),
  _out(out)
{
	// TODO Auto-generated constructor stub

}

template <int8_t NB_MOTORS>
Modulator<NB_MOTORS>::~Modulator() {
	// TODO Auto-generated destructor stub
}


/** @brief Init the process */
template <int8_t NB_MOTORS>
::infra::status Modulator<NB_MOTORS>::initialize()
{
	uint8_t iMotor;

	for (iMotor=0 ; iMotor<NB_MOTORS ; iMotor++)
	{
		_out.channels[iMotor] = MIN_PULSEWIDTH;
		board::Board::board.getPwm().enable_out(iMotor);
	}

	return 0;
}

} /* namespace autom */

#endif /* MODULATOR_HPP_ */
