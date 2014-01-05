/*
 * AttGuid.hpp
 *
 *  Created on: 10 juin 2013
 *      Author: Aberzen
 */

#ifndef ATTGUID_HPP_
#define ATTGUID_HPP_

#include <arch/app/include/Process.hpp>
#include <math/include/Quaternion.hpp>
#include <hw/pwm/include/Pwm.hpp>
#include <autom/guid/include/NavGuid.hpp>
#include <autom/ctrl/include/ControllerPid3Axes.hpp>
namespace autom {

class AttGuid : public infra::Process {
public:
	typedef struct
	{
		::math::Quaternion qDem_IB;
		::math::Vector3f angRateDem_B;
	} Output;
public:
	AttGuid(
			/* Inputs */
			const hw::Pwm::Output& rc,
			const autom::NavGuid::Output& nav,
			const ::math::Vector3f& frcDem_I,
			/* Outputs */
			Output& out
			);
	virtual ~AttGuid() {
		// TODO Auto-generated destructor stub
	}
protected:
	/** @brief Output of navigation guidance */
	const autom::NavGuid::Output& _nav;

	/** @brief Demanded force in inertial frame (output of the navigation controller */
	const ::math::Vector3f& _frcDem_I;

	/** @brief RC */
	const hw::Pwm::Output& _rc;

	/** @brief Demanded attitude (output) */
	Output& _out;

};

} /* namespace autom */
#endif /* ATTGUID_HPP_ */
