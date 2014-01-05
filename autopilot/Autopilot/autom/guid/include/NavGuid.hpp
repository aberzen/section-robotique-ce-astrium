/*
 * NavGuid.hpp
 *
 *  Created on: 10 juin 2013
 *      Author: Aberzen
 */

#ifndef NAVGUID_HPP_
#define NAVGUID_HPP_

#include <arch/app/include/Process.hpp>
#include <math/include/Vector3.hpp>
#include <hw/pwm/include/Pwm.hpp>

namespace autom {

class NavGuid : public infra::Process {
public:
	typedef struct
	{
		::math::Vector3f posDem_I;
		::math::Vector3f velDem_I;
	} Output;
public:
	NavGuid(
			/* Inputs */
			const hw::Pwm::Output& rc,
			/* Outputs */
			Output& out
);
	virtual ~NavGuid() {
		// TODO Auto-generated destructor stub
	}
protected:
	/** @brief Radio command */
	const hw::Pwm::Output& _rc;
	/** @brief Demanded data (outputs) */
	Output& _out;
};

} /* namespace autom */
#endif /* NAVGUID_HPP_ */
