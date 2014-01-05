/*
 * NavGuid.cpp
 *
 *  Created on: 10 juin 2013
 *      Author: Aberzen
 */

#include <autom/guid/include/NavGuid.hpp>

namespace autom {

NavGuid::NavGuid(
		/* Inputs */
		const hw::Pwm::Output& rc,
		/* Outputs */
		Output& out
) :
_out(out),
_rc(rc)
{
	// TODO Auto-generated constructor stub

}

} /* namespace autom */
