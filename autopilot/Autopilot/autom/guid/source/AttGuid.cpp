/*
 * AttGuid.cpp
 *
 *  Created on: 10 juin 2013
 *      Author: Aberzen
 */

#include "../include/AttGuid.hpp"

namespace autom {

AttGuid::AttGuid(
		/* Inputs */
		const hw::Pwm::Output& rc,
		const autom::NavGuid::Output& nav,
		const ::math::Vector3f& frcDem_I,
		/* Outputs */
		Output& out) :
Process(),
_nav(nav),
_frcDem_I(frcDem_I),
_rc(rc),
_out(out)
{
}

} /* namespace autom */
