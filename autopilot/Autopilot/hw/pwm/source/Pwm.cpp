/*
 *       APM_RC_.cpp - Radio Control Library for Ardupilot Mega. Arduino
 *       Code by Jordi Muñoz and Jose Julio. DIYDrones.com
 *
 */
#include "../include/Pwm.hpp"


namespace hw {

Pwm::Pwm(
			/* Inputs */
			const Pwm::Input& in,
			/* Outputs */
			Pwm::Output& out
			)
: Driver(),
  _in(in),
  _out(out)
{

}

} /* namespace hw */

