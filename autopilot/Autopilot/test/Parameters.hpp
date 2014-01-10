/*
 * Parameters.hpp
 *
 *  Created on: 20 mai 2013
 *      Author: Aberzen
 */

#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

#include <avr/pgmspace.h>
#include <gcs/include/ParameterMgt.hpp>

namespace test {

//#define CONFIG_PARAMETERS_COUNT	185
#define CONFIG_PARAMETERS_COUNT	4
extern PROGMEM const mavlink::ParameterMgt::ParamInfo config[CONFIG_PARAMETERS_COUNT];

} /* namespace test */
#endif /* PARAMETERS_HPP_ */
