/*
 * DigivcamMgt.cpp
 *
 *  Created on: 5 juin 2013
 *      Author: Aberzen
 */

#include "../include/DigivcamMgt.hpp"

namespace mavlink {

DigivcamMgt::DigivcamMgt() {
	// TODO Auto-generated constructor stub

}

DigivcamMgt::~DigivcamMgt() {
	// TODO Auto-generated constructor stub

}

void DigivcamMgt::configure(
		float extra_value,
		uint16_t shutter_speed,
		uint8_t mode,
		uint8_t aperture,
		uint8_t iso,
		uint8_t exposure_type,
		uint8_t command_id,
		uint8_t engine_cut_off,
		uint8_t extra_param
)
{

}


void DigivcamMgt::control(
		float extra_value,
		uint8_t session,
		uint8_t zoom_pos,
		int8_t zoom_step,
		uint8_t focus_lock,
		uint8_t shot,
		uint8_t command_id,
		uint8_t extra_param
)
{

}


} /* namespace mavlink */
