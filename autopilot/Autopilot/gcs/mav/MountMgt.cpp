/*
 * MountMgt.cpp
 *
 *  Created on: 5 juin 2013
 *      Author: Aberzen
 */

#include "../include/MountMgt.hpp"

namespace mavlink {

MountMgt::MountMgt() {
}


MountMgt::~MountMgt() {
}


void MountMgt::configure(
		uint8_t mount_mode,
		uint8_t stab_roll,
		uint8_t stab_pitch,
		uint8_t stab_yaw
)
{

}

void MountMgt::control(
		int32_t input_a,
		int32_t input_b,
		int32_t input_c,
		uint8_t save_position
)
{

}


void MountMgt::status(
		int32_t pointing_a,
		int32_t pointing_b,
		int32_t pointing_c
)
{

}


} /* namespace mavlink */
