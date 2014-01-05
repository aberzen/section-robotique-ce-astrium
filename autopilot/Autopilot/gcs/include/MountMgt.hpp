/*
 * MountMgt.hpp
 *
 *  Created on: 5 juin 2013
 *      Author: Aberzen
 */

#ifndef MOUNTMGT_HPP_
#define MOUNTMGT_HPP_

#include <stdint.h>

namespace mavlink {

class MountMgt {
public:
	MountMgt();
	virtual ~MountMgt();

	void configure(
			uint8_t mount_mode,
			uint8_t stab_roll,
			uint8_t stab_pitch,
			uint8_t stab_yaw
	);

	void control(
			int32_t input_a,
			int32_t input_b,
			int32_t input_c,
			uint8_t save_position
	);

	void status(
			int32_t pointing_a,
			int32_t pointing_b,
			int32_t pointing_c
	);

};

} /* namespace mavlink */
#endif /* MOUNTMGT_HPP_ */
