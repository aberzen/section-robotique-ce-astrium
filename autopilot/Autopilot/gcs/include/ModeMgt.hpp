/*
 * ModeMgt.hpp
 *
 *  Created on: 5 juin 2013
 *      Author: Aberzen
 */

#ifndef MODEMGT_HPP_
#define MODEMGT_HPP_

#include <stdint.h>

namespace mavlink {

class ModeMgt {
public:
	ModeMgt();
	virtual ~ModeMgt();

	void setMode(
			uint32_t custom_mode,
			uint8_t base_mode);
};

} /* namespace mavlink */
#endif /* MODEMGT_HPP_ */
