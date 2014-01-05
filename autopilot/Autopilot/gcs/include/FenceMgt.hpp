/*
 * FenceMgt.hpp
 *
 *  Created on: 5 juin 2013
 *      Author: Aberzen
 */

#ifndef FENCEMGT_HPP_
#define FENCEMGT_HPP_

#include <stdint.h>

namespace mavlink {

class FenceMgt {
public:
	FenceMgt();
	virtual ~FenceMgt();

	void setPoint(
			float lat,
			float lng,
			uint8_t idx,
			uint8_t count
	);

	void fetchPoint(
			uint8_t idx
	);

};

} /* namespace mavlink */
#endif /* FENCEMGT_HPP_ */
