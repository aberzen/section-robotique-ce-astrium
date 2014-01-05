/*
 * CommandMgt.hpp
 *
 *  Created on: 7 juin 2013
 *      Author: Aberzen
 */

#ifndef COMMANDMGT_HPP_
#define COMMANDMGT_HPP_

#include <stdint.h>

namespace mavlink {

class CommandMgt {
public:
	CommandMgt();
	virtual ~CommandMgt();

	void cmd(
					float param1,
					float param2,
					float param3,
					float param4,
					float param5,
					float param6,
					float param7,
					uint16_t command,
					uint8_t confirmation
			);
};

} /* namespace mavlink */
#endif /* COMMANDMGT_HPP_ */
