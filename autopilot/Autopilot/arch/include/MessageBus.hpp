/*
 * MessageBus.h
 *
 *  Created on: 8 janv. 2013
 *      Author: Aberzen
 */

#ifndef MESSAGEBUS_H_
#define MESSAGEBUS_H_

#include "MessageListener.hpp"

namespace arducopter {

class MessageBus {
public:
	MessageBus();
	virtual ~MessageBus() {
	}
};

} /* namespace arducopter */
#endif /* MESSAGEBUS_H_ */
