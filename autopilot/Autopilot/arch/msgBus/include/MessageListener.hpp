/*
 * MessageListener.h
 *
 *  Created on: 8 janv. 2013
 *      Author: Aberzen
 */

#ifndef MESSAGELISTENER_H_
#define MESSAGELISTENER_H_

#include "Message.hpp"

namespace arducopter {


class MessageListener {
public:
	MessageListener();
	virtual ~MessageListener();

	virtual void processMessage(Message* message) = 0;
};

} /* namespace arducopter */
#endif /* MESSAGELISTENER_H_ */
