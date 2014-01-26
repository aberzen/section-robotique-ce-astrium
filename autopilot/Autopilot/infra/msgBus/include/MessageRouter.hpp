/*
 * MessageRouter.h
 *
 *  Created on: 9 janv. 2013
 *      Author: Aberzen
 */

#ifndef MESSAGEROUTER_H_
#define MESSAGEROUTER_H_

#include "Message.hpp"
#include <infra/include/Queue.hpp>

namespace arducopter {

typedef enum EMessageRouterStatus {
	E_MESSAGEROUTER_STATUS_OK=0,
	E_MESSAGEROUTER_STATUS_UNKDEST,
} MessageRouterStatus ;

class MessageRouter {
public:
	MessageRouter();
	virtual ~MessageRouter();

	virtual MessageRouterStatus routeMessage(Message *msg);

};

} /* namespace arducopter */
#endif /* MESSAGEROUTER_H_ */
