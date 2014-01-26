/*
 * Message.cpp
 *
 *  Created on: 8 janv. 2013
 *      Author: Aberzen
 */

#include "../include/Message.hpp"

namespace arducopter {


Message::Message() :
		_header(0x0000)
{
}

Message::Message(bool isRequest, uint16_t senderId, uint16_t destId, uint16_t svcId) :
	_header(HEADER_SET_TYPE(isRequest) | HEADER_SET_SENDER(senderId) | HEADER_SET_DEST(destId) | HEADER_SET_SVC(svcId))
{
}

Message::~Message()
{
}


} /* namespace arducopter */
