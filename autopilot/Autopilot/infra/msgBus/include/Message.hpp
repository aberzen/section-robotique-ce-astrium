/*
 * Message.h
 *
 *  Created on: 8 janv. 2013
 *      Author: Aberzen
 */

#ifndef MESSAGE_H_
#define MESSAGE_H_

#include <inttypes.h>

namespace arducopter {

#define MESSAGE_TYPE_BITS	(1)
#define MESSAGE_SENDER_BITS	(5)
#define MESSAGE_DEST_BITS	(5)
#define MESSAGE_SVC_BITS	(5)

#define MESSAGE_MAX_TYPE_CNT	(1<<MESSAGE_TYPE_BITS)
#define MESSAGE_MAX_SENDER_CNT	(1<<MESSAGE_SENDER_BITS)
#define MESSAGE_MAX_DEST_CNT	(1<<MESSAGE_DEST_BITS)
#define MESSAGE_MAX_SVC_CNT		(1<<MESSAGE_SVC_BITS)

#define HEADER_GET_TYPE(header) \
	(((header)>>15) & (0x0001))

#define HEADER_GET_SENDER(header) \
	(((header)>>10) & (0x001F))

#define HEADER_GET_DEST(header) \
	(((header)>>5) & (0x001F))

#define HEADER_GET_SVC(header) \
	((header)>>0) & (0x001F))


#define HEADER_SET_TYPE(type) \
	(((type) & (0x0001))<<15)

#define HEADER_SET_SENDER(senderId) \
	(((senderId) & (0x001F))<<10)

#define HEADER_SET_DEST(destId) \
	(((destId) & (0x001F))<<5)

#define HEADER_SET_SVC(svcId) \
	(((svcId) & (0x001F))<<0)


class Message {
public:
	Message();
	Message(bool isRequest, uint16_t senderId, uint16_t destId, uint16_t svcId);
	virtual ~Message();

	/** @brief set message header */
	inline void setHeader(bool isRequest, uint16_t senderId, uint16_t destId, uint16_t svcId);

	/** @brief is the message a request */
	inline bool isRequest(void);
	/** @brief retrieve the sender ID of the message */
	inline uint16_t getSenderId(void);
	/** @brief retrieve the destination ID of the message */
	inline uint16_t getDestId(void);
	/** @brief retrieve the service ID of the message */
	inline uint16_t getSvcId(void);

protected:
	uint16_t _header;
};

/** @brief set message header */
inline void Message::setHeader(bool isRequest, uint16_t senderId, uint16_t destId, uint16_t svcId){
	this->_header = ((isRequest& 0x0001)<<15) | ((senderId& 0x001F)<<10) | ((destId& 0x001F)<<5) | ((svcId& 0x001F)<<0);
}

/** @brief is the message a request */
inline bool Message::isRequest(void){
	return ((this->_header)>>15) & 0x0001;
}
/** @brief retrieve the sender ID of the message */
inline uint16_t Message::getSenderId(void){
	return ((this->_header)>>10) & 0x001F;
}
/** @brief retrieve the destination ID of the message */
inline uint16_t Message::getDestId(void){
	return ((this->_header)>> 5) & 0x001F;
}
/** @brief retrieve the service ID of the message */
inline uint16_t Message::getSvcId(void){
	return ((this->_header)>> 0) & 0x001F;
}


} /* namespace arducopter */
#endif /* MESSAGE_H_ */
