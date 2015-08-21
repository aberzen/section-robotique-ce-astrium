/*
 * SerialChannel.hpp
 *
 *  Created on: 1 juin 2013
 *      Author: Aberzen
 */

#ifndef SERIALCHANNEL_HPP_
#define SERIALCHANNEL_HPP_

#include "Channel.hpp"
#include <hw/serial/Serial.hpp>

namespace mavlink {

class SerialChannel: public Channel {
public:
	SerialChannel(mavlink_channel_t chan, hw::Serial& serial);
	virtual ~SerialChannel();

	virtual bool receiveMessage(mavlink_message_t** r_message);
	virtual bool sendMessage(const uint8_t* buffer, size_t len);
	virtual bool canSend(size_t len);

protected:
	hw::Serial& _serial;
};

} /* namespace mavlink */
#endif /* SERIALCHANNEL_HPP_ */
