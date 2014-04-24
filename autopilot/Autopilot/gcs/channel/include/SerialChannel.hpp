/*
 * SerialChannel.hpp
 *
 *  Created on: 1 juin 2013
 *      Author: Aberzen
 */

#ifndef SERIALCHANNEL_HPP_
#define SERIALCHANNEL_HPP_

#include "Channel.hpp"
#include <hw/serial/include/FastSerial.hpp>

namespace mavlink {

class SerialChannel: public Channel {
public:
	SerialChannel(mavlink_channel_t chan, FastSerial& serial);
	virtual ~SerialChannel();

	virtual bool receiveMessage(mavlink_message_t* r_message);

protected:
	FastSerial& _serial;
	virtual void sendChar(uint8_t ch);

};

} /* namespace mavlink */
#endif /* SERIALCHANNEL_HPP_ */
