/*
 * Channel.hpp
 *
 *  Created on: 1 juin 2013
 *      Author: Aberzen
 */

#ifndef CHANNEL_HPP_
#define CHANNEL_HPP_

#include "mavlink_bridge.hpp"

namespace mavlink {

class Channel {
	friend void ::comm_send_ch(mavlink_channel_t chan, uint8_t ch);
public:
	static Channel* getChannel(mavlink_channel_t chan);

protected:
	static Channel* _channels[MAVLINK_COMM_3+1];
	static void setChannel(mavlink_channel_t chan, Channel* channel);

public:
	Channel(mavlink_channel_t chan);
	virtual ~Channel();

	virtual bool receiveMessage(mavlink_message_t** r_message) = 0;
	virtual bool sendMessage(const uint8_t* buffer, size_t len) = 0;
	virtual bool canSend(size_t len) = 0;
	mavlink_status_t* getStatus();

protected:
	mavlink_channel_t _chan;
	mavlink_message_t _r_message;
	mavlink_status_t _r_mavlink_status;

	uint8_t parseChar(uint8_t c);

};

} /* namespace mavlink */
#endif /* CHANNEL_HPP_ */
