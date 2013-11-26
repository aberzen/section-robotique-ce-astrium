/*
 * HouseKeepingMgt.hpp
 *
 *  Created on: 7 juin 2013
 *      Author: Aberzen
 */

#ifndef HOUSEKEEPINGMGT_HPP_
#define HOUSEKEEPINGMGT_HPP_

#include <mavlink/v1.0/common/mavlink.h>
#include "RawSensorStream.hpp"
#include "EstimatorStream.hpp"

namespace mavlink {


class HouseKeepingMgt : public arch::Process {
public:
	HouseKeepingMgt(mavlink_channel_t port);
	virtual ~HouseKeepingMgt();

	void setDataStream(
			uint16_t req_message_rate,
			uint8_t req_stream_id,
			uint8_t start_stop
			);

	/** @brief Update periodic */
	void update();

	/** @brief Init the process */
	virtual status initialize();

	/** @brief Execute the process */
	virtual status execute();


//protected:
	/** @brief Mavlink port to be used */
	mavlink_channel_t _port;

	/** @brief Raw Sensors Data Stream */
	RawSensorStream _rawSensorsStream;

	/** @brief Estimator Data Stream */
	EstimatorStream _estStream;
};



} /* namespace mavlink */
#endif /* HOUSEKEEPINGMGT_HPP_ */
