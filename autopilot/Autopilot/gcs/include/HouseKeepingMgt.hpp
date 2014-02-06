/*
 * HouseKeepingMgt.hpp
 *
 *  Created on: 7 juin 2013
 *      Author: Aberzen
 */

#ifndef HOUSEKEEPINGMGT_HPP_
#define HOUSEKEEPINGMGT_HPP_

#include <common/mavlink.h>
#include "RawSensorStream.hpp"
#include "EstimatorStream.hpp"

namespace mavlink {


class HouseKeepingMgt : public infra::Process {
public:
	HouseKeepingMgt(mavlink_channel_t port,
			const autom::Estimator::Estimations& est,
			const board::Board::Measurements& meas,
			const board::Board::RawMeasurements& rawMeas);
	virtual ~HouseKeepingMgt();

	void setDataStream(
			uint16_t req_message_rate,
			uint8_t req_stream_id,
			uint8_t start_stop
			);

	/** @brief Update periodic */
	void update();

	/** @brief Init the process */
	virtual void initialize();

	/** @brief Execute the process */
	virtual void execute();


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
