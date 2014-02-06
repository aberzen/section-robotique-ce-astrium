/*
 * Board.hpp
 *
 *  Created on: 20 mars 2013
 *      Author: Aberzen
 */

#ifndef BOARD_HPP_
#define BOARD_HPP_

#include <hw/imu/include/HalImu.hpp>
#include <hw/mag/include/HalMagnetometer.hpp>
#include <hw/baro/include/HalBarometer.hpp>
#include <hw/gps/include/Gps.hpp>
#include <hw/pwm/include/Pwm.hpp>

namespace board {

class Board {
public:
	typedef struct {
		::hw::HalImu::Output imu;
		::hw::HalMagnetometer::Output compass;
		::hw::HalBarometer::Output baro;
		::hw::Gps::Output gps;
	} Measurements;
	typedef struct {
		::hw::HalImu::RawOutput imu;
		::hw::HalMagnetometer::RawOutput compass;
		::hw::HalBarometer::RawOutput baro;
		::hw::Gps::RawOutput gps;
	} RawMeasurements;

protected:
	Board(
			hw::HalImu& imu,
			hw::HalBarometer& baro,
			hw::HalMagnetometer& compass,
			hw::Gps& gps,
			hw::Pwm& pwm
			);

public:
	virtual ~Board();

	/** @brief Init the process */
	virtual void initialize();

	/** @brief Execute the process */
	virtual void execute();

public:

	/** @brief Get IMU */
	hw::HalImu& imu;

	/** @brief Get Baro */
	hw::HalBarometer& baro;

	/** @brief Get Magnetometer */
	hw::HalMagnetometer& compass;

	/** @brief Get Gps */
	hw::Gps& gps;

	/** @brief Get Pwm */
	hw::Pwm& pwm;

	/** @brief Measurements */
	Measurements meas;

	/** @brief Measurements */
	RawMeasurements rawMeas;

	/** @brief Pwm Input */
	hw::Pwm::Input pwmVal;

	/** @brief Pwm Output */
	hw::Pwm::Output radio;
};


} /* namespace board */
#endif /* BOARD_HPP_ */
