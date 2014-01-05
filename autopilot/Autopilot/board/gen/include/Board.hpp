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
public:
	Board();
	virtual ~Board();

	/** @brief Init the process */
	virtual infra::status initialize();

	/** @brief Execute the process */
	virtual infra::status execute();

	/** @brief Get IMU */
	virtual inline hw::HalImu& getImu() = 0;

	/** @brief Get Baro */
	virtual inline hw::HalBarometer& getBaro() = 0;

	/** @brief Get Magnetometer */
	virtual inline hw::HalMagnetometer& getCompass() = 0;

	/** @brief Get Gps */
	virtual inline hw::Gps& getGps() = 0;

	/** @brief Get Pwm */
	virtual inline hw::Pwm& getPwm() = 0;

public:
	/* Unique static board */
	static Board& board;

public:
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
