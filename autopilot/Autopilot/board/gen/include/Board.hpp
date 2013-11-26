/*
 * Board.hpp
 *
 *  Created on: 20 mars 2013
 *      Author: Aberzen
 */

#ifndef BOARD_HPP_
#define BOARD_HPP_

#include <arch/include/Process.hpp>

#include <hw/imu/include/HalImu.hpp>
#include <hw/mag/include/HalMagnetometer.hpp>
#include <sensor/imu/include/Imu.hpp>
#include <sensor/baro/include/Barometer.hpp>
#include <sensor/mag/include/Magnetometer.hpp>
#include <sensor/gps/include/Gps.hpp>
#include <sensor/sonar/include/Sonar.hpp>

namespace board {

class Board : arch::Process {
public:
	Board();
	virtual ~Board();

	/** @brief Init the process */
	virtual status initialize();

	/** @brief Execute the process */
	virtual status execute();

	/** @brief Get IMU HAL */
	virtual inline hw::HalImu& getHalImu() = 0;

	/** @brief Get Magnetometer sensor */
	virtual inline hw::HalMagnetometer& getHalMagnetometer() = 0;

	/** @brief Get IMU sensor */
	virtual inline sensor::Imu& getImu() = 0;

	/** @brief Get Barometer sensor */
	virtual inline sensor::Barometer& getBarometer() = 0;

	/** @brief Get Gps sensor */
	virtual inline sensor::Gps& getGps() = 0;

	/** @brief Get Sonar sensor */
	virtual inline sensor::Sonar& getSonar() = 0;

	/** @brief Get Magnetometer sensor */
	virtual inline sensor::Magnetometer& getMagnetometer() = 0;

//	/** @brief Get Led1 sensor */
//	virtual inline sensor::Led& getLed1() = 0;
//	static Board& board;
};


} /* namespace board */
#endif /* BOARD_HPP_ */
