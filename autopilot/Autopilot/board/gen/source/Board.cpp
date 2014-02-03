/*
 * Board.cpp
 *
 *  Created on: 20 mars 2013
 *      Author: Aberzen
 */

#include "../include/Board.hpp"

namespace board {

Board::Board(
		hw::HalImu& imu,
		hw::HalBarometer& baro,
		hw::HalMagnetometer& compass,
		hw::Gps& gps,
		hw::Pwm& pwm
		)
: imu(imu),
  baro(baro),
  compass(compass),
  gps(gps),
  pwm(pwm)
{
}

Board::~Board() {
}

/** @brief Init the process */
infra::status Board::initialize()
{
	infra::status result;

	result = imu.initialize();
	if (result < 0)
		return result;

	result = compass.initialize();
	if (result < 0)
		return result;

	result = baro.initialize();
	if (result < 0)
		return result;

	result = pwm.initialize();
	if (result < 0)
		return result;

	return 0;
}

/** @brief Execute the process */
infra::status Board::execute()
{
	infra::status result;

	result = imu.execute();
	if (result < 0)
		return result;

	result = compass.execute();
	if (result < 0)
		return result;

	result = baro.execute();
	if (result < 0)
		return result;

	result = pwm.execute();
	if (result < 0)
		return result;

	return 0;
}


} /* namespace board */
