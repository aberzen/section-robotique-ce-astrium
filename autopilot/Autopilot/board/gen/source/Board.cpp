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
void Board::initialize()
{
	imu.initialize();
	compass.initialize();
	baro.initialize();
	pwm.initialize();
}

/** @brief Execute the process */
void Board::execute()
{
	imu.execute();
	compass.execute();
	baro.execute();
	pwm.execute();
}


} /* namespace board */
