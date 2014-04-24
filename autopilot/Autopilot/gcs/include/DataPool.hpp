/*
 * DataPool.hpp
 *
 *  Created on: 2 juin 2013
 *      Author: Aberzen
 */

#ifndef DATAPOOL_HPP_
#define DATAPOOL_HPP_

#include <math/include/Quaternion.hpp>

namespace mavlink {

class DataPool {
public:
	DataPool();
	virtual ~DataPool();

public:

	float roll; ///< Roll angle (rad, -pi..+pi)
	float pitch; ///< Pitch angle (rad, -pi..+pi)
	float yaw; ///< Yaw angle (rad, -pi..+pi)
	float rollspeed; ///< Roll angular speed (rad/s)
	float pitchspeed; ///< Pitch angular speed (rad/s)
	float yawspeed; ///< Yaw angular speed (rad/s)
	math::Quaternion _estQuat_BI; ///< Estimated attitude
	char key[32]; ///< key
	int32_t current_consumed; ///< Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
	int32_t energy_consumed; ///< Consumed energy, in 100*Joules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
	uint16_t voltage_cell_1; ///< Battery voltage of cell 1, in millivolts (1 = 1 millivolt)
	uint16_t voltage_cell_2; ///< Battery voltage of cell 2, in millivolts (1 = 1 millivolt), -1: no cell
	uint16_t voltage_cell_3; ///< Battery voltage of cell 3, in millivolts (1 = 1 millivolt), -1: no cell
	uint16_t voltage_cell_4; ///< Battery voltage of cell 4, in millivolts (1 = 1 millivolt), -1: no cell
	uint16_t voltage_cell_5; ///< Battery voltage of cell 5, in millivolts (1 = 1 millivolt), -1: no cell
	uint16_t voltage_cell_6; ///< Battery voltage of cell 6, in millivolts (1 = 1 millivolt), -1: no cell
	int16_t current_battery; ///< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
	uint8_t accu_id; ///< Accupack ID
	int8_t battery_remaining; ///< Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery


	math::Vector3f _rawMag_B;
	math::Vector3f _rawImuAcc_B;
	math::Vector3f _rawImuRate_B;
	float _rawBaroPressure;
	float _rawBaroTemperature;

	math::Vector3f _estVel_B;
	math::Vector3f _estPos_B;
	math::Vector3f _estRate_B;

};

} /* namespace mavlink */
#endif /* DATAPOOL_HPP_ */
