/*
 * nrd.h
 *
 *  Created on: 17 janv. 2013
 *      Author: Aberzen
 */

#include <math/include/Matrix3.hpp>

#ifndef NRD_H_
#define NRD_H_

/* **************************************************** */
/* Physics */

#define PHYSICS_GRAVITY (9.81) ///<@ Gravity constant
extern math::Vector3f g_I;

/* **************************************************** */
/* Hardware configuration */
#define IMU_MPU6000_SPI_CS_PIN (53)
#define IMU_MPU6000_GYR_CNF (hw::HalImuMpu6000::E_GYR_CNF_2000DPS)
#define IMU_MPU6000_ACC_CNF (hw::HalImuMpu6000::E_ACC_CNF_4G)
#define IMU_MPU6000_FREQ_CNF (hw::HalImuMpu6000::E_UPT_FREQ_100HZ)

#define BARO_MS5611_SPI_CS_PIN (40)

/* Pre processing configuration */
extern math::Matrix3f cnfImuRateMat_UB;
extern math::Vector3f cnfImuRateBias_B;
extern math::Matrix3f cnfImuAccMat_UB;
extern math::Vector3f cnfImuAccBias_B;
extern math::Matrix3f cnfMagMat_UB;
extern math::Vector3f cnfMagBias_B;

/* **************************************************** */
/* Global FSW parameter */

#define FSW_SYSTEM_TICK_SEC			(0.001) /* s */
#define FSW_SYSTEM_TICK_MSEC		(FSW_SYSTEM_TICK_SEC*1000.) 	/* ms */
#define FSW_TASK_CTRL_PERIOD_TICK	(10)	/* tick/period */
#define FSW_TASK_CTRL_PERIOD_SEC	(FSW_TASK_CTRL_PERIOD_TICK*FSW_SYSTEM_TICK_SEC)	/* period/s */
#define FSW_TASK_CTRL_PERIOD_MSEC	(FSW_TASK_CTRL_PERIOD_SEC*1000.)	/* period/ms */


/* **************************************************** */
/* Modulator parameters */
#define MODULATOR_NB_MOTORS		4

/* **************************************************** */
/* Attitude controller: angular rate only */

#define CTRL_ATT_ANGRATE_X_dt		(0.02)
#define CTRL_ATT_ANGRATE_X_Kp		(0)
#define CTRL_ATT_ANGRATE_X_Kd		(0)
#define CTRL_ATT_ANGRATE_X_Ki		(0)
#define CTRL_ATT_ANGRATE_X_maxI		(0)
#define CTRL_ATT_ANGRATE_X_useOfRb	(false)
#define CTRL_ATT_ANGRATE_X_Krb		(0)
#define CTRL_ATT_ANGRATE_X_rbThd	(0)
#define CTRL_ATT_ANGRATE_X_rb		(0)

#define CTRL_ATT_ANGRATE_Y_dt		(0.02)
#define CTRL_ATT_ANGRATE_Y_Kp		(0)
#define CTRL_ATT_ANGRATE_Y_Kd		(0)
#define CTRL_ATT_ANGRATE_Y_Ki		(0)
#define CTRL_ATT_ANGRATE_Y_maxI		(0)
#define CTRL_ATT_ANGRATE_Y_useOfRb	(false)
#define CTRL_ATT_ANGRATE_Y_Krb		(0)
#define CTRL_ATT_ANGRATE_Y_rbThd	(0)
#define CTRL_ATT_ANGRATE_Y_rb		(0)

#define CTRL_ATT_ANGRATE_Z_dt		(0.02)
#define CTRL_ATT_ANGRATE_Z_Kp		(0)
#define CTRL_ATT_ANGRATE_Z_Kd		(0)
#define CTRL_ATT_ANGRATE_Z_Ki		(0)
#define CTRL_ATT_ANGRATE_Z_maxI		(0)
#define CTRL_ATT_ANGRATE_Z_useOfRb	(false)
#define CTRL_ATT_ANGRATE_Z_Krb		(0)
#define CTRL_ATT_ANGRATE_Z_rbThd	(0)
#define CTRL_ATT_ANGRATE_Z_rb		(0)

/* **************************************************** */
/* Attitude controller: angular attitude with rate bias */

#define CTRL_ATT_ANGATT_X_dt		(0.02)
#define CTRL_ATT_ANGATT_X_Kp		(0)
#define CTRL_ATT_ANGATT_X_Kd		(0)
#define CTRL_ATT_ANGATT_X_Ki		(0)
#define CTRL_ATT_ANGATT_X_maxI		(0)
#define CTRL_ATT_ANGATT_X_useOfRb	(true)
#define CTRL_ATT_ANGATT_X_Krb		(0)
#define CTRL_ATT_ANGATT_X_rbThd		(0)
#define CTRL_ATT_ANGATT_X_rb		(0)

#define CTRL_ATT_ANGATT_Y_dt		(0.02)
#define CTRL_ATT_ANGATT_Y_Kp		(0)
#define CTRL_ATT_ANGATT_Y_Kd		(0)
#define CTRL_ATT_ANGATT_Y_Ki		(0)
#define CTRL_ATT_ANGATT_Y_maxI		(0)
#define CTRL_ATT_ANGATT_Y_useOfRb	(true)
#define CTRL_ATT_ANGATT_Y_Krb		(0)
#define CTRL_ATT_ANGATT_Y_rbThd		(0)
#define CTRL_ATT_ANGATT_Y_rb		(0)

#define CTRL_ATT_ANGATT_Z_dt		(0.02)
#define CTRL_ATT_ANGATT_Z_Kp		(0)
#define CTRL_ATT_ANGATT_Z_Kd		(0)
#define CTRL_ATT_ANGATT_Z_Ki		(0)
#define CTRL_ATT_ANGATT_Z_maxI		(0)
#define CTRL_ATT_ANGATT_Z_useOfRb	(true)
#define CTRL_ATT_ANGATT_Z_Krb		(0)
#define CTRL_ATT_ANGATT_Z_rbThd		(0)
#define CTRL_ATT_ANGATT_Z_rb		(0)


#endif /* NRD_H_ */
