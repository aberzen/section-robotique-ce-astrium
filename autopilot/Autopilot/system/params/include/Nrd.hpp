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

#define PHYSICS_GRAVITY 		(9.81) 	/* [m/s^2] gravity acceleration norm */
#define PHYSICS_MASS 			(1.600) /* [Kg] mass */
#define PHYSICS_INERTIA_XX 		(0.0229623) /* [Kg.m^2] Diagonal inertia XX */
#define PHYSICS_INERTIA_YY 		(0.0448513) /* [Kg.m^2] Diagonal inertia YY */
#define PHYSICS_INERTIA_ZZ 		(0.0617154) /* [Kg.m^2] Diagonal inertia ZZ */

/* **************************************************** */
/* Hardware configuration */
#define IMU_MPU6000_SPI_CS_PIN 	(53)
#define IMU_MPU6000_GYR_CNF 	(hw::HalImuMpu6000::E_GYR_CNF_2000DPS)
#define IMU_MPU6000_ACC_CNF 	(hw::HalImuMpu6000::E_ACC_CNF_4G)
#define IMU_MPU6000_FREQ_CNF 	(hw::HalImuMpu6000::E_UPT_FREQ_100HZ)

#define BARO_MS5611_SPI_CS_PIN 	(40)

/* **************************************************** */
/* Global FSW parameter */

#define FSW_SYSTEM_TICK_SEC			(0.001) /* s */
#define FSW_SYSTEM_TICK_MSEC		(FSW_SYSTEM_TICK_SEC*1000.) 	/* ms */
#define FSW_TASK_CTRL_PERIOD_TICK	(10)	/* tick/period */
#define FSW_TASK_CTRL_PERIOD_SEC	(FSW_TASK_CTRL_PERIOD_TICK*FSW_SYSTEM_TICK_SEC)	/* period/s */
#define FSW_TASK_CTRL_PERIOD_MSEC	(FSW_TASK_CTRL_PERIOD_SEC*1000.)	/* period/ms */

/* **************************************************** */
/* Attitude controller: angular attitude with rate bias */

#define CTRL_ATT_X_Kp		(0.6295249)
#define CTRL_ATT_X_Kd		(0.1683226)
#define CTRL_ATT_X_Ki		(0.)
#define CTRL_ATT_X_maxI		(0.)
#define CTRL_ATT_X_useOfRb	(false)
#define CTRL_ATT_X_Krb		(0.)
#define CTRL_ATT_X_rbThd	(0.)
#define CTRL_ATT_X_rb		(0.)

#define CTRL_ATT_Y_Kp		(1.229625)
#define CTRL_ATT_Y_Kd		(0.3287775)
#define CTRL_ATT_Y_Ki		(0.)
#define CTRL_ATT_Y_maxI		(0.)
#define CTRL_ATT_Y_useOfRb	(false)
#define CTRL_ATT_Y_Krb		(0.)
#define CTRL_ATT_Y_rbThd	(0.)
#define CTRL_ATT_Y_rb		(0.)

#define CTRL_ATT_Z_Kp		(0.2707139)
#define CTRL_ATT_Z_Kd		(0.1809589)
#define CTRL_ATT_Z_Ki		(0.)
#define CTRL_ATT_Z_maxI		(0.)
#define CTRL_ATT_Z_useOfRb	(false)
#define CTRL_ATT_Z_Krb		(0.)
#define CTRL_ATT_Z_rbThd	(0.)
#define CTRL_ATT_Z_rb		(0.)

/* **************************************************** */
/* Filter: coeff for attitude filter */
#define FLT_ATTX_NUM_0		(  1.7106918)
#define FLT_ATTX_NUM_1		(- 1.4874214)
#define FLT_ATTX_DEN_0		(  1.0000000)
#define FLT_ATTX_DEN_1		(- 0.7767296)
#define FLT_ATTX_NUM		{FLT_ATTX_NUM_0, FLT_ATTX_NUM_1}
#define FLT_ATTX_DEN		{FLT_ATTX_DEN_0, FLT_ATTX_DEN_1}

#define FLT_ATTY_NUM_0		(  1.7106918)
#define FLT_ATTY_NUM_1		(- 1.4874214)
#define FLT_ATTY_DEN_0		(  1.0000000)
#define FLT_ATTY_DEN_1		(- 0.7767296)
#define FLT_ATTY_NUM		{FLT_ATTY_NUM_0, FLT_ATTY_NUM_1}
#define FLT_ATTY_DEN		{FLT_ATTY_DEN_0, FLT_ATTY_DEN_1}

#define FLT_ATTZ_NUM_0		(  1.0000000)
#define FLT_ATTZ_NUM_1		(  0.0000000)
#define FLT_ATTZ_DEN_0		(  1.0000000)
#define FLT_ATTZ_DEN_1		(  0.0000000)
#define FLT_ATTZ_NUM		{FLT_ATTZ_NUM_0, FLT_ATTZ_NUM_1}
#define FLT_ATTZ_DEN		{FLT_ATTZ_DEN_0, FLT_ATTZ_DEN_1}


/* **************************************************** */
/* Estimator: gain and other parameters for Kalman filter */

#define EST_KALMAN_GAIN_ACCO	(0.05)  /* gainAcco */
#define EST_KALMAN_GAIN_COMPASS	(0.005) /* gainCompass */

/* **************************************************** */
/* IMU calibration: parameters for IMU calibration */

#define PROC_CALIBIMU_FILT_NUM0		( 0.002493765586) 	/* Numerator of the filter */
#define PROC_CALIBIMU_FILT_NUM1		( 0.002493765586) 	/* Numerator of the filter */
#define PROC_CALIBIMU_FILT_NUM		{PROC_CALIBIMU_FILT_NUM0, PROC_CALIBIMU_FILT_NUM1} 	/* Numerator of the filter */
#define PROC_CALIBIMU_FILT_DEN0		( 1.000000000000) 	/* Numerator of the filter */
#define PROC_CALIBIMU_FILT_DEN1		(-0.995012468828) 	/* Numerator of the filter */
#define PROC_CALIBIMU_FILT_DEN		{PROC_CALIBIMU_FILT_DEN0, PROC_CALIBIMU_FILT_DEN1} /* Denominator of the filter */
#define PROC_CALIBIMU_NBMEAS_BIAS	(2000) /* Number of measurements for bias calibration */
#define PROC_CALIBIMU_NBMEAS_VAR	(50) /* Number of measurements for variance verification */
#define PROC_CALIBIMU_GYRO_VARTHD	(0.001) /* Variance of the gyro measurement */
#define PROC_CALIBIMU_ACCO_VARTHD	(0.002) /* Variance of the acco measurement */

/* **************************************************** */
/* Compass declination: parameters for IMU calibration */

#define PROC_COMPDEC_FILT_NUM0	( 0.01850082361) /* Numerator of the filter */
#define PROC_COMPDEC_FILT_NUM1	( 0.01850082361) /* Numerator of the filter */
#define PROC_COMPDEC_FILT_NUM	{PROC_COMPDEC_FILT_NUM0, PROC_COMPDEC_FILT_NUM1} /* Numerator of the filter */
#define PROC_COMPDEC_FILT_DEN0	( 1.00000000000) /* Denominator of the filter */
#define PROC_COMPDEC_FILT_DEN1	(-0.96299835278) /* Denominator of the filter */
#define PROC_COMPDEC_FILT_DEN	{PROC_COMPDEC_FILT_DEN0, PROC_COMPDEC_FILT_DEN1} /* Denominator of the filter */
#define PROC_COMPDEC_NBMEAS 	(500) /* Number of the measurement for calibration */

/* **************************************************** */
/* Radio Parameters                                     */

#define RC_CH1_MIN	960
#define RC_CH1_MAX	1960
#define RC_CH1_TRIM	1460

#define RC_CH2_MIN	960
#define RC_CH2_MAX	1960
#define RC_CH2_TRIM	1460

#define RC_CH3_MIN	960
#define RC_CH3_MAX	1960
#define RC_CH3_TRIM	1460

#define RC_CH4_MIN	960
#define RC_CH4_MAX	1960
#define RC_CH4_TRIM	1460

#define RC_CH5_MIN	1100
#define RC_CH5_MAX	1900
#define RC_CH5_TRIM	1500

#define RC_CH6_MIN	1100
#define RC_CH6_MAX	1900
#define RC_CH6_TRIM	1500

#define RC_CH7_MIN	1100
#define RC_CH7_MAX	1900
#define RC_CH7_TRIM	1500

#define RC_CH8_MIN	1100
#define RC_CH8_MAX	1900
#define RC_CH8_TRIM	1500


/* **************************************************** */
/* Modulator parameters: pseudo inverse based modulator */

#define MODULATOR_NB_MOTORS		4

#define MODULATOR_INFMAT_0_0	(  0.0000000)
#define MODULATOR_INFMAT_0_1	(  0.0000000)
#define MODULATOR_INFMAT_0_2	(  0.0000000)
#define MODULATOR_INFMAT_0_3	(  0.0000000)
#define MODULATOR_INFMAT_1_0	(  0.0000000)
#define MODULATOR_INFMAT_1_1	(  0.0000000)
#define MODULATOR_INFMAT_1_2	(  0.0000000)
#define MODULATOR_INFMAT_1_3	(  0.0000000)
#define MODULATOR_INFMAT_2_0	(  0.0064395)
#define MODULATOR_INFMAT_2_1	(  0.0064395)
#define MODULATOR_INFMAT_2_2	(  0.0064395)
#define MODULATOR_INFMAT_2_3	(  0.0064395)
#define MODULATOR_INFMAT_3_0	(- 0.0013362)
#define MODULATOR_INFMAT_3_1	(  0.0013845)
#define MODULATOR_INFMAT_3_2	(  0.0013362)
#define MODULATOR_INFMAT_3_3	(- 0.0013845)
#define MODULATOR_INFMAT_4_0	(- 0.0012581)
#define MODULATOR_INFMAT_4_1	(  0.0014381)
#define MODULATOR_INFMAT_4_2	(- 0.0012581)
#define MODULATOR_INFMAT_4_3	(  0.0014381)
#define MODULATOR_INFMAT_5_0	(- 0.0001163)
#define MODULATOR_INFMAT_5_1	(- 0.0001163)
#define MODULATOR_INFMAT_5_2	(  0.0001163)
#define MODULATOR_INFMAT_5_3	(  0.0001163)
#define MODULATOR_INFMAT_0		{MODULATOR_INFMAT_0_0, MODULATOR_INFMAT_0_1, MODULATOR_INFMAT_0_2, MODULATOR_INFMAT_0_3}
#define MODULATOR_INFMAT_1		{MODULATOR_INFMAT_1_0, MODULATOR_INFMAT_1_1, MODULATOR_INFMAT_1_2, MODULATOR_INFMAT_1_3}
#define MODULATOR_INFMAT_2		{MODULATOR_INFMAT_2_0, MODULATOR_INFMAT_2_1, MODULATOR_INFMAT_2_2, MODULATOR_INFMAT_2_3}
#define MODULATOR_INFMAT_3		{MODULATOR_INFMAT_3_0, MODULATOR_INFMAT_3_1, MODULATOR_INFMAT_3_2, MODULATOR_INFMAT_3_3}
#define MODULATOR_INFMAT_4		{MODULATOR_INFMAT_4_0, MODULATOR_INFMAT_4_1, MODULATOR_INFMAT_4_2, MODULATOR_INFMAT_4_3}
#define MODULATOR_INFMAT_5		{MODULATOR_INFMAT_5_0, MODULATOR_INFMAT_5_1, MODULATOR_INFMAT_5_2, MODULATOR_INFMAT_5_3}

#define MODULATOR_PINVINFMAT_0_0	(     0.0000000)
#define MODULATOR_PINVINFMAT_0_1	(     0.0000000)
#define MODULATOR_PINVINFMAT_0_2	(    41.4151630)
#define MODULATOR_PINVINFMAT_0_3	(-  183.7762200)
#define MODULATOR_PINVINFMAT_0_4	(-  185.4462000)
#define MODULATOR_PINVINFMAT_0_5	(- 2187.1913000)
#define MODULATOR_PINVINFMAT_1_0	(     0.0000000)
#define MODULATOR_PINVINFMAT_1_1	(     0.0000000)
#define MODULATOR_PINVINFMAT_1_2	(    36.2302910)
#define MODULATOR_PINVINFMAT_1_3	(   183.7762200)
#define MODULATOR_PINVINFMAT_1_4	(   185.4462000)
#define MODULATOR_PINVINFMAT_1_5	(- 2110.8939000)
#define MODULATOR_PINVINFMAT_2_0	(     0.0000000)
#define MODULATOR_PINVINFMAT_2_1	(     0.0000000)
#define MODULATOR_PINVINFMAT_2_2	(    41.4151630)
#define MODULATOR_PINVINFMAT_2_3	(   183.7762200)
#define MODULATOR_PINVINFMAT_2_4	(-  185.4462000)
#define MODULATOR_PINVINFMAT_2_5	(  2187.1913000)
#define MODULATOR_PINVINFMAT_3_0	(     0.0000000)
#define MODULATOR_PINVINFMAT_3_1	(     0.0000000)
#define MODULATOR_PINVINFMAT_3_2	(    36.2302910)
#define MODULATOR_PINVINFMAT_3_3	(-  183.7762200)
#define MODULATOR_PINVINFMAT_3_4	(   185.4462000)
#define MODULATOR_PINVINFMAT_3_5	(  2110.8939000)

#define MODULATOR_PINVINFMAT_0	{MODULATOR_PINVINFMAT_0_0, MODULATOR_PINVINFMAT_0_1, MODULATOR_PINVINFMAT_0_2, MODULATOR_PINVINFMAT_0_3, MODULATOR_PINVINFMAT_0_4, MODULATOR_PINVINFMAT_0_5}
#define MODULATOR_PINVINFMAT_1	{MODULATOR_PINVINFMAT_1_0, MODULATOR_PINVINFMAT_1_1, MODULATOR_PINVINFMAT_1_2, MODULATOR_PINVINFMAT_1_3, MODULATOR_PINVINFMAT_1_4, MODULATOR_PINVINFMAT_1_5}
#define MODULATOR_PINVINFMAT_2	{MODULATOR_PINVINFMAT_2_0, MODULATOR_PINVINFMAT_2_1, MODULATOR_PINVINFMAT_2_2, MODULATOR_PINVINFMAT_2_3, MODULATOR_PINVINFMAT_2_4, MODULATOR_PINVINFMAT_2_5}
#define MODULATOR_PINVINFMAT_3	{MODULATOR_PINVINFMAT_3_0, MODULATOR_PINVINFMAT_3_1, MODULATOR_PINVINFMAT_3_2, MODULATOR_PINVINFMAT_3_3, MODULATOR_PINVINFMAT_3_4, MODULATOR_PINVINFMAT_3_5}

#define MODULATOR_DESCVECT_0	(1.0000000)
#define MODULATOR_DESCVECT_1	(0.8748074)
#define MODULATOR_DESCVECT_2	(1.0000000)
#define MODULATOR_DESCVECT_3	(0.8748074)

#define MODULATOR_DESCVECT		{MODULATOR_DESCVECT_0, MODULATOR_DESCVECT_1, MODULATOR_DESCVECT_2, MODULATOR_DESCVECT_3}

#define MODULATOR_PWMMINOFFSET	1000

/* **************************************************** */
/* Mode parameters: auto-stabilized */

#define MODE_AUTOSTAB_SCALE_ROLLPWM			(  11)
#define MODE_AUTOSTAB_SCALE_ROLLPWMEXP		(- 13)
#define MODE_AUTOSTAB_SCALE_PITCHPWM		(  11)
#define MODE_AUTOSTAB_SCALE_PITCHPWMEXP		(- 13)
#define MODE_AUTOSTAB_SCALE_YAWRATEPWM		(  11)
#define MODE_AUTOSTAB_SCALE_YAWRATEPWMEXP	(- 13)
#define MODE_AUTOSTAB_SCALE_THRUSTPWM		(   1)
#define MODE_AUTOSTAB_SCALE_THRUSTPWMEXP	(-  6)
#define MODE_AUTOSTAB_DEADZONE				( 150)
#define MODE_AUTOSTAB_THRUSTDIR_B_X	(0.)
#define MODE_AUTOSTAB_THRUSTDIR_B_Y	(0.)
#define MODE_AUTOSTAB_THRUSTDIR_B_Z	(1.)

/* **************************************************** */
/* Gimbal management */

#define GIMBAL_PITCH_MAX					0.7853982
#define GIMBAL_PITCH_MIN				   -0.7853982
#define GIMBAL_PITCH_SCALE					763.94373
#define GIMBAL_PITCH_OFFSET					1500
#define GIMBAL_PITCH_IDX					6

#define GIMBAL_ROLL_MAX						0.7853982
#define GIMBAL_ROLL_MIN					   -0.7853982
#define GIMBAL_ROLL_SCALE				   -763.94373
#define GIMBAL_ROLL_OFFSET					1500
#define GIMBAL_ROLL_IDX						7


/* **************************************************** */
/* MAVLINK configuration */

#define CONFIG_MAVLINK_EEPROMREV	(0x0012)
#endif /* NRD_H_ */
