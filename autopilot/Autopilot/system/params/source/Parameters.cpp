/*
 * Parameters.cpp
 *
 *  Created on: 20 mai 2013
 *      Author: Aberzen
 */

#include "../include/Parameters.hpp"

namespace test {

#define PARAM(type, name, addr, defVal) { MAV_PARAM_TYPE_ ## type, name, addr, {type : defVal} }



#define CONFIG_SYSID_SW_MREV	(0)
#define CONFIG_SYSID_SW_TYPE	(0)
#define CONFIG_SYSID_THISMAV	(11)
#define CONFIG_SYSID_MYGCS		(255)
#define CONFIG_SERIAL3_BAUD		(115200/1000)
#define CONFIG_TELEM_DELAY		(0)
#define CONFIG_RTL_ALT			(100)
#define CONFIG_SONAR_ENABLE		(0)
#define CONFIG_SONAR_TYPE		(0)
#define CONFIG_BATT_MONITOR		(0)
#define CONFIG_FS_BATT_ENABLE	(0)
#define CONFIG_VOLT_DIVIDER		(1)
#define CONFIG_AMP_PER_VOLT		(1)
#define CONFIG_INPUT_VOLTS		(12.6)
#define CONFIG_BATT_CAPACITY	(9.6)
#define CONFIG_MAG_ENABLE		(1)
#define CONFIG_FLOW_ENABLE		(0)
#define CONFIG_LOW_VOLT			(9.6)
#define CONFIG_SUPER_SIMPLE		(0)
#define CONFIG_RTL_ALT_FINAL	(2)
#define CONFIG_TILT				(0)
#define CONFIG_BATT_VOLT_PIN	(0)
#define CONFIG_BATT_CURR_PIN	(0)
#define CONFIG_RSSI_PIN			(-1)
#define CONFIG_THR_ACC_ENABLE	(1)
#define CONFIG_YAW_OVR_BEHAVE	(0)
#define CONFIG_WP_TOTAL			(0)
#define CONFIG_WP_INDEX			(0)
#define CONFIG_WP_RADIUS		(1)
#define CONFIG_CIRCLE_RADIUS	(10)
#define CONFIG_WP_SPEED_MAX		(18)
#define CONFIG_XTRK_GAIN_SC		(0)
#define CONFIG_XTRK_MIN_DIST	(0)
#define CONFIG_RTL_LOIT_TIME	(0)
#define CONFIG_LAND_SPEED		(0)
#define CONFIG_AUTO_VELZ_MIN	(0)
#define CONFIG_AUTO_VELZ_MAX	(0)
#define CONFIG_PILOT_VELZ_MAX	(0)
#define CONFIG_THR_MIN			(0)
#define CONFIG_THR_MAX			(0)
#define CONFIG_FS_THR_ENABLE	(0)
#define CONFIG_FS_THR_VALUE		(0)
#define CONFIG_TRIM_THROTTLE	(0)
#define CONFIG_THR_MID			(0)
#define CONFIG_FLTMODE1			(0)
#define CONFIG_FLTMODE2			(0)
#define CONFIG_FLTMODE3			(0)
#define CONFIG_FLTMODE4			(0)
#define CONFIG_FLTMODE5			(0)
#define CONFIG_FLTMODE6			(0)
#define CONFIG_SIMPLE			(0)
#define CONFIG_LOG_BITMASK		(0)
#define CONFIG_TOY_RATE			(1)
#define CONFIG_ESC				(0)
#define CONFIG_TUNE				(0)
#define CONFIG_TUNE_LOW			(0)
#define CONFIG_TUNE_HIGH		(1000)
#define CONFIG_FRAME			(0)
#define CONFIG_CH7_OPT			(0)
#define CONFIG_AUTO_SLEW		(0)
#define CONFIG_RC1_MIN			(1100)
#define CONFIG_RC1_TRIM			(1500)
#define CONFIG_RC1_MAX			(1900)
#define CONFIG_RC1_REV			(1)
#define CONFIG_RC1_DZ			(0)
#define CONFIG_RC2_MIN			(1100)
#define CONFIG_RC2_TRIM			(1500)
#define CONFIG_RC2_MAX			(1900)
#define CONFIG_RC2_REV			(1)
#define CONFIG_RC2_DZ			(0)
#define CONFIG_RC3_MIN			(1100)
#define CONFIG_RC3_TRIM			(1500)
#define CONFIG_RC3_MAX			(1900)
#define CONFIG_RC3_REV			(1)
#define CONFIG_RC3_DZ			(0)
#define CONFIG_RC4_MIN			(1100)
#define CONFIG_RC4_TRIM			(1500)
#define CONFIG_RC4_MAX			(1900)
#define CONFIG_RC4_REV			(1)
#define CONFIG_RC4_DZ			(0)
#define CONFIG_RC5_MIN			(1100)
#define CONFIG_RC5_TRIM			(1500)
#define CONFIG_RC5_MAX			(1900)
#define CONFIG_RC5_REV			(1)
#define CONFIG_RC5_DZ			(0)
#define CONFIG_RC5_FUNCTION		(0)
#define CONFIG_RC6_MIN			(1100)
#define CONFIG_RC6_TRIM			(1500)
#define CONFIG_RC6_MAX			(1900)
#define CONFIG_RC6_REV			(1)
#define CONFIG_RC6_DZ			(0)
#define CONFIG_RC6_FUNCTION		(0)
#define CONFIG_RC7_MIN			(1100)
#define CONFIG_RC7_TRIM			(1500)
#define CONFIG_RC7_MAX			(1900)
#define CONFIG_RC7_REV			(1)
#define CONFIG_RC7_DZ			(0)
#define CONFIG_RC7_FUNCTION		(0)
#define CONFIG_RC8_MIN			(1100)
#define CONFIG_RC8_TRIM			(1500)
#define CONFIG_RC8_MAX			(1900)
#define CONFIG_RC8_REV			(1)
#define CONFIG_RC8_DZ			(0)
#define CONFIG_RC8_FUNCTION		(0)
#define CONFIG_RC10_MIN			(1100)
#define CONFIG_RC10_TRIM		(1500)
#define CONFIG_RC10_MAX			(1900)
#define CONFIG_RC10_REV			(1)
#define CONFIG_RC10_DZ			(0)
#define CONFIG_RC10_FUNCTION	(0)
#define CONFIG_RC11_MIN			(1100)
#define CONFIG_RC11_TRIM		(1500)
#define CONFIG_RC11_MAX			(1900)
#define CONFIG_RC11_REV			(1)
#define CONFIG_RC11_DZ			(0)
#define CONFIG_RC11_FUNCTION	(0)
#define CONFIG_RC_SPEED			(0)
#define CONFIG_ACRO_P			(0)
#define CONFIG_AXIS_ENABLE		(0)
#define CONFIG_ACRO_BAL_ROLL	(0)
#define CONFIG_ACRO_BAL_PITCH	(0)
#define CONFIG_ACRO_TRAINER		(0)
#define CONFIG_LED_MODE			(9)
#define CONFIG_RATE_RLL_P		(0)
#define CONFIG_RATE_RLL_I		(0)
#define CONFIG_RATE_RLL_D		(0)
#define CONFIG_RATE_RLL_IMAX	(0)
#define CONFIG_RATE_PIT_P		(0)
#define CONFIG_RATE_PIT_I		(0)
#define CONFIG_RATE_PIT_D		(0)
#define CONFIG_RATE_PIT_IMAX	(0)
#define CONFIG_RATE_YAW_P		(0)
#define CONFIG_RATE_YAW_I		(0)
#define CONFIG_RATE_YAW_D		(0)
#define CONFIG_RATE_YAW_IMAX	(0)
#define CONFIG_LOITER_LAT_P		(0)
#define CONFIG_LOITER_LAT_I		(0)
#define CONFIG_LOITER_LAT_D		(0)
#define CONFIG_LOITER_LAT_IMAX	(0)
#define CONFIG_LOITER_LON_P		(0)
#define CONFIG_LOITER_LON_I		(0)
#define CONFIG_LOITER_LON_D		(0)
#define CONFIG_LOITER_LON_IMAX	(0)
#define CONFIG_NAT_LAT_P		(0)
#define CONFIG_NAT_LAT_I		(0)
#define CONFIG_NAT_LAT_D		(0)
#define CONFIG_NAT_LAT_IMAX		(0)
#define CONFIG_NAT_LON_P		(0)
#define CONFIG_NAT_LON_I		(0)
#define CONFIG_NAT_LON_D		(0)
#define CONFIG_NAT_LON_IMAX		(0)
#define CONFIG_THR_RATE_P		(0)
#define CONFIG_THR_RATE_I		(0)
#define CONFIG_THR_RATE_D		(0)
#define CONFIG_THR_RATE_IMAX	(0)
#define CONFIG_THR_ACCEL_P		(0)
#define CONFIG_THR_ACCEL_I		(0)
#define CONFIG_THR_ACCEL_D		(0)
#define CONFIG_THR_ACCEL_IMAX	(0)
#define CONFIG_OF_RLL_P			(0)
#define CONFIG_OF_RLL_I			(0)
#define CONFIG_OF_RLL_D			(0)
#define CONFIG_OF_RLL_IMAX		(0)
#define CONFIG_OF_PIT_P			(0)
#define CONFIG_OF_PIT_I			(0)
#define CONFIG_OF_PIT_D			(0)
#define CONFIG_OF_PIT_IMAX		(0)
#define CONFIG_STB_RLL_P		(0)
#define CONFIG_STB_RLL_I		(0)
#define CONFIG_STB_RLL_IMAX		(0)
#define CONFIG_STB_PIT_P		(0)
#define CONFIG_STB_PIT_I		(0)
#define CONFIG_STB_PIT_IMAX		(0)
#define CONFIG_STB_YAW_P		(0)
#define CONFIG_STB_YAW_I		(0)
#define CONFIG_STB_YAW_IMAX		(0)
#define CONFIG_THR_ALT_P		(0)
#define CONFIG_THR_ALT_I		(0)
#define CONFIG_THR_ALT_IMAX		(0)
#define CONFIG_HLD_LAT_P		(0)
#define CONFIG_HLD_LAT_I		(0)
#define CONFIG_HLD_LAT_IMAX		(0)
#define CONFIG_HLD_LON_P		(0)
#define CONFIG_HLD_LON_I		(0)
#define CONFIG_HLD_LON_IMAX		(0)



float VAL_SYSID_SW_MREV = CONFIG_SYSID_SW_MREV;
float VAL_SYSID_SW_TYPE = CONFIG_SYSID_SW_TYPE;
float VAL_SYSID_THISMAV = CONFIG_SYSID_THISMAV;
float VAL_SYSID_MYGCS = CONFIG_SYSID_MYGCS;
//float VAL_SERIAL3_BAUD = CONFIG_SERIAL3_BAUD;
//float VAL_TELEM_DELAY = CONFIG_TELEM_DELAY;
//float VAL_RTL_ALT = CONFIG_RTL_ALT;
//float VAL_SONAR_ENABLE = CONFIG_SONAR_ENABLE;
//float VAL_SONAR_TYPE = CONFIG_SONAR_TYPE;
//float VAL_BATT_MONITOR = CONFIG_BATT_MONITOR;
//float VAL_FS_BATT_ENABLE = CONFIG_FS_BATT_ENABLE;
//float VAL_VOLT_DIVIDER = CONFIG_VOLT_DIVIDER;
//float VAL_AMP_PER_VOLT = CONFIG_AMP_PER_VOLT;
//float VAL_INPUT_VOLTS = CONFIG_INPUT_VOLTS;
//float VAL_BATT_CAPACITY = CONFIG_BATT_CAPACITY;
//float VAL_MAG_ENABLE = CONFIG_MAG_ENABLE;
//float VAL_FLOW_ENABLE = CONFIG_FLOW_ENABLE;
//float VAL_LOW_VOLT = CONFIG_LOW_VOLT;
//float VAL_SUPER_SIMPLE = CONFIG_SUPER_SIMPLE;
//float VAL_RTL_ALT_FINAL = CONFIG_RTL_ALT_FINAL;
//float VAL_TILT = CONFIG_TILT;
//float VAL_BATT_VOLT_PIN = CONFIG_BATT_VOLT_PIN;
//float VAL_BATT_CURR_PIN = CONFIG_BATT_CURR_PIN;
//float VAL_RSSI_PIN = CONFIG_RSSI_PIN;
//float VAL_THR_ACC_ENABLE = CONFIG_THR_ACC_ENABLE;
//float VAL_YAW_OVR_BEHAVE = CONFIG_YAW_OVR_BEHAVE;
//float VAL_WP_TOTAL = CONFIG_WP_TOTAL;
//float VAL_WP_INDEX = CONFIG_WP_INDEX;
//float VAL_WP_RADIUS = CONFIG_WP_RADIUS;
//float VAL_CIRCLE_RADIUS = CONFIG_CIRCLE_RADIUS;
//float VAL_WP_SPEED_MAX = CONFIG_WP_SPEED_MAX;
//float VAL_XTRK_GAIN_SC = CONFIG_XTRK_GAIN_SC;
//float VAL_XTRK_MIN_DIST = CONFIG_XTRK_MIN_DIST;
//float VAL_RTL_LOIT_TIME = CONFIG_RTL_LOIT_TIME;
//float VAL_LAND_SPEED = CONFIG_LAND_SPEED;
//float VAL_AUTO_VELZ_MIN = CONFIG_AUTO_VELZ_MIN;
//float VAL_AUTO_VELZ_MAX = CONFIG_AUTO_VELZ_MAX;
//float VAL_PILOT_VELZ_MAX = CONFIG_PILOT_VELZ_MAX;
//float VAL_THR_MIN = CONFIG_THR_MIN;
//float VAL_THR_MAX = CONFIG_THR_MAX;
//float VAL_FS_THR_ENABLE = CONFIG_FS_THR_ENABLE;
//float VAL_FS_THR_VALUE = CONFIG_FS_THR_VALUE;
//float VAL_TRIM_THROTTLE = CONFIG_TRIM_THROTTLE;
//float VAL_THR_MID = CONFIG_THR_MID;
//float VAL_FLTMODE1 = CONFIG_FLTMODE1;
//float VAL_FLTMODE2 = CONFIG_FLTMODE2;
//float VAL_FLTMODE3 = CONFIG_FLTMODE3;
//float VAL_FLTMODE4 = CONFIG_FLTMODE4;
//float VAL_FLTMODE5 = CONFIG_FLTMODE5;
//float VAL_FLTMODE6 = CONFIG_FLTMODE6;
//float VAL_SIMPLE = CONFIG_SIMPLE;
//float VAL_LOG_BITMASK = CONFIG_LOG_BITMASK;
//float VAL_TOY_RATE = CONFIG_TOY_RATE;
//float VAL_ESC = CONFIG_ESC;
//float VAL_TUNE = CONFIG_TUNE;
//float VAL_TUNE_LOW = CONFIG_TUNE_LOW;
//float VAL_TUNE_HIGH = CONFIG_TUNE_HIGH;
//float VAL_FRAME = CONFIG_FRAME;
//float VAL_CH7_OPT = CONFIG_CH7_OPT;
//float VAL_AUTO_SLEW = CONFIG_AUTO_SLEW;
//float VAL_RC1_MIN = CONFIG_RC1_MIN;
//float VAL_RC1_TRIM = CONFIG_RC1_TRIM;
//float VAL_RC1_MAX = CONFIG_RC1_MAX;
//float VAL_RC1_REV = CONFIG_RC1_REV;
//float VAL_RC1_DZ = CONFIG_RC1_DZ;
//float VAL_RC2_MIN = CONFIG_RC2_MIN;
//float VAL_RC2_TRIM = CONFIG_RC2_TRIM;
//float VAL_RC2_MAX = CONFIG_RC2_MAX;
//float VAL_RC2_REV = CONFIG_RC2_REV;
//float VAL_RC2_DZ = CONFIG_RC2_DZ;
//float VAL_RC3_MIN = CONFIG_RC3_MIN;
//float VAL_RC3_TRIM = CONFIG_RC3_TRIM;
//float VAL_RC3_MAX = CONFIG_RC3_MAX;
//float VAL_RC3_REV = CONFIG_RC3_REV;
//float VAL_RC3_DZ = CONFIG_RC3_DZ;
//float VAL_RC4_MIN = CONFIG_RC4_MIN;
//float VAL_RC4_TRIM = CONFIG_RC4_TRIM;
//float VAL_RC4_MAX = CONFIG_RC4_MAX;
//float VAL_RC4_REV = CONFIG_RC4_REV;
//float VAL_RC4_DZ = CONFIG_RC4_DZ;
//float VAL_RC5_MIN = CONFIG_RC5_MIN;
//float VAL_RC5_TRIM = CONFIG_RC5_TRIM;
//float VAL_RC5_MAX = CONFIG_RC5_MAX;
//float VAL_RC5_REV = CONFIG_RC5_REV;
//float VAL_RC5_DZ = CONFIG_RC5_DZ;
//float VAL_RC5_FUNCTION = CONFIG_RC5_FUNCTION;
//float VAL_RC6_MIN = CONFIG_RC6_MIN;
//float VAL_RC6_TRIM = CONFIG_RC6_TRIM;
//float VAL_RC6_MAX = CONFIG_RC6_MAX;
//float VAL_RC6_REV = CONFIG_RC6_REV;
//float VAL_RC6_DZ = CONFIG_RC6_DZ;
//float VAL_RC6_FUNCTION = CONFIG_RC6_FUNCTION;
//float VAL_RC7_MIN = CONFIG_RC7_MIN;
//float VAL_RC7_TRIM = CONFIG_RC7_TRIM;
//float VAL_RC7_MAX = CONFIG_RC7_MAX;
//float VAL_RC7_REV = CONFIG_RC7_REV;
//float VAL_RC7_DZ = CONFIG_RC7_DZ;
//float VAL_RC7_FUNCTION = CONFIG_RC7_FUNCTION;
//float VAL_RC8_MIN = CONFIG_RC8_MIN;
//float VAL_RC8_TRIM = CONFIG_RC8_TRIM;
//float VAL_RC8_MAX = CONFIG_RC8_MAX;
//float VAL_RC8_REV = CONFIG_RC8_REV;
//float VAL_RC8_DZ = CONFIG_RC8_DZ;
//float VAL_RC8_FUNCTION = CONFIG_RC8_FUNCTION;
//float VAL_RC10_MIN = CONFIG_RC10_MIN;
//float VAL_RC10_TRIM = CONFIG_RC10_TRIM;
//float VAL_RC10_MAX = CONFIG_RC10_MAX;
//float VAL_RC10_REV = CONFIG_RC10_REV;
//float VAL_RC10_DZ = CONFIG_RC10_DZ;
//float VAL_RC10_FUNCTION = CONFIG_RC10_FUNCTION;
//float VAL_RC11_MIN = CONFIG_RC11_MIN;
//float VAL_RC11_TRIM = CONFIG_RC11_TRIM;
//float VAL_RC11_MAX = CONFIG_RC11_MAX;
//float VAL_RC11_REV = CONFIG_RC11_REV;
//float VAL_RC11_DZ = CONFIG_RC11_DZ;
//float VAL_RC11_FUNCTION = CONFIG_RC11_FUNCTION;
//float VAL_RC_SPEED = CONFIG_RC_SPEED;
//float VAL_ACRO_P = CONFIG_ACRO_P;
//float VAL_AXIS_ENABLE = CONFIG_AXIS_ENABLE;
//float VAL_ACRO_BAL_ROLL = CONFIG_ACRO_BAL_ROLL;
//float VAL_ACRO_BAL_PITCH = CONFIG_ACRO_BAL_PITCH;
//float VAL_ACRO_TRAINER = CONFIG_ACRO_TRAINER;
//float VAL_LED_MODE = CONFIG_LED_MODE;
//float VAL_RATE_RLL_P = CONFIG_RATE_RLL_P;
//float VAL_RATE_RLL_I = CONFIG_RATE_RLL_I;
//float VAL_RATE_RLL_D = CONFIG_RATE_RLL_D;
//float VAL_RATE_RLL_IMAX = CONFIG_RATE_RLL_IMAX;
//float VAL_RATE_PIT_P = CONFIG_RATE_PIT_P;
//float VAL_RATE_PIT_I = CONFIG_RATE_PIT_I;
//float VAL_RATE_PIT_D = CONFIG_RATE_PIT_D;
//float VAL_RATE_PIT_IMAX = CONFIG_RATE_PIT_IMAX;
//float VAL_RATE_YAW_P = CONFIG_RATE_YAW_P;
//float VAL_RATE_YAW_I = CONFIG_RATE_YAW_I;
//float VAL_RATE_YAW_D = CONFIG_RATE_YAW_D;
//float VAL_RATE_YAW_IMAX = CONFIG_RATE_YAW_IMAX;
//float VAL_LOITER_LAT_P = CONFIG_LOITER_LAT_P;
//float VAL_LOITER_LAT_I = CONFIG_LOITER_LAT_I;
//float VAL_LOITER_LAT_D = CONFIG_LOITER_LAT_D;
//float VAL_LOITER_LAT_IMAX = CONFIG_LOITER_LAT_IMAX;
//float VAL_LOITER_LON_P = CONFIG_LOITER_LON_P;
//float VAL_LOITER_LON_I = CONFIG_LOITER_LON_I;
//float VAL_LOITER_LON_D = CONFIG_LOITER_LON_D;
//float VAL_LOITER_LON_IMAX = CONFIG_LOITER_LON_IMAX;
//float VAL_NAT_LAT_P = CONFIG_NAT_LAT_P;
//float VAL_NAT_LAT_I = CONFIG_NAT_LAT_I;
//float VAL_NAT_LAT_D = CONFIG_NAT_LAT_D;
//float VAL_NAT_LAT_IMAX = CONFIG_NAT_LAT_IMAX;
//float VAL_NAT_LON_P = CONFIG_NAT_LON_P;
//float VAL_NAT_LON_I = CONFIG_NAT_LON_I;
//float VAL_NAT_LON_D = CONFIG_NAT_LON_D;
//float VAL_NAT_LON_IMAX = CONFIG_NAT_LON_IMAX;
//float VAL_THR_RATE_P = CONFIG_THR_RATE_P;
//float VAL_THR_RATE_I = CONFIG_THR_RATE_I;
//float VAL_THR_RATE_D = CONFIG_THR_RATE_D;
//float VAL_THR_RATE_IMAX = CONFIG_THR_RATE_IMAX;
//float VAL_THR_ACCEL_P = CONFIG_THR_ACCEL_P;
//float VAL_THR_ACCEL_I = CONFIG_THR_ACCEL_I;
//float VAL_THR_ACCEL_D = CONFIG_THR_ACCEL_D;
//float VAL_THR_ACCEL_IMAX = CONFIG_THR_ACCEL_IMAX;
//float VAL_OF_RLL_P = CONFIG_OF_RLL_P;
//float VAL_OF_RLL_I = CONFIG_OF_RLL_I;
//float VAL_OF_RLL_D = CONFIG_OF_RLL_D;
//float VAL_OF_RLL_IMAX = CONFIG_OF_RLL_IMAX;
//float VAL_OF_PIT_P = CONFIG_OF_PIT_P;
//float VAL_OF_PIT_I = CONFIG_OF_PIT_I;
//float VAL_OF_PIT_D = CONFIG_OF_PIT_D;
//float VAL_OF_PIT_IMAX = CONFIG_OF_PIT_IMAX;
//float VAL_STB_RLL_P = CONFIG_STB_RLL_P;
//float VAL_STB_RLL_I = CONFIG_STB_RLL_I;
//float VAL_STB_RLL_IMAX = CONFIG_STB_RLL_IMAX;
//float VAL_STB_PIT_P = CONFIG_STB_PIT_P;
//float VAL_STB_PIT_I = CONFIG_STB_PIT_I;
//float VAL_STB_PIT_IMAX = CONFIG_STB_PIT_IMAX;
//float VAL_STB_YAW_P = CONFIG_STB_YAW_P;
//float VAL_STB_YAW_I = CONFIG_STB_YAW_I;
//float VAL_STB_YAW_IMAX = CONFIG_STB_YAW_IMAX;
//float VAL_THR_ALT_P = CONFIG_THR_ALT_P;
//float VAL_THR_ALT_I = CONFIG_THR_ALT_I;
//float VAL_THR_ALT_IMAX = CONFIG_THR_ALT_IMAX;
//float VAL_HLD_LAT_P = CONFIG_HLD_LAT_P;
//float VAL_HLD_LAT_I = CONFIG_HLD_LAT_I;
//float VAL_HLD_LAT_IMAX = CONFIG_HLD_LAT_IMAX;
//float VAL_HLD_LON_P = CONFIG_HLD_LON_P;
//float VAL_HLD_LON_I = CONFIG_HLD_LON_I;
//float VAL_HLD_LON_IMAX = CONFIG_HLD_LON_IMAX;


autom::Ancs::Param config_ancs = {
		{ /* gen */
				PHYSICS_MASS, /* mass */
				PHYSICS_INERTIA_XX, /* inertiaXX */
				PHYSICS_INERTIA_YY, /* inertiaYY */
				PHYSICS_INERTIA_ZZ /* inertiaZZ */
		}, /* gen */
		{ /* est */
			EST_KALMAN_GAIN_ACCO, /* gainAcco */
			EST_KALMAN_GAIN_COMPASS /* gainCompass */
		}, /* est */
		{ /* procCalibImu */
				PROC_CALIBIMU_FILT_NUM, /* filtCoeffNum */
				PROC_CALIBIMU_FILT_DEN, /* filtCoeffDen */
				PROC_CALIBIMU_NBMEAS_BIAS, /* biasNbMeas */
				PROC_CALIBIMU_NBMEAS_VAR, /* varNbMeas */
				PROC_CALIBIMU_GYRO_VARTHD, /* gyroVarianceThd */
				PROC_CALIBIMU_ACCO_VARTHD /* accoVarianceThd */
		}, /* procCalibImu */
		{ /* procCompDec */
				PROC_COMPDEC_FILT_NUM, /* filtCoeffNum */
				PROC_COMPDEC_FILT_DEN, /* filtCoeffDen */
				PROC_COMPDEC_NBMEAS /* biasNbMeas */
		}, /* procCompDec */
		{ /* procGrdDetect */
				0.981, /* detectThd */
				20 /* filtDur */
		}, /* procGrdDetect */
		{ /* modGen */
				{
						MODULATOR_INFMAT_0,
						MODULATOR_INFMAT_1,
						MODULATOR_INFMAT_2,
						MODULATOR_INFMAT_3,
						MODULATOR_INFMAT_4,
						MODULATOR_INFMAT_5
				}
		}, /* modGen */
		{ /* modPinv */
				{ /* pInvInfMat */
						MODULATOR_PINVINFMAT_0,
						MODULATOR_PINVINFMAT_1,
						MODULATOR_PINVINFMAT_2,
						MODULATOR_PINVINFMAT_3,
				}, /* pInvInfMat */
				MODULATOR_DESCVECT
		}, /* modPinv */
		{ /* modeStabilized */
				{ /* attCtrl */
					{ /* x */
						CTRL_ATT_X_Kp, /* _Kp */
						CTRL_ATT_X_Kd, /*_Kd */
						CTRL_ATT_X_Ki, /*_Ki */
						CTRL_ATT_X_useOfRb, /*_useOfRb */
						CTRL_ATT_X_Krb, /*_Krb */
						CTRL_ATT_X_rbThd, /*_rbThd */
						CTRL_ATT_X_rb, /*_rb */
						CTRL_ATT_X_maxI /*_maxI */
					}, /* x */
					{ /* y */
						CTRL_ATT_Y_Kp, /* _Kp */
						CTRL_ATT_Y_Kd, /*_Kd */
						CTRL_ATT_Y_Ki, /*_Ki */
						CTRL_ATT_Y_useOfRb, /*_useOfRb */
						CTRL_ATT_Y_Krb, /*_Krb */
						CTRL_ATT_Y_rbThd, /*_rbThd */
						CTRL_ATT_Y_rb, /*_rb */
						CTRL_ATT_Y_maxI /*_maxI */
					}, /* y */
					{ /* z */
						CTRL_ATT_Z_Kp, /* _Kp */
						CTRL_ATT_Z_Kd, /*_Kd */
						CTRL_ATT_Z_Ki, /*_Ki */
						CTRL_ATT_Z_useOfRb, /*_useOfRb */
						CTRL_ATT_Z_Krb, /*_Krb */
						CTRL_ATT_Z_rbThd, /*_rbThd */
						CTRL_ATT_Z_rb, /*_rb */
						CTRL_ATT_Z_maxI /*_maxI */
					} /* z */
				}, /* attCtrl */
				MODE_AUTOSTAB_SCALE_ROLLPWM, /* rollPwmScale */
				MODE_AUTOSTAB_SCALE_ROLLPWMEXP, /* rollPwmScaleExp */
				MODE_AUTOSTAB_SCALE_PITCHPWM, /* pitchPwmScale */
				MODE_AUTOSTAB_SCALE_PITCHPWMEXP, /* pitchPwmScaleExp */
				MODE_AUTOSTAB_SCALE_YAWRATEPWM, /* yawRatePwmScale */
				MODE_AUTOSTAB_SCALE_YAWRATEPWMEXP, /* yawRatePwmScaleExp */
				MODE_AUTOSTAB_SCALE_THRUSTPWM, /* thrustPwmScale */
				MODE_AUTOSTAB_SCALE_THRUSTPWMEXP, /* thrustPwmScaleExp */
				MODE_AUTOSTAB_SCALE_THRUSTDIR_B_X, /* thrustDir_B_x */
				MODE_AUTOSTAB_SCALE_THRUSTDIR_B_Y, /* thrustDir_B_y */
				MODE_AUTOSTAB_SCALE_THRUSTDIR_B_Z /* thrustDir_B_z */
		} /* modeStabilized */
};



const mavlink::ParameterMgt::ParamInfo config[CONFIG_PARAMETERS_COUNT] = {
		// @Param: SYSID_SW_MREV
		// @DisplayName: Eeprom format version number
		// @Description: This value is incremented when changes are made to the eeprom format
		// @User: Advanced
		PARAM(REAL32, "SYSID_SW_MREV", &VAL_SYSID_SW_MREV, CONFIG_SYSID_SW_MREV),

		// @Param: SYSID_SW_TYPE
		// @DisplayName: Software Type
		// @Description: This is used by the ground station to recognise the software type (eg ArduPlane vs ArduCopter)
		// @User: Advanced
		PARAM(REAL32, "SYSID_SW_TYPE", &VAL_SYSID_SW_TYPE, CONFIG_SYSID_SW_TYPE),

		// @Param: SYSID_THISMAV
		// @DisplayName: Mavlink version
		// @Description: Allows reconising the mavlink version
		// @User: Advanced
		PARAM(REAL32, "SYSID_THISMAV", &VAL_SYSID_THISMAV, CONFIG_SYSID_THISMAV),
		PARAM(REAL32, "SYSID_MYGCS", &VAL_SYSID_MYGCS, CONFIG_SYSID_MYGCS),



		PARAM(REAL32, "GEN_MASS", &config_ancs.gen.mass, PHYSICS_MASS),
		PARAM(REAL32, "GEN_INER_XX", &config_ancs.gen.inertiaXX, PHYSICS_INERTIA_XX),
		PARAM(REAL32, "GEN_INER_YY", &config_ancs.gen.inertiaYY, PHYSICS_INERTIA_YY),
		PARAM(REAL32, "GEN_INER_ZZ", &config_ancs.gen.inertiaZZ, PHYSICS_INERTIA_ZZ),

		PARAM(REAL32, "EST_K_ACCO", &config_ancs.est.gainAcco, EST_KALMAN_GAIN_ACCO),
		PARAM(REAL32, "EST_K_COMP", &config_ancs.est.gainAcco, EST_KALMAN_GAIN_COMPASS),

		PARAM(REAL32, "CALIMU_NUM0", &config_ancs.procCalibImu.filtCoeffNum[0], PROC_CALIBIMU_FILT_NUM0),
		PARAM(REAL32, "CALIMU_NUM1", &config_ancs.procCalibImu.filtCoeffNum[1], PROC_CALIBIMU_FILT_NUM1),
		PARAM(REAL32, "CALIMU_DEN0", &config_ancs.procCalibImu.filtCoeffDen[0], PROC_CALIBIMU_FILT_DEN0),
		PARAM(REAL32, "CALIMU_DEN1", &config_ancs.procCalibImu.filtCoeffDen[1], PROC_CALIBIMU_FILT_DEN1),
		PARAM(UINT16, "CALIMU_NBBIAS", &config_ancs.procCalibImu.biasNbMeas, PROC_CALIBIMU_NBMEAS_BIAS),
		PARAM(UINT16, "CALIMU_NBVAR", &config_ancs.procCalibImu.varNbMeas, PROC_CALIBIMU_NBMEAS_VAR),
		PARAM(REAL32, "CALIMU_THDGYRO", &config_ancs.procCalibImu.gyroVarianceThd, PROC_CALIBIMU_GYRO_VARTHD),
		PARAM(REAL32, "CALIMU_THDACCO", &config_ancs.procCalibImu.accoVarianceThd, PROC_CALIBIMU_ACCO_VARTHD),

		PARAM(REAL32, "DECLIN_NUM0", &config_ancs.procCompDec.filtCoeffNum[0], PROC_COMPDEC_FILT_NUM0),
		PARAM(REAL32, "DECLIN_NUM1", &config_ancs.procCompDec.filtCoeffNum[1], PROC_COMPDEC_FILT_NUM1),
		PARAM(REAL32, "DECLIN_DEN0", &config_ancs.procCompDec.filtCoeffDen[0], PROC_COMPDEC_FILT_DEN0),
		PARAM(REAL32, "DECLIN_DEN1", &config_ancs.procCompDec.filtCoeffDen[1], PROC_COMPDEC_FILT_DEN1),
		PARAM(UINT16, "DECLIN_NB", &config_ancs.procCompDec.nbMeas, PROC_COMPDEC_NBMEAS),

		PARAM(REAL32, "MOD_INFMAT_0_0", &config_ancs.modGen.infMat[0][0], MODULATOR_INFMAT_0_0),
		PARAM(REAL32, "MOD_INFMAT_0_1", &config_ancs.modGen.infMat[0][1], MODULATOR_INFMAT_0_1),
		PARAM(REAL32, "MOD_INFMAT_0_2", &config_ancs.modGen.infMat[0][2], MODULATOR_INFMAT_0_2),
		PARAM(REAL32, "MOD_INFMAT_0_3", &config_ancs.modGen.infMat[0][3], MODULATOR_INFMAT_0_3),
		PARAM(REAL32, "MOD_INFMAT_1_0", &config_ancs.modGen.infMat[1][0], MODULATOR_INFMAT_1_0),
		PARAM(REAL32, "MOD_INFMAT_1_1", &config_ancs.modGen.infMat[1][1], MODULATOR_INFMAT_1_1),
		PARAM(REAL32, "MOD_INFMAT_1_2", &config_ancs.modGen.infMat[1][2], MODULATOR_INFMAT_1_2),
		PARAM(REAL32, "MOD_INFMAT_1_3", &config_ancs.modGen.infMat[1][3], MODULATOR_INFMAT_1_3),
		PARAM(REAL32, "MOD_INFMAT_2_0", &config_ancs.modGen.infMat[2][0], MODULATOR_INFMAT_2_0),
		PARAM(REAL32, "MOD_INFMAT_2_1", &config_ancs.modGen.infMat[2][1], MODULATOR_INFMAT_2_1),
		PARAM(REAL32, "MOD_INFMAT_2_2", &config_ancs.modGen.infMat[2][2], MODULATOR_INFMAT_2_2),
		PARAM(REAL32, "MOD_INFMAT_2_3", &config_ancs.modGen.infMat[2][3], MODULATOR_INFMAT_2_3),
		PARAM(REAL32, "MOD_INFMAT_3_0", &config_ancs.modGen.infMat[3][0], MODULATOR_INFMAT_3_0),
		PARAM(REAL32, "MOD_INFMAT_3_1", &config_ancs.modGen.infMat[3][1], MODULATOR_INFMAT_3_1),
		PARAM(REAL32, "MOD_INFMAT_3_2", &config_ancs.modGen.infMat[3][2], MODULATOR_INFMAT_3_2),
		PARAM(REAL32, "MOD_INFMAT_3_3", &config_ancs.modGen.infMat[3][3], MODULATOR_INFMAT_3_3),
		PARAM(REAL32, "MOD_INFMAT_4_0", &config_ancs.modGen.infMat[4][0], MODULATOR_INFMAT_4_0),
		PARAM(REAL32, "MOD_INFMAT_4_1", &config_ancs.modGen.infMat[4][1], MODULATOR_INFMAT_4_1),
		PARAM(REAL32, "MOD_INFMAT_4_2", &config_ancs.modGen.infMat[4][2], MODULATOR_INFMAT_4_2),
		PARAM(REAL32, "MOD_INFMAT_4_3", &config_ancs.modGen.infMat[4][3], MODULATOR_INFMAT_4_3),
		PARAM(REAL32, "MOD_INFMAT_5_0", &config_ancs.modGen.infMat[5][0], MODULATOR_INFMAT_5_0),
		PARAM(REAL32, "MOD_INFMAT_5_1", &config_ancs.modGen.infMat[5][1], MODULATOR_INFMAT_5_1),
		PARAM(REAL32, "MOD_INFMAT_5_2", &config_ancs.modGen.infMat[5][2], MODULATOR_INFMAT_5_2),
		PARAM(REAL32, "MOD_INFMAT_5_3", &config_ancs.modGen.infMat[5][3], MODULATOR_INFMAT_5_3),

		PARAM(REAL32, "MOD_PINV_0_0", &config_ancs.modPinv.pInvInfMat[0][0], MODULATOR_PINVINFMAT_0_0),
		PARAM(REAL32, "MOD_PINV_0_1", &config_ancs.modPinv.pInvInfMat[0][1], MODULATOR_PINVINFMAT_0_1),
		PARAM(REAL32, "MOD_PINV_0_2", &config_ancs.modPinv.pInvInfMat[0][2], MODULATOR_PINVINFMAT_0_2),
		PARAM(REAL32, "MOD_PINV_0_3", &config_ancs.modPinv.pInvInfMat[0][3], MODULATOR_PINVINFMAT_0_3),
		PARAM(REAL32, "MOD_PINV_0_4", &config_ancs.modPinv.pInvInfMat[0][4], MODULATOR_PINVINFMAT_0_4),
		PARAM(REAL32, "MOD_PINV_0_5", &config_ancs.modPinv.pInvInfMat[0][5], MODULATOR_PINVINFMAT_0_5),
		PARAM(REAL32, "MOD_PINV_1_0", &config_ancs.modPinv.pInvInfMat[1][0], MODULATOR_PINVINFMAT_1_0),
		PARAM(REAL32, "MOD_PINV_1_1", &config_ancs.modPinv.pInvInfMat[1][1], MODULATOR_PINVINFMAT_1_1),
		PARAM(REAL32, "MOD_PINV_1_2", &config_ancs.modPinv.pInvInfMat[1][2], MODULATOR_PINVINFMAT_1_2),
		PARAM(REAL32, "MOD_PINV_1_3", &config_ancs.modPinv.pInvInfMat[1][3], MODULATOR_PINVINFMAT_1_3),
		PARAM(REAL32, "MOD_PINV_1_4", &config_ancs.modPinv.pInvInfMat[1][4], MODULATOR_PINVINFMAT_1_4),
		PARAM(REAL32, "MOD_PINV_1_5", &config_ancs.modPinv.pInvInfMat[1][5], MODULATOR_PINVINFMAT_1_5),
		PARAM(REAL32, "MOD_PINV_2_0", &config_ancs.modPinv.pInvInfMat[2][0], MODULATOR_PINVINFMAT_2_0),
		PARAM(REAL32, "MOD_PINV_2_1", &config_ancs.modPinv.pInvInfMat[2][1], MODULATOR_PINVINFMAT_2_1),
		PARAM(REAL32, "MOD_PINV_2_2", &config_ancs.modPinv.pInvInfMat[2][2], MODULATOR_PINVINFMAT_2_2),
		PARAM(REAL32, "MOD_PINV_2_3", &config_ancs.modPinv.pInvInfMat[2][3], MODULATOR_PINVINFMAT_2_3),
		PARAM(REAL32, "MOD_PINV_2_4", &config_ancs.modPinv.pInvInfMat[2][4], MODULATOR_PINVINFMAT_2_4),
		PARAM(REAL32, "MOD_PINV_2_5", &config_ancs.modPinv.pInvInfMat[2][5], MODULATOR_PINVINFMAT_2_5),
		PARAM(REAL32, "MOD_PINV_3_0", &config_ancs.modPinv.pInvInfMat[3][0], MODULATOR_PINVINFMAT_3_0),
		PARAM(REAL32, "MOD_PINV_3_1", &config_ancs.modPinv.pInvInfMat[3][1], MODULATOR_PINVINFMAT_3_1),
		PARAM(REAL32, "MOD_PINV_3_2", &config_ancs.modPinv.pInvInfMat[3][2], MODULATOR_PINVINFMAT_3_2),
		PARAM(REAL32, "MOD_PINV_3_3", &config_ancs.modPinv.pInvInfMat[3][3], MODULATOR_PINVINFMAT_3_3),
		PARAM(REAL32, "MOD_PINV_3_4", &config_ancs.modPinv.pInvInfMat[3][4], MODULATOR_PINVINFMAT_3_4),
		PARAM(REAL32, "MOD_PINV_3_5", &config_ancs.modPinv.pInvInfMat[3][5], MODULATOR_PINVINFMAT_3_5),

		PARAM(REAL32, "MOD_DSC_0", &config_ancs.modPinv.descVect[0], MODULATOR_DESCVECT_0),
		PARAM(REAL32, "MOD_DSC_1", &config_ancs.modPinv.descVect[1], MODULATOR_DESCVECT_1),
		PARAM(REAL32, "MOD_DSC_2", &config_ancs.modPinv.descVect[2], MODULATOR_DESCVECT_2),
		PARAM(REAL32, "MOD_DSC_3", &config_ancs.modPinv.descVect[3], MODULATOR_DESCVECT_3),

		PARAM(REAL32, "CTLATT_AS_X_KP", &config_ancs.modeStabilized.attCtrl.x.Kp, CTRL_ATT_X_Kp),
		PARAM(REAL32, "CTLATT_AS_X_KD", &config_ancs.modeStabilized.attCtrl.x.Kd, CTRL_ATT_X_Kd),
		PARAM(REAL32, "CTLATT_AS_X_KI", &config_ancs.modeStabilized.attCtrl.x.Ki, CTRL_ATT_X_Ki),
		PARAM(UINT8, "CTLATT_AS_X_ISRB", &config_ancs.modeStabilized.attCtrl.x.useOfRb, CTRL_ATT_X_useOfRb),
		PARAM(REAL32, "CTLATT_AS_X_KRB", &config_ancs.modeStabilized.attCtrl.x.Krb, CTRL_ATT_X_Krb),
		PARAM(REAL32, "CTLATT_AS_X_RB", &config_ancs.modeStabilized.attCtrl.x.rbThd, CTRL_ATT_X_rbThd),
		PARAM(REAL32, "CTLATT_AS_X_MAXI", &config_ancs.modeStabilized.attCtrl.x.maxI, CTRL_ATT_X_maxI),
		PARAM(REAL32, "CTLATT_AS_Y_KP", &config_ancs.modeStabilized.attCtrl.y.Kp, CTRL_ATT_Y_Kp),
		PARAM(REAL32, "CTLATT_AS_Y_KD", &config_ancs.modeStabilized.attCtrl.y.Kd, CTRL_ATT_Y_Kd),
		PARAM(REAL32, "CTLATT_AS_Y_KI", &config_ancs.modeStabilized.attCtrl.y.Ki, CTRL_ATT_Y_Ki),
		PARAM(UINT8, "CTLATT_AS_Y_ISRB", &config_ancs.modeStabilized.attCtrl.y.useOfRb, CTRL_ATT_Y_useOfRb),
		PARAM(REAL32, "CTLATT_AS_Y_KRB", &config_ancs.modeStabilized.attCtrl.y.Krb, CTRL_ATT_Y_Krb),
		PARAM(REAL32, "CTLATT_AS_Y_RB", &config_ancs.modeStabilized.attCtrl.y.rbThd, CTRL_ATT_Y_rbThd),
		PARAM(REAL32, "CTLATT_AS_Y_MAXI", &config_ancs.modeStabilized.attCtrl.y.maxI, CTRL_ATT_Y_maxI),
		PARAM(REAL32, "CTLATT_AS_Z_KP", &config_ancs.modeStabilized.attCtrl.z.Kp, CTRL_ATT_Z_Kp),
		PARAM(REAL32, "CTLATT_AS_Z_KD", &config_ancs.modeStabilized.attCtrl.z.Kd, CTRL_ATT_Z_Kd),
		PARAM(REAL32, "CTLATT_AS_Z_KI", &config_ancs.modeStabilized.attCtrl.z.Ki, CTRL_ATT_Z_Ki),
		PARAM(UINT8, "CTLATT_AS_Z_ISRB", &config_ancs.modeStabilized.attCtrl.z.useOfRb, CTRL_ATT_Z_useOfRb),
		PARAM(REAL32, "CTLATT_AS_Z_KRB", &config_ancs.modeStabilized.attCtrl.z.Krb, CTRL_ATT_Z_Krb),
		PARAM(REAL32, "CTLATT_AS_Z_RB", &config_ancs.modeStabilized.attCtrl.z.rbThd, CTRL_ATT_Z_rbThd),
		PARAM(REAL32, "CTLATT_AS_Z_MAXI", &config_ancs.modeStabilized.attCtrl.z.maxI, CTRL_ATT_Z_maxI),

		PARAM(INT8, "AS_SCL_R", &config_ancs.modeStabilized.rollPwmScale, MODE_AUTOSTAB_SCALE_ROLLPWM),
		PARAM(INT8, "AS_SCL_REXP", &config_ancs.modeStabilized.rollPwmScaleExp, MODE_AUTOSTAB_SCALE_ROLLPWMEXP),
		PARAM(INT8, "AS_SCL_P", &config_ancs.modeStabilized.pitchPwmScale, MODE_AUTOSTAB_SCALE_PITCHPWM),
		PARAM(INT8, "AS_SCL_PEXP", &config_ancs.modeStabilized.pitchPwmScaleExp, MODE_AUTOSTAB_SCALE_PITCHPWMEXP),
		PARAM(INT8, "AS_SCL_Y", &config_ancs.modeStabilized.yawRatePwmScale, MODE_AUTOSTAB_SCALE_YAWRATEPWM),
		PARAM(INT8, "AS_SCL_YEXP", &config_ancs.modeStabilized.yawRatePwmScaleExp, MODE_AUTOSTAB_SCALE_YAWRATEPWMEXP),
		PARAM(INT8, "AS_SCL_T", &config_ancs.modeStabilized.thrustPwmScale, MODE_AUTOSTAB_SCALE_THRUSTPWM),
		PARAM(INT8, "AS_SCL_TEXP", &config_ancs.modeStabilized.thrustPwmScaleExp, MODE_AUTOSTAB_SCALE_THRUSTPWMEXP),
		PARAM(REAL32, "AS_SCL_T_X", &config_ancs.modeStabilized.thrustDir_B_x, MODE_AUTOSTAB_SCALE_THRUSTDIR_B_X),
		PARAM(REAL32, "AS_SCL_T_Y", &config_ancs.modeStabilized.thrustDir_B_y, MODE_AUTOSTAB_SCALE_THRUSTDIR_B_Y),
		PARAM(REAL32, "AS_SCL_T_Z", &config_ancs.modeStabilized.thrustDir_B_z, MODE_AUTOSTAB_SCALE_THRUSTDIR_B_Z),


//		// @Param: SERIAL3_BAUD
//		// @DisplayName: Telemetry Baud Rate
//		// @Description: The baud rate used on the telemetry port
//		// @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
//		// @User: Standard
//		PARAM(REAL32, "SERIAL3_BAUD", &VAL_SERIAL3_BAUD, CONFIG_SERIAL3_BAUD),
//
//		// @Param: TELEM_DELAY
//		// @DisplayName: Telemetry startup delay
//		// @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
//		// @User: Standard
//		// @Units: seconds
//		// @Range: 0 10
//		// @Increment: 1
//		PARAM(REAL32, "TELEM_DELAY", &VAL_TELEM_DELAY, CONFIG_TELEM_DELAY),
//
//		// @Param: ALT_RTL
//		// @DisplayName: RTL Altitude
//		// @Description: The minimum altitude the model will move to before Returning to Launch.  Set to zero to return at current altitude.
//		// @Units: Centimeters
//		// @Range: 0 4000
//		// @Increment: 1
//		// @User: Standard
//		PARAM(REAL32, "RTL_ALT", &VAL_RTL_ALT, CONFIG_RTL_ALT),
//
//		// @Param: SONAR_ENABLE
//		// @DisplayName: Enable Sonar
//		// @Description: Setting this to Enabled(1) will enable the sonar. Setting this to Disabled(0) will disable the sonar
//		// @Values: 0:Disabled,1:Enabled
//		// @User: Standard
//		PARAM(REAL32, "SONAR_ENABLE", &VAL_SONAR_ENABLE, CONFIG_SONAR_ENABLE),
//
//		// @Param: SONAR_TYPE
//		// @DisplayName: Sonar type
//		// @Description: Used to adjust scaling to match the sonar used (only Maxbotix sonars are supported at this time)
//		// @Values: 0:XL-EZ0,1:LV-EZ0,2:XLL-EZ0,3:HRLV
//		// @User: Standard
//		PARAM(REAL32, "SONAR_TYPE", &VAL_SONAR_TYPE, CONFIG_SONAR_TYPE),
//
//		// @Param: BATT_MONITOR
//		// @DisplayName: Battery monitoring
//		// @Description: Controls enabling monitoring of the battery's voltage and current
//		// @Values: 0:Disabled,3:Voltage Only,4:Voltage and Current
//		// @User: Standard
//		PARAM(REAL32, "BATT_MONITOR", &VAL_BATT_MONITOR, CONFIG_BATT_MONITOR),
//
//		// @Param: FS_BATT_ENABLE
//		// @DisplayName: Battery Failsafe Enable
//		// @Description: Controls whether failsafe will be invoked when battery voltage or current runs low
//		// @Values: 0:Disabled,1:Enabled
//		// @User: Standard
//		PARAM(REAL32, "FS_BATT_ENABLE", &VAL_FS_BATT_ENABLE, CONFIG_FS_BATT_ENABLE),
//
//		// @Param: VOLT_DIVIDER
//		// @DisplayName: Voltage Divider
//		// @Description: Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin voltage * INPUT_VOLTS/1024 * VOLT_DIVIDER)
//		// @User: Advanced
//		PARAM(REAL32, "VOLT_DIVIDER", &VAL_VOLT_DIVIDER, CONFIG_VOLT_DIVIDER),
//
//		// @Param: AMP_PER_VOLT
//		// @DisplayName: Current Amps per volt
//		// @Description: Used to convert the voltage on the current sensing pin (BATT_CURR_PIN) to the actual current being consumed in amps (curr pin voltage * INPUT_VOLTS/1024 * AMP_PER_VOLT )
//		// @User: Advanced
//		PARAM(REAL32, "AMP_PER_VOLT", &VAL_AMP_PER_VOLT, CONFIG_AMP_PER_VOLT),
//
//		// @Param: INPUT_VOLTS
//		// @DisplayName: Max internal voltage of the battery voltage and current sensing pins
//		// @Description: Used to convert the voltage read in on the voltage and current pins for battery monitoring.  Normally 5 meaning 5 volts.
//		// @User: Advanced
//		PARAM(REAL32, "INPUT_VOLTS", &VAL_INPUT_VOLTS, CONFIG_INPUT_VOLTS),
//
//		// @Param: BATT_CAPACITY
//		// @DisplayName: Battery Capacity
//		// @Description: Battery capacity in milliamp-hours (mAh)
//		// @Units: mAh
//		PARAM(REAL32, "BATT_CAPACITY", &VAL_BATT_CAPACITY, CONFIG_BATT_CAPACITY),
//
//		// @Param: MAG_ENABLE
//		// @DisplayName: Enable Compass
//		// @Description: Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass
//		// @Values: 0:Disabled,1:Enabled
//		// @User: Standard
//		PARAM(REAL32, "MAG_ENABLE", &VAL_MAG_ENABLE, CONFIG_MAG_ENABLE),
//
//		// @Param: FLOW_ENABLE
//		// @DisplayName: Enable Optical Flow
//		// @Description: Setting this to Enabled(1) will enable optical flow. Setting this to Disabled(0) will disable optical flow
//		// @Values: 0:Disabled,1:Enabled
//		// @User: Standard
//		PARAM(REAL32, "FLOW_ENABLE", &VAL_FLOW_ENABLE, CONFIG_FLOW_ENABLE),
//
//		// @Param: LOW_VOLT
//		// @DisplayName: Low Voltage
//		// @Description: Set this to the voltage you want to represent low voltage
//		// @Range: 0 20
//		// @Increment: .1
//		// @User: Standard
//		PARAM(REAL32, "LOW_VOLT", &VAL_LOW_VOLT, CONFIG_LOW_VOLT),
//
//		// @Param: SUPER_SIMPLE
//		// @DisplayName: Enable Super Simple Mode
//		// @Description: Setting this to Enabled(1) will enable Super Simple Mode. Setting this to Disabled(0) will disable Super Simple Mode
//		// @Values: 0:Disabled,1:Enabled
//		// @User: Standard
//		PARAM(REAL32, "SUPER_SIMPLE", &VAL_SUPER_SIMPLE, CONFIG_SUPER_SIMPLE),
//
//		// @Param: RTL_ALT_FINAL
//		// @DisplayName: RTL Final Altitude
//		// @Description: This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission.  Set to -1 to disable, zero to land.
//		// @Units: Centimeters
//		// @Range: -1 1000
//		// @Increment: 1
//		// @User: Standard
//		PARAM(REAL32, "RTL_ALT_FINAL", &VAL_RTL_ALT_FINAL, CONFIG_RTL_ALT_FINAL),
//
//		// @Param: TILT
//		// @DisplayName: Auto Tilt Compensation
//		// @Description: This is a feed-forward compensation which helps the aircraft achieve target waypoint speed.
//		// @Range: 0 100
//		// @Increment: 1
//		// @User: Advanced
//		PARAM(REAL32, "TILT", &VAL_TILT, CONFIG_TILT),
//
//		// @Param: BATT_VOLT_PIN
//		// @DisplayName: Battery Voltage sensing pin
//		// @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13.
//		// @Values: -1:Disabled, 0:A0, 1:A1, 13:A13
//		// @User: Standard
//		PARAM(REAL32, "BATT_VOLT_PIN", &VAL_BATT_VOLT_PIN, CONFIG_BATT_VOLT_PIN),
//
//		// @Param: BATT_CURR_PIN
//		// @DisplayName: Battery Current sensing pin
//		// @Description: Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13.
//		// @Values: -1:Disabled, 1:A1, 2:A2, 13:A13
//		// @User: Standard
//		PARAM(REAL32, "BATT_CURR_PIN", &VAL_BATT_CURR_PIN, CONFIG_BATT_CURR_PIN),
//
//		// @Param: RSSI_PIN
//		// @DisplayName: Receiver RSSI sensing pin
//		// @Description: This selects an analog pin for the receiver RSSI voltage. It assumes the voltage is 5V for max rssi, 0V for minimum
//		// @Values: -1:Disabled, 0:A0, 1:A1, 2:A2, 13:A13
//		// @User: Standard
//		PARAM(REAL32, "RSSI_PIN", &VAL_RSSI_PIN, CONFIG_RSSI_PIN),
//
//		// @Param: THR_ACC_ENABLE
//		// @DisplayName: Enable Accel based throttle controller
//		// @Description: This allows enabling and disabling the accelerometer based throttle controller.  If disabled a velocity based controller is used.
//		// @Values: 0:Disabled, 1:Enabled
//		// @User: Standard
//		PARAM(REAL32, "THR_ACC_ENABLE", &VAL_THR_ACC_ENABLE, CONFIG_THR_ACC_ENABLE),
//
//		// @Param: YAW_OVR_BEHAVE
//		// @DisplayName: Yaw override behaviour
//		// @Description: Controls when autopilot takes back normal control of yaw after pilot overrides
//		// @Values: 0:At Next WP, 1:On Mission Restart
//		// @User: Advanced
//		PARAM(REAL32, "YAW_OVR_BEHAVE", &VAL_YAW_OVR_BEHAVE, CONFIG_YAW_OVR_BEHAVE),
//
//		// @Param: WP_TOTAL
//		// @DisplayName: Waypoint Total
//		// @Description: Total number of commands in the mission stored in the eeprom.  Do not update this parameter directly!
//		// @User: Advanced
//		PARAM(REAL32, "WP_TOTAL", &VAL_WP_TOTAL, CONFIG_WP_TOTAL),
//
//		// @Param: WP_INDEX
//		// @DisplayName: Waypoint Index
//		// @Description: The index number of the command that is currently being executed.  Do not update this parameter directly!
//		// @User: Advanced
//		PARAM(REAL32, "WP_INDEX", &VAL_WP_INDEX, CONFIG_WP_INDEX),
//
//		// @Param: WP_RADIUS
//		// @DisplayName: Waypoint Radius
//		// @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
//		// @Units: Meters
//		// @Range: 1 127
//		// @Increment: 1
//		// @User: Standard
//		PARAM(REAL32, "WP_RADIUS", &VAL_WP_RADIUS, CONFIG_WP_RADIUS),
//
//		// @Param: CIRCLE_RADIUS
//		// @DisplayName: Circle radius
//		// @Description: Defines the radius of the circle the vehicle will fly when in Circle flight mode
//		// @Units: Meters
//		// @Range: 1 127
//		// @Increment: 1
//		// @User: Standard
//		PARAM(REAL32, "CIRCLE_RADIUS", &VAL_CIRCLE_RADIUS, CONFIG_CIRCLE_RADIUS),
//
//		// @Param: WP_SPEED_MAX
//		// @DisplayName: Waypoint Max Speed Target
//		// @Description: Defines the speed which the aircraft will attempt to maintain during a WP mission.
//		// @Units: Centimeters/Second
//		// @Increment: 100
//		// @User: Standard
//		PARAM(REAL32, "WP_SPEED_MAX", &VAL_WP_SPEED_MAX, CONFIG_WP_SPEED_MAX),
//
//		// @Param: XTRK_GAIN_SC
//		// @DisplayName: Cross-Track Gain
//		// @Description: This controls the rate that the Auto Controller will attempt to return original track
//		// @Units: Dimensionless
//		// @User: Standard
//		PARAM(REAL32, "XTRK_GAIN_SC", &VAL_XTRK_GAIN_SC, CONFIG_XTRK_GAIN_SC),
//
//		// @Param: XTRK_MIN_DIST
//		// @DisplayName: Crosstrack mininum distance
//		// @Description: Minimum distance in meters between waypoints to do crosstrack correction.
//		// @Units: Meters
//		// @Range: 0 32767
//		// @Increment: 1
//		// @User: Standard
//		PARAM(REAL32, "XTRK_MIN_DIST", &VAL_XTRK_MIN_DIST, CONFIG_XTRK_MIN_DIST),
//
//		// @Param: RTL_LOITER_TIME
//		// @DisplayName: RTL loiter time
//		// @Description: Time (in milliseconds) to loiter above home before begining final descent
//		// @Units: ms
//		// @Range: 0 60000
//		// @Increment: 1000
//		// @User: Standard
//		PARAM(REAL32, "RTL_LOIT_TIME", &VAL_RTL_LOIT_TIME, CONFIG_RTL_LOIT_TIME),
//
//		// @Param: LAND_SPEED
//		// @DisplayName: Land speed
//		// @Description: The descent speed for the final stage of landing in cm/s
//		// @Units: Centimeters/Second
//		// @Range: 10 200
//		// @Increment: 10
//		// @User: Standard
//		PARAM(REAL32, "LAND_SPEED", &VAL_LAND_SPEED, CONFIG_LAND_SPEED),
//
//		// @Param: AUTO_VELZ_MIN
//		// @DisplayName: Autopilot's min vertical speed (max descent) in cm/s
//		// @Description: The minimum vertical velocity (i.e. descent speed) the autopilot may request in cm/s
//		// @Units: Centimeters/Second
//		// @Range: -500 -50
//		// @Increment: 10
//		// @User: Standard
//		PARAM(REAL32, "AUTO_VELZ_MIN", &VAL_AUTO_VELZ_MIN, CONFIG_AUTO_VELZ_MIN),
//
//		// @Param: AUTO_VELZ_MAX
//		// @DisplayName: Auto pilot's max vertical speed in cm/s
//		// @Description: The maximum vertical velocity the autopilot may request in cm/s
//		// @Units: Centimeters/Second
//		// @Range: 50 500
//		// @Increment: 10
//		// @User: Standard
//		PARAM(REAL32, "AUTO_VELZ_MAX", &VAL_AUTO_VELZ_MAX, CONFIG_AUTO_VELZ_MAX),
//
//		// @Param: PILOT_VELZ_MAX
//		// @DisplayName: Pilot maximum vertical speed
//		// @Description: The maximum vertical velocity the pilot may request in cm/s
//		// @Units: Centimeters/Second
//		// @Range: 10 500
//		// @Increment: 10
//		// @User: Standard
//		PARAM(REAL32, "PILOT_VELZ_MAX", &VAL_PILOT_VELZ_MAX, CONFIG_PILOT_VELZ_MAX),
//
//		// @Param: THR_MIN
//		// @DisplayName: Minimum Throttle
//		// @Description: The minimum throttle that will be sent to the motors to keep them spinning
//		// @Units: ms
//		// @Range: 0 1000
//		// @Increment: 1
//		// @User: Standard
//		PARAM(REAL32, "THR_MIN", &VAL_THR_MIN, CONFIG_THR_MIN),
//
//		// @Param: THR_MAX
//		// @DisplayName: Maximum Throttle
//		// @Description: The maximum throttle that will be sent to the motors
//		// @Units: ms
//		// @Range: 0 1000
//		// @Increment: 1
//		// @User: Standard
//		PARAM(REAL32, "THR_MAX", &VAL_THR_MAX, CONFIG_THR_MAX),
//
//		// @Param: FS_THR_ENABLE
//		// @DisplayName: Throttle Failsafe Enable
//		// @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
//		// @Values: 0:Disabled,1:Enabled always RTL,2:Enabled Continue with Mission in Auto Mode
//		// @User: Standard
//		PARAM(REAL32, "FS_THR_ENABLE", &VAL_FS_THR_ENABLE, CONFIG_FS_THR_ENABLE),
//
//		// @Param: FS_THR_VALUE
//		// @DisplayName: Throttle Failsafe Value
//		// @Description: The PWM level on channel 3 below which throttle sailsafe triggers
//		// @User: Standard
//		PARAM(REAL32, "FS_THR_VALUE", &VAL_FS_THR_VALUE, CONFIG_FS_THR_VALUE),
//
//		// @Param: TRIM_THROTTLE
//		// @DisplayName: Throttle Trim
//		// @Description: The PWM level on channel 3 below which throttle sailsafe triggers
//		// @User: Standard
//		PARAM(REAL32, "TRIM_THROTTLE", &VAL_TRIM_THROTTLE, CONFIG_TRIM_THROTTLE),
//
//		// @Param: THR_MID
//		// @DisplayName: Throttle Mid Position
//		// @Description: The throttle output (0 ~ 1000) when throttle stick is in mid position.  Used to scale the manual throttle so that the mid throttle stick position is close to the throttle required to hover
//		// @User: Standard
//		// @Range: 300 700
//		// @Increment: 1
//		PARAM(REAL32, "THR_MID", &VAL_THR_MID, CONFIG_THR_MID),
//
//		// @Param: FLTMODE1
//		// @DisplayName: Flight Mode 1
//		// @Description: Flight mode when Channel 5 pwm is <= 1230
//		// @User: Standard
//		PARAM(REAL32, "FLTMODE1", &VAL_FLTMODE1, CONFIG_FLTMODE1),
//
//		// @Param: FLTMODE2
//		// @DisplayName: Flight Mode 2
//		// @Description: Flight mode when Channel 5 pwm is >1230, <= 1360
//		// @User: Standard
//		PARAM(REAL32, "FLTMODE2", &VAL_FLTMODE2, CONFIG_FLTMODE2),
//
//		// @Param: FLTMODE3
//		// @DisplayName: Flight Mode 3
//		// @Description: Flight mode when Channel 5 pwm is >1360, <= 1490
//		// @User: Standard
//		PARAM(REAL32, "FLTMODE3", &VAL_FLTMODE3, CONFIG_FLTMODE3),
//
//		// @Param: FLTMODE4
//		// @DisplayName: Flight Mode 4
//		// @Description: Flight mode when Channel 5 pwm is >1490, <= 1620
//		// @User: Standard
//		PARAM(REAL32, "FLTMODE4", &VAL_FLTMODE4, CONFIG_FLTMODE4),
//
//		// @Param: FLTMODE5
//		// @DisplayName: Flight Mode 5
//		// @Description: Flight mode when Channel 5 pwm is >1620, <= 1749
//		// @User: Standard
//		PARAM(REAL32, "FLTMODE5", &VAL_FLTMODE5, CONFIG_FLTMODE5),
//
//		// @Param: FLTMODE6
//		// @DisplayName: Flight Mode 6
//		// @Description: Flight mode when Channel 5 pwm is >=1750
//		// @User: Standard
//		PARAM(REAL32, "FLTMODE6", &VAL_FLTMODE6, CONFIG_FLTMODE6),
//
//		// @Param: SIMPLE
//		// @DisplayName: Simple mode bitmask
//		// @Description: Bitmask which holds which flight modes use simple heading mode (eg bit 0 = 1 means Flight Mode 0 uses simple mode)
//		// @User: Advanced
//		PARAM(REAL32, "SIMPLE", &VAL_SIMPLE, CONFIG_SIMPLE),
//
//		// @Param: LOG_BITMASK
//		// @DisplayName: Log bitmask
//		// @Description: 2 byte bitmap of log types to enable
//		// @User: Advanced
//		PARAM(REAL32, "LOG_BITMASK", &VAL_LOG_BITMASK, CONFIG_LOG_BITMASK),
//
//		// @Param: TOY_RATE
//		// @DisplayName: Toy Yaw Rate
//		// @Description: Controls yaw rate in Toy mode.  Higher values will cause a slower yaw rate.  Do not set to zero!
//		// @User: Advanced
//		// @Range: 1 10
//		PARAM(REAL32, "TOY_RATE", &VAL_TOY_RATE, CONFIG_TOY_RATE),
//
//		// @Param: ESC
//		// @DisplayName: ESC Calibration
//		// @Description: Controls whether ArduCopter will enter ESC calibration on the next restart.  Do not adjust this parameter manually.
//		// @User: Advanced
//		// @Values: 0:Normal Start-up,1:Start-up in ESC Calibration mode
//		PARAM(REAL32, "ESC", &VAL_ESC, CONFIG_ESC),
//
//		// @Param: TUNE
//		// @DisplayName: Channel 6 Tuning
//		// @Description: Controls which parameters (normally PID gains) are being tuned with transmitter's channel 6 knob
//		// @User: Standard
//		// @Values: 0:CH6_NONE,1:CH6_STABILIZE_KP,2:CH6_STABILIZE_KI,3:CH6_YAW_KP,4:CH6_RATE_KP,5:CH6_RATE_KI,6:CH6_YAW_RATE_KP,7:CH6_THROTTLE_KP,8:CH6_TOP_BOTTOM_RATIO,9:CH6_RELAY,10:CH6_TRAVERSE_SPEED,11:CH6_NAV_KP,12:CH6_LOITER_KP,13:CH6_HELI_EXTERNAL_GYRO,14:CH6_THR_HOLD_KP,17:CH6_OPTFLOW_KP,18:CH6_OPTFLOW_KI,19:CH6_OPTFLOW_KD,20:CH6_NAV_KI,21:CH6_RATE_KD,22:CH6_LOITER_RATE_KP,23:CH6_LOITER_RATE_KD,24:CH6_YAW_KI,25:CH6_ACRO_KP,26:CH6_YAW_RATE_KD,27:CH6_LOITER_KI,28:CH6_LOITER_RATE_KI,29:CH6_STABILIZE_KD,30:CH6_AHRS_YAW_KP,31:CH6_AHRS_KP,32:CH6_INAV_TC,33:CH6_THROTTLE_KI,34:CH6_THR_ACCEL_KP,35:CH6_THR_ACCEL_KI,36:CH6_THR_ACCEL_KD
//		PARAM(REAL32, "TUNE", &VAL_TUNE, CONFIG_TUNE),
//
//		// @Param: TUNE_LOW
//		// @DisplayName: Tuning minimum
//		// @Description: The minimum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
//		// @User: Standard
//		// @Range: 0 32767
//		PARAM(REAL32, "TUNE_LOW", &VAL_TUNE_LOW, CONFIG_TUNE_LOW),
//
//		// @Param: TUNE_HIGH
//		// @DisplayName: Tuning maximum
//		// @Description: The maximum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
//		// @User: Standard
//		// @Range: 0 32767
//		PARAM(REAL32, "TUNE_HIGH", &VAL_TUNE_HIGH, CONFIG_TUNE_HIGH),
//
//		// @Param: FRAME
//		// @DisplayName: Frame Orientation
//		// @Description: Congrols motor mixing The maximum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
//		// @User: Standard
//		// @Range: 0 32767
//		PARAM(REAL32, "FRAME", &VAL_FRAME, CONFIG_FRAME),
//
//		// @Param: CH7_OPT
//		// @DisplayName: Channel 7 option
//		// @Description: Select which function if performed when CH7 is above 1800 pwm
//		// @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 9:Camera Trigger
//		// @User: Standard
//		PARAM(REAL32, "CH7_OPT", &VAL_CH7_OPT, CONFIG_CH7_OPT),
//
//		// @Param: AUTO_SLEW
//		// @DisplayName: Auto Slew Rate
//		// @Description: This restricts the rate of change of the roll and pitch attitude commanded by the auto pilot
//		// @Units: Degrees/Second
//		// @Range: 1 45
//		// @Increment: 1
//		// @User: Advanced
//		PARAM(REAL32, "AUTO_SLEW", &VAL_AUTO_SLEW, CONFIG_AUTO_SLEW),
//
//		// RC channel
//		//-----------
//		// @Group: RC1_
//		// @Path: ../libraries/RC_Channel/RC_Channel.cpp
//	    // @Param: MIN
//	    // @DisplayName: RC min PWM
//	    // @Description: RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC1_MIN", &VAL_RC1_MIN, CONFIG_RC1_MIN),
//
//	    // @Param: TRIM
//	    // @DisplayName: RC trim PWM
//	    // @Description: RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC1_TRIM", &VAL_RC1_TRIM, CONFIG_RC1_TRIM),
//
//	    // @Param: MAX
//	    // @DisplayName: RC max PWM
//	    // @Description: RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC1_MAX", &VAL_RC1_MAX, CONFIG_RC1_MAX),
//
//	    // @Param: REV
//	    // @DisplayName: RC reverse
//	    // @Description: Reverse servo operation. Ignored on APM1 unless dip-switches are disabled.
//	    // @Values: -1:Reversed,1:Normal
//	    // @User: Advanced
//	    PARAM(REAL32, "RC1_REV", &VAL_RC1_REV, CONFIG_RC1_REV),
//
//	    // @Param: DZ
//	    // @DisplayName: RC dead-zone
//	    // @Description: dead zone around trim.
//	    // @User: Advanced
//	    PARAM(REAL32, "RC1_DZ", &VAL_RC1_DZ, CONFIG_RC1_DZ),
//
//	    // @Param: MIN
//	    // @DisplayName: RC min PWM
//	    // @Description: RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC2_MIN", &VAL_RC2_MIN, CONFIG_RC2_MIN),
//
//	    // @Param: TRIM
//	    // @DisplayName: RC trim PWM
//	    // @Description: RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC2_TRIM", &VAL_RC2_TRIM, CONFIG_RC2_TRIM),
//
//	    // @Param: MAX
//	    // @DisplayName: RC max PWM
//	    // @Description: RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC2_MAX", &VAL_RC2_MAX, CONFIG_RC2_MAX),
//
//	    // @Param: REV
//	    // @DisplayName: RC reverse
//	    // @Description: Reverse servo operation. Ignored on APM1 unless dip-switches are disabled.
//	    // @Values: -1:Reversed,1:Normal
//	    // @User: Advanced
//	    PARAM(REAL32, "RC2_REV", &VAL_RC2_REV, CONFIG_RC2_REV),
//
//	    // @Param: DZ
//	    // @DisplayName: RC dead-zone
//	    // @Description: dead zone around trim.
//	    // @User: Advanced
//	    PARAM(REAL32, "RC2_DZ", &VAL_RC2_DZ, CONFIG_RC2_DZ),
//
//	    // @Param: MIN
//	    // @DisplayName: RC min PWM
//	    // @Description: RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC3_MIN", &VAL_RC3_MIN, CONFIG_RC3_MIN),
//
//	    // @Param: TRIM
//	    // @DisplayName: RC trim PWM
//	    // @Description: RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC3_TRIM", &VAL_RC3_TRIM, CONFIG_RC3_TRIM),
//
//	    // @Param: MAX
//	    // @DisplayName: RC max PWM
//	    // @Description: RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC3_MAX", &VAL_RC3_MAX, CONFIG_RC3_MAX),
//
//	    // @Param: REV
//	    // @DisplayName: RC reverse
//	    // @Description: Reverse servo operation. Ignored on APM1 unless dip-switches are disabled.
//	    // @Values: -1:Reversed,1:Normal
//	    // @User: Advanced
//	    PARAM(REAL32, "RC3_REV", &VAL_RC3_REV, CONFIG_RC3_REV),
//
//	    // @Param: DZ
//	    // @DisplayName: RC dead-zone
//	    // @Description: dead zone around trim.
//	    // @User: Advanced
//	    PARAM(REAL32, "RC3_DZ", &VAL_RC3_DZ, CONFIG_RC3_DZ),
//
//	    // @Param: MIN
//	    // @DisplayName: RC min PWM
//	    // @Description: RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC4_MIN", &VAL_RC4_MIN, CONFIG_RC4_MIN),
//
//	    // @Param: TRIM
//	    // @DisplayName: RC trim PWM
//	    // @Description: RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC4_TRIM", &VAL_RC4_TRIM, CONFIG_RC4_TRIM),
//
//	    // @Param: MAX
//	    // @DisplayName: RC max PWM
//	    // @Description: RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC4_MAX", &VAL_RC4_MAX, CONFIG_RC4_MAX),
//
//	    // @Param: REV
//	    // @DisplayName: RC reverse
//	    // @Description: Reverse servo operation. Ignored on APM1 unless dip-switches are disabled.
//	    // @Values: -1:Reversed,1:Normal
//	    // @User: Advanced
//	    PARAM(REAL32, "RC4_REV", &VAL_RC4_REV, CONFIG_RC4_REV),
//
//	    // @Param: DZ
//	    // @DisplayName: RC dead-zone
//	    // @Description: dead zone around trim.
//	    // @User: Advanced
//	    PARAM(REAL32, "RC4_DZ", &VAL_RC4_DZ, CONFIG_RC4_DZ),
//
//	    // @Group: RC5_
//		// @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
//	    // @Param: MIN
//	    // @DisplayName: RC min PWM
//	    // @Description: RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC5_MIN", &VAL_RC5_MIN, CONFIG_RC5_MIN),
//
//	    // @Param: TRIM
//	    // @DisplayName: RC trim PWM
//	    // @Description: RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC5_TRIM", &VAL_RC5_TRIM, CONFIG_RC5_TRIM),
//
//	    // @Param: MAX
//	    // @DisplayName: RC max PWM
//	    // @Description: RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC5_MAX", &VAL_RC5_MAX, CONFIG_RC5_MAX),
//
//	    // @Param: REV
//	    // @DisplayName: RC reverse
//	    // @Description: Reverse servo operation. Ignored on APM1 unless dip-switches are disabled.
//	    // @Values: -1:Reversed,1:Normal
//	    // @User: Advanced
//	    PARAM(REAL32, "RC5_REV", &VAL_RC5_REV, CONFIG_RC5_REV),
//
//	    // @Param: DZ
//	    // @DisplayName: RC dead-zone
//	    // @Description: dead zone around trim.
//	    // @User: Advanced
//	    PARAM(REAL32, "RC5_DZ", &VAL_RC5_DZ, CONFIG_RC5_DZ),
//
//	    // @Param: FUNCTION
//	    // @DisplayName: Servo out function
//	    // @Description: Setting this to Disabled(0) will disable this output, any other value will enable the corresponding function
//	    // @Values: 0:Disabled,1:Manual,2:Flap,3:Flap_auto,4:Aileron,5:flaperon,6:mount_pan,7:mount_tilt,8:mount_roll,9:mount_open,10:camera_trigger,11:release,12:mount2_pan,13:mount2_tilt,14:mount2_roll,15:mount2_open,16:DifferentialSpoiler1,17:DifferentialSpoiler2,18:AileronWithInput
//	    // @User: Standard
//	    PARAM(REAL32, "RC5_FUNCTION", &VAL_RC5_FUNCTION, CONFIG_RC5_FUNCTION),
//		// @Group: RC6_
//		// @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
//	    // @Param: MIN
//	    // @DisplayName: RC min PWM
//	    // @Description: RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC6_MIN", &VAL_RC6_MIN, CONFIG_RC6_MIN),
//
//	    // @Param: TRIM
//	    // @DisplayName: RC trim PWM
//	    // @Description: RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC6_TRIM", &VAL_RC6_TRIM, CONFIG_RC6_TRIM),
//
//	    // @Param: MAX
//	    // @DisplayName: RC max PWM
//	    // @Description: RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC6_MAX", &VAL_RC6_MAX, CONFIG_RC6_MAX),
//
//	    // @Param: REV
//	    // @DisplayName: RC reverse
//	    // @Description: Reverse servo operation. Ignored on APM1 unless dip-switches are disabled.
//	    // @Values: -1:Reversed,1:Normal
//	    // @User: Advanced
//	    PARAM(REAL32, "RC6_REV", &VAL_RC6_REV, CONFIG_RC6_REV),
//
//	    // @Param: DZ
//	    // @DisplayName: RC dead-zone
//	    // @Description: dead zone around trim.
//	    // @User: Advanced
//	    PARAM(REAL32, "RC6_DZ", &VAL_RC6_DZ, CONFIG_RC6_DZ),
//
//	    // @Param: FUNCTION
//	    // @DisplayName: Servo out function
//	    // @Description: Setting this to Disabled(0) will disable this output, any other value will enable the corresponding function
//	    // @Values: 0:Disabled,1:Manual,2:Flap,3:Flap_auto,4:Aileron,5:flaperon,6:mount_pan,7:mount_tilt,8:mount_roll,9:mount_open,10:camera_trigger,11:release,12:mount2_pan,13:mount2_tilt,14:mount2_roll,15:mount2_open,16:DifferentialSpoiler1,17:DifferentialSpoiler2,18:AileronWithInput
//	    // @User: Standard
//	    PARAM(REAL32, "RC6_FUNCTION", &VAL_RC6_FUNCTION, CONFIG_RC6_FUNCTION),
//		// @Group: RC7_
//		// @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
//	    // @Param: MIN
//	    // @DisplayName: RC min PWM
//	    // @Description: RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC7_MIN", &VAL_RC7_MIN, CONFIG_RC7_MIN),
//
//	    // @Param: TRIM
//	    // @DisplayName: RC trim PWM
//	    // @Description: RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC7_TRIM", &VAL_RC7_TRIM, CONFIG_RC7_TRIM),
//
//	    // @Param: MAX
//	    // @DisplayName: RC max PWM
//	    // @Description: RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC7_MAX", &VAL_RC7_MAX, CONFIG_RC7_MAX),
//
//	    // @Param: REV
//	    // @DisplayName: RC reverse
//	    // @Description: Reverse servo operation. Ignored on APM1 unless dip-switches are disabled.
//	    // @Values: -1:Reversed,1:Normal
//	    // @User: Advanced
//	    PARAM(REAL32, "RC7_REV", &VAL_RC7_REV, CONFIG_RC7_REV),
//
//	    // @Param: DZ
//	    // @DisplayName: RC dead-zone
//	    // @Description: dead zone around trim.
//	    // @User: Advanced
//	    PARAM(REAL32, "RC7_DZ", &VAL_RC7_DZ, CONFIG_RC7_DZ),
//
//	    // @Param: FUNCTION
//	    // @DisplayName: Servo out function
//	    // @Description: Setting this to Disabled(0) will disable this output, any other value will enable the corresponding function
//	    // @Values: 0:Disabled,1:Manual,2:Flap,3:Flap_auto,4:Aileron,5:flaperon,6:mount_pan,7:mount_tilt,8:mount_roll,9:mount_open,10:camera_trigger,11:release,12:mount2_pan,13:mount2_tilt,14:mount2_roll,15:mount2_open,16:DifferentialSpoiler1,17:DifferentialSpoiler2,18:AileronWithInput
//	    // @User: Standard
//	    PARAM(REAL32, "RC7_FUNCTION", &VAL_RC7_FUNCTION, CONFIG_RC7_FUNCTION),
//		// @Group: RC8_
//		// @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
//	    // @Param: MIN
//	    // @DisplayName: RC min PWM
//	    // @Description: RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC8_MIN", &VAL_RC8_MIN, CONFIG_RC8_MIN),
//
//	    // @Param: TRIM
//	    // @DisplayName: RC trim PWM
//	    // @Description: RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC8_TRIM", &VAL_RC8_TRIM, CONFIG_RC8_TRIM),
//
//	    // @Param: MAX
//	    // @DisplayName: RC max PWM
//	    // @Description: RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC8_MAX", &VAL_RC8_MAX, CONFIG_RC8_MAX),
//
//	    // @Param: REV
//	    // @DisplayName: RC reverse
//	    // @Description: Reverse servo operation. Ignored on APM1 unless dip-switches are disabled.
//	    // @Values: -1:Reversed,1:Normal
//	    // @User: Advanced
//	    PARAM(REAL32, "RC8_REV", &VAL_RC8_REV, CONFIG_RC8_REV),
//
//	    // @Param: DZ
//	    // @DisplayName: RC dead-zone
//	    // @Description: dead zone around trim.
//	    // @User: Advanced
//	    PARAM(REAL32, "RC8_DZ", &VAL_RC8_DZ, CONFIG_RC8_DZ),
//
//	    // @Param: FUNCTION
//	    // @DisplayName: Servo out function
//	    // @Description: Setting this to Disabled(0) will disable this output, any other value will enable the corresponding function
//	    // @Values: 0:Disabled,1:Manual,2:Flap,3:Flap_auto,4:Aileron,5:flaperon,6:mount_pan,7:mount_tilt,8:mount_roll,9:mount_open,10:camera_trigger,11:release,12:mount2_pan,13:mount2_tilt,14:mount2_roll,15:mount2_open,16:DifferentialSpoiler1,17:DifferentialSpoiler2,18:AileronWithInput
//	    // @User: Standard
//	    PARAM(REAL32, "RC8_FUNCTION", &VAL_RC8_FUNCTION, CONFIG_RC8_FUNCTION),
//
//		// @Group: RC10_
//		// @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
//	    // @Param: MIN
//	    // @DisplayName: RC min PWM
//	    // @Description: RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC10_MIN", &VAL_RC10_MIN, CONFIG_RC10_MIN),
//
//	    // @Param: TRIM
//	    // @DisplayName: RC trim PWM
//	    // @Description: RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC10_TRIM", &VAL_RC10_TRIM, CONFIG_RC10_TRIM),
//
//	    // @Param: MAX
//	    // @DisplayName: RC max PWM
//	    // @Description: RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC10_MAX", &VAL_RC10_MAX, CONFIG_RC10_MAX),
//
//	    // @Param: REV
//	    // @DisplayName: RC reverse
//	    // @Description: Reverse servo operation. Ignored on APM1 unless dip-switches are disabled.
//	    // @Values: -1:Reversed,1:Normal
//	    // @User: Advanced
//	    PARAM(REAL32, "RC10_REV", &VAL_RC10_REV, CONFIG_RC10_REV),
//
//	    // @Param: DZ
//	    // @DisplayName: RC dead-zone
//	    // @Description: dead zone around trim.
//	    // @User: Advanced
//	    PARAM(REAL32, "RC10_DZ", &VAL_RC10_DZ, CONFIG_RC10_DZ),
//
//	    // @Param: FUNCTION
//	    // @DisplayName: Servo out function
//	    // @Description: Setting this to Disabled(0) will disable this output, any other value will enable the corresponding function
//	    // @Values: 0:Disabled,1:Manual,2:Flap,3:Flap_auto,4:Aileron,5:flaperon,6:mount_pan,7:mount_tilt,8:mount_roll,9:mount_open,10:camera_trigger,11:release,12:mount2_pan,13:mount2_tilt,14:mount2_roll,15:mount2_open,16:DifferentialSpoiler1,17:DifferentialSpoiler2,18:AileronWithInput
//	    // @User: Standard
//	    PARAM(REAL32, "RC10_FUNCTION", &VAL_RC10_FUNCTION, CONFIG_RC10_FUNCTION),
//
//		// @Group: RC11_
//		// @Path: ../libraries/RC_Channel/RC_Channel_aux.cpp
//	    // @Param: MIN
//	    // @DisplayName: RC min PWM
//	    // @Description: RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC11_MIN", &VAL_RC11_MIN, CONFIG_RC11_MIN),
//
//	    // @Param: TRIM
//	    // @DisplayName: RC trim PWM
//	    // @Description: RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC11_TRIM", &VAL_RC11_TRIM, CONFIG_RC11_TRIM),
//
//	    // @Param: MAX
//	    // @DisplayName: RC max PWM
//	    // @Description: RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
//	    // @Units: ms
//	    // @Range: 800 2200
//	    // @Increment: 1
//	    // @User: Advanced
//	    PARAM(REAL32, "RC11_MAX", &VAL_RC11_MAX, CONFIG_RC11_MAX),
//
//	    // @Param: REV
//	    // @DisplayName: RC reverse
//	    // @Description: Reverse servo operation. Ignored on APM1 unless dip-switches are disabled.
//	    // @Values: -1:Reversed,1:Normal
//	    // @User: Advanced
//	    PARAM(REAL32, "RC11_REV", &VAL_RC11_REV, CONFIG_RC11_REV),
//
//	    // @Param: DZ
//	    // @DisplayName: RC dead-zone
//	    // @Description: dead zone around trim.
//	    // @User: Advanced
//	    PARAM(REAL32, "RC11_DZ", &VAL_RC11_DZ, CONFIG_RC11_DZ),
//
//	    // @Param: FUNCTION
//	    // @DisplayName: Servo out function
//	    // @Description: Setting this to Disabled(0) will disable this output, any other value will enable the corresponding function
//	    // @Values: 0:Disabled,1:Manual,2:Flap,3:Flap_auto,4:Aileron,5:flaperon,6:mount_pan,7:mount_tilt,8:mount_roll,9:mount_open,10:camera_trigger,11:release,12:mount2_pan,13:mount2_tilt,14:mount2_roll,15:mount2_open,16:DifferentialSpoiler1,17:DifferentialSpoiler2,18:AileronWithInput
//	    // @User: Standard
//	    PARAM(REAL32, "RC11_FUNCTION", &VAL_RC11_FUNCTION, CONFIG_RC11_FUNCTION),
//
//		// @Param: RC_SPEED
//		// @DisplayName: ESC Update Speed
//		// @Description: This is the speed in Hertz that your ESCs will receive updates
//		// @Units: Hertz (Hz)
//		// @Values: 125,400,490
//		// @User: Advanced
//		PARAM(REAL32, "RC_SPEED", &VAL_RC_SPEED, CONFIG_RC_SPEED),
//
//		// @Param: ACRO_P
//		// @DisplayName: Acro P gain
//		// @Description: Used to convert pilot roll, pitch and yaw input into a dssired rate of rotation in ACRO mode.  Higher values mean faster rate of rotation.
//		// @Range: 1 10
//		// @User: Standard
//		PARAM(REAL32, "ACRO_P", &VAL_ACRO_P, CONFIG_ACRO_P),
//
//		// @Param: AXIS_ENABLE
//		// @DisplayName: Acro Axis
//		// @Description: Used to control whether acro mode actively maintains the current angle when control sticks are released (Enabled = maintains current angle)
//		// @Values: 0:Disabled, 1:Enabled
//		// @User: Standard
//		PARAM(REAL32, "AXIS_ENABLE", &VAL_AXIS_ENABLE, CONFIG_AXIS_ENABLE),
//
//		// @Param: ACRO_BAL_ROLL
//		// @DisplayName: Acro Balance Roll
//		// @Description: rate at which roll angle returns to level in acro mode
//		// @Range: 0 300
//		// @Increment: 1
//		// @User: Advanced
//		PARAM(REAL32, "ACRO_BAL_ROLL", &VAL_ACRO_BAL_ROLL, CONFIG_ACRO_BAL_ROLL),
//
//		// @Param: ACRO_BAL_PITCH
//		// @DisplayName: Acro Balance Pitch
//		// @Description: rate at which pitch angle returns to level in acro mode
//		// @Range: 0 300
//		// @Increment: 1
//		// @User: Advanced
//		PARAM(REAL32, "ACRO_BAL_PITCH", &VAL_ACRO_BAL_PITCH, CONFIG_ACRO_BAL_PITCH),
//
//		// @Param: ACRO_TRAINER
//		// @DisplayName: Acro Trainer Enabled
//		// @Description: Set to 1 (Enabled) to make roll return to within 45 degrees of level automatically
//		// @Values: 0:Disabled,1:Enabled
//		// @User: Advanced
//		PARAM(REAL32, "ACRO_TRAINER", &VAL_ACRO_TRAINER, CONFIG_ACRO_TRAINER),
//
//		// @Param: LED_MODE
//		// @DisplayName: Copter LED Mode
//		// @Description: bitmap to control the copter led mode
//		// @Values: 0:Disabled,1:Enable,2:GPS On,4:Aux,8:Buzzer,16:Oscillate,32:Nav Blink,64:GPS Nav Blink
//		// @User: Standard
//		PARAM(REAL32, "LED_MODE", &VAL_LED_MODE, CONFIG_LED_MODE),
//
//		// PID controller
//		//---------------
//	    // @Param: P
//	    // @DisplayName: PID Proportional Gain
//	    // @Description: P Gain which produces an output value that is proportional to the current error value
//	    PARAM(REAL32, "RATE_RLL_P", &VAL_RATE_RLL_P, CONFIG_RATE_RLL_P),
//	    // @Param: I
//	    // @DisplayName: PID Integral Gain
//	    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
//	    PARAM(REAL32, "RATE_RLL_I", &VAL_RATE_RLL_I, CONFIG_RATE_RLL_I),
//	    // @Param: D
//	    // @DisplayName: PID Derivative Gain
//	    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
//	    PARAM(REAL32, "RATE_RLL_D", &VAL_RATE_RLL_D, CONFIG_RATE_RLL_D),
//	    // @Param: IMAX
//	    // @DisplayName: PID Integral Maximum
//	    // @Description: The maximum/minimum value that the I term can output
//	    PARAM(REAL32, "RATE_RLL_IMAX", &VAL_RATE_RLL_IMAX, CONFIG_RATE_RLL_IMAX),
//	    // @Param: P
//	    // @DisplayName: PID Proportional Gain
//	    // @Description: P Gain which produces an output value that is proportional to the current error value
//	    PARAM(REAL32, "RATE_PIT_P", &VAL_RATE_PIT_P, CONFIG_RATE_PIT_P),
//	    // @Param: I
//	    // @DisplayName: PID Integral Gain
//	    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
//	    PARAM(REAL32, "RATE_PIT_I", &VAL_RATE_PIT_I, CONFIG_RATE_PIT_I),
//	    // @Param: D
//	    // @DisplayName: PID Derivative Gain
//	    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
//	    PARAM(REAL32, "RATE_PIT_D", &VAL_RATE_PIT_D, CONFIG_RATE_PIT_D),
//	    // @Param: IMAX
//	    // @DisplayName: PID Integral Maximum
//	    // @Description: The maximum/minimum value that the I term can output
//	    PARAM(REAL32, "RATE_PIT_IMAX", &VAL_RATE_PIT_IMAX, CONFIG_RATE_PIT_IMAX),
//	    // @Param: P
//	    // @DisplayName: PID Proportional Gain
//	    // @Description: P Gain which produces an output value that is proportional to the current error value
//	    PARAM(REAL32, "RATE_YAW_P", &VAL_RATE_YAW_P, CONFIG_RATE_YAW_P),
//	    // @Param: I
//	    // @DisplayName: PID Integral Gain
//	    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
//	    PARAM(REAL32, "RATE_YAW_I", &VAL_RATE_YAW_I, CONFIG_RATE_YAW_I),
//	    // @Param: D
//	    // @DisplayName: PID Derivative Gain
//	    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
//	    PARAM(REAL32, "RATE_YAW_D", &VAL_RATE_YAW_D, CONFIG_RATE_YAW_D),
//	    // @Param: IMAX
//	    // @DisplayName: PID Integral Maximum
//	    // @Description: The maximum/minimum value that the I term can output
//	    PARAM(REAL32, "RATE_YAW_IMAX", &VAL_RATE_YAW_IMAX, CONFIG_RATE_YAW_IMAX),
//
//
//	    // @Param: P
//	    // @DisplayName: PID Proportional Gain
//	    // @Description: P Gain which produces an output value that is proportional to the current error value
//	    PARAM(REAL32, "LOITER_LAT_P", &VAL_LOITER_LAT_P, CONFIG_LOITER_LAT_P),
//	    // @Param: I
//	    // @DisplayName: PID Integral Gain
//	    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
//	    PARAM(REAL32, "LOITER_LAT_I", &VAL_LOITER_LAT_I, CONFIG_LOITER_LAT_I),
//	    // @Param: D
//	    // @DisplayName: PID Derivative Gain
//	    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
//	    PARAM(REAL32, "LOITER_LAT_D", &VAL_LOITER_LAT_D, CONFIG_LOITER_LAT_D),
//	    // @Param: IMAX
//	    // @DisplayName: PID Integral Maximum
//	    // @Description: The maximum/minimum value that the I term can output
//	    PARAM(REAL32, "LOITER_LAT_IMAX", &VAL_LOITER_LAT_IMAX, CONFIG_LOITER_LAT_IMAX),
//	    // @Param: P
//	    // @DisplayName: PID Proportional Gain
//	    // @Description: P Gain which produces an output value that is proportional to the current error value
//	    PARAM(REAL32, "LOITER_LON_P", &VAL_LOITER_LON_P, CONFIG_LOITER_LON_P),
//	    // @Param: I
//	    // @DisplayName: PID Integral Gain
//	    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
//	    PARAM(REAL32, "LOITER_LON_I", &VAL_LOITER_LON_I, CONFIG_LOITER_LON_I),
//	    // @Param: D
//	    // @DisplayName: PID Derivative Gain
//	    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
//	    PARAM(REAL32, "LOITER_LON_D", &VAL_LOITER_LON_D, CONFIG_LOITER_LON_D),
//	    // @Param: IMAX
//	    // @DisplayName: PID Integral Maximum
//	    // @Description: The maximum/minimum value that the I term can output
//	    PARAM(REAL32, "LOITER_LON_IMAX", &VAL_LOITER_LON_IMAX, CONFIG_LOITER_LON_IMAX),
//
//	    // @Param: P
//	    // @DisplayName: PID Proportional Gain
//	    // @Description: P Gain which produces an output value that is proportional to the current error value
//	    PARAM(REAL32, "NAT_LAT_P", &VAL_NAT_LAT_P, CONFIG_NAT_LAT_P),
//	    // @Param: I
//	    // @DisplayName: PID Integral Gain
//	    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
//	    PARAM(REAL32, "NAT_LAT_I", &VAL_NAT_LAT_I, CONFIG_NAT_LAT_I),
//	    // @Param: D
//	    // @DisplayName: PID Derivative Gain
//	    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
//	    PARAM(REAL32, "NAT_LAT_D", &VAL_NAT_LAT_D, CONFIG_NAT_LAT_D),
//	    // @Param: IMAX
//	    // @DisplayName: PID Integral Maximum
//	    // @Description: The maximum/minimum value that the I term can output
//	    PARAM(REAL32, "NAT_LAT_IMAX", &VAL_NAT_LAT_IMAX, CONFIG_NAT_LAT_IMAX),
//	    // @Param: P
//	    // @DisplayName: PID Proportional Gain
//	    // @Description: P Gain which produces an output value that is proportional to the current error value
//	    PARAM(REAL32, "NAT_LON_P", &VAL_NAT_LON_P, CONFIG_NAT_LON_P),
//	    // @Param: I
//	    // @DisplayName: PID Integral Gain
//	    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
//	    PARAM(REAL32, "NAT_LON_I", &VAL_NAT_LON_I, CONFIG_NAT_LON_I),
//	    // @Param: D
//	    // @DisplayName: PID Derivative Gain
//	    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
//	    PARAM(REAL32, "NAT_LON_D", &VAL_NAT_LON_D, CONFIG_NAT_LON_D),
//	    // @Param: IMAX
//	    // @DisplayName: PID Integral Maximum
//	    // @Description: The maximum/minimum value that the I term can output
//	    PARAM(REAL32, "NAT_LON_IMAX", &VAL_NAT_LON_IMAX, CONFIG_NAT_LON_IMAX),
//
//	    // @Param: P
//	    // @DisplayName: PID Proportional Gain
//	    // @Description: P Gain which produces an output value that is proportional to the current error value
//	    PARAM(REAL32, "THR_RATE_P", &VAL_THR_RATE_P, CONFIG_THR_RATE_P),
//	    // @Param: I
//	    // @DisplayName: PID Integral Gain
//	    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
//	    PARAM(REAL32, "THR_RATE_I", &VAL_THR_RATE_I, CONFIG_THR_RATE_I),
//	    // @Param: D
//	    // @DisplayName: PID Derivative Gain
//	    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
//	    PARAM(REAL32, "THR_RATE_D", &VAL_THR_RATE_D, CONFIG_THR_RATE_D),
//	    // @Param: IMAX
//	    // @DisplayName: PID Integral Maximum
//	    // @Description: The maximum/minimum value that the I term can output
//	    PARAM(REAL32, "THR_RATE_IMAX", &VAL_THR_RATE_IMAX, CONFIG_THR_RATE_IMAX),
//	    // @Param: P
//	    // @DisplayName: PID Proportional Gain
//	    // @Description: P Gain which produces an output value that is proportional to the current error value
//	    PARAM(REAL32, "THR_ACCEL_P", &VAL_THR_ACCEL_P, CONFIG_THR_ACCEL_P),
//	    // @Param: I
//	    // @DisplayName: PID Integral Gain
//	    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
//	    PARAM(REAL32, "THR_ACCEL_I", &VAL_THR_ACCEL_I, CONFIG_THR_ACCEL_I),
//	    // @Param: D
//	    // @DisplayName: PID Derivative Gain
//	    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
//	    PARAM(REAL32, "THR_ACCEL_D", &VAL_THR_ACCEL_D, CONFIG_THR_ACCEL_D),
//	    // @Param: IMAX
//	    // @DisplayName: PID Integral Maximum
//	    // @Description: The maximum/minimum value that the I term can output
//	    PARAM(REAL32, "THR_ACCEL_IMAX", &VAL_THR_ACCEL_IMAX, CONFIG_THR_ACCEL_IMAX),
//
//	    // @Param: P
//	    // @DisplayName: PID Proportional Gain
//	    // @Description: P Gain which produces an output value that is proportional to the current error value
//	    PARAM(REAL32, "OF_RLL_P", &VAL_OF_RLL_P, CONFIG_OF_RLL_P),
//	    // @Param: I
//	    // @DisplayName: PID Integral Gain
//	    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
//	    PARAM(REAL32, "OF_RLL_I", &VAL_OF_RLL_I, CONFIG_OF_RLL_I),
//	    // @Param: D
//	    // @DisplayName: PID Derivative Gain
//	    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
//	    PARAM(REAL32, "OF_RLL_D", &VAL_OF_RLL_D, CONFIG_OF_RLL_D),
//	    // @Param: IMAX
//	    // @DisplayName: PID Integral Maximum
//	    // @Description: The maximum/minimum value that the I term can output
//	    PARAM(REAL32, "OF_RLL_IMAX", &VAL_OF_RLL_IMAX, CONFIG_OF_RLL_IMAX),
//	    // @Param: P
//	    // @DisplayName: PID Proportional Gain
//	    // @Description: P Gain which produces an output value that is proportional to the current error value
//	    PARAM(REAL32, "OF_PIT_P", &VAL_OF_PIT_P, CONFIG_OF_PIT_P),
//	    // @Param: I
//	    // @DisplayName: PID Integral Gain
//	    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
//	    PARAM(REAL32, "OF_PIT_I", &VAL_OF_PIT_I, CONFIG_OF_PIT_I),
//	    // @Param: D
//	    // @DisplayName: PID Derivative Gain
//	    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
//	    PARAM(REAL32, "OF_PIT_D", &VAL_OF_PIT_D, CONFIG_OF_PIT_D),
//	    // @Param: IMAX
//	    // @DisplayName: PID Integral Maximum
//	    // @Description: The maximum/minimum value that the I term can output
//	    PARAM(REAL32, "OF_PIT_IMAX", &VAL_OF_PIT_IMAX, CONFIG_OF_PIT_IMAX),
//
//		// PI controller
//		//--------------
//	    PARAM(REAL32, "STB_RLL_P", &VAL_STB_RLL_P, CONFIG_STB_RLL_P),
//	    PARAM(REAL32, "STB_RLL_I", &VAL_STB_RLL_I, CONFIG_STB_RLL_I),
//	    PARAM(REAL32, "STB_RLL_IMAX", &VAL_STB_RLL_IMAX, CONFIG_STB_RLL_IMAX),
//	    PARAM(REAL32, "STB_PIT_P", &VAL_STB_PIT_P, CONFIG_STB_PIT_P),
//	    PARAM(REAL32, "STB_PIT_I", &VAL_STB_PIT_I, CONFIG_STB_PIT_I),
//	    PARAM(REAL32, "STB_PIT_IMAX", &VAL_STB_PIT_IMAX, CONFIG_STB_PIT_IMAX),
//	    PARAM(REAL32, "STB_YAW_P", &VAL_STB_YAW_P, CONFIG_STB_YAW_P),
//	    PARAM(REAL32, "STB_YAW_I", &VAL_STB_YAW_I, CONFIG_STB_YAW_I),
//	    PARAM(REAL32, "STB_YAW_IMAX", &VAL_STB_YAW_IMAX, CONFIG_STB_YAW_IMAX),
//
//	    PARAM(REAL32, "THR_ALT_P", &VAL_THR_ALT_P, CONFIG_THR_ALT_P),
//	    PARAM(REAL32, "THR_ALT_I", &VAL_THR_ALT_I, CONFIG_THR_ALT_I),
//	    PARAM(REAL32, "THR_ALT_IMAX", &VAL_THR_ALT_IMAX, CONFIG_THR_ALT_IMAX),
//	    PARAM(REAL32, "HLD_LAT_P", &VAL_HLD_LAT_P, CONFIG_HLD_LAT_P),
//	    PARAM(REAL32, "HLD_LAT_I", &VAL_HLD_LAT_I, CONFIG_HLD_LAT_I),
//	    PARAM(REAL32, "HLD_LAT_IMAX", &VAL_HLD_LAT_IMAX, CONFIG_HLD_LAT_IMAX),
//	    PARAM(REAL32, "HLD_LON_P", &VAL_HLD_LON_P, CONFIG_HLD_LON_P),
//	    PARAM(REAL32, "HLD_LON_I", &VAL_HLD_LON_I, CONFIG_HLD_LON_I),
//	    PARAM(REAL32, "HLD_LON_IMAX", &VAL_HLD_LON_IMAX, CONFIG_HLD_LON_IMAX)

		// variables not in the g class which contain EEPROM saved variables

		// variables not in the g class which contain EEPROM saved variables
//#if CAMERA == ENABLED
//		// @Group: CAM_
//		// @Path: ../libraries/AP_Camera/AP_Camera.cpp
//		GOBJECT(camera,           "CAM_", AP_Camera),
//#endif

		// @Group: COMPASS_
		// @Path: ../libraries/AP_Compass/Compass.cpp
//		GOBJECT(compass,        "COMPASS_", Compass),

		// @Group: INS_
		// @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
//#if HIL_MODE == HIL_MODE_DISABLED
//		GOBJECT(ins,            "INS_", AP_InertialSensor),
//#endif

//#if INERTIAL_NAV_XY == ENABLED || INERTIAL_NAV_Z == ENABLED
//		// @Group: INAV_
//		// @Path: ../libraries/AP_InertialNav/AP_InertialNav.cpp
//		GOBJECT(inertial_nav,           "INAV_",    AP_InertialNav),
//#endif

//		GOBJECT(gcs0,                   "SR0_",     GCS_MAVLINK),
//		GOBJECT(gcs3,                   "SR3_",     GCS_MAVLINK),

		// @Group: AHRS_
		// @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
//		GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

//#if MOUNT == ENABLED
//		// @Group: MNT_
//		// @Path: ../libraries/AP_Mount/AP_Mount.cpp
//		GOBJECT(camera_mount,           "MNT_", AP_Mount),
//#endif

//#if MOUNT2 == ENABLED
//		// @Group: MNT2_
//		// @Path: ../libraries/AP_Mount/AP_Mount.cpp
//		GOBJECT(camera_mount2,           "MNT2_",       AP_Mount),
//#endif

//#if AP_LIMITS == ENABLED
//		//@Group: LIM_
//		//@Path: ../libraries/AP_Limits/AP_Limits.cpp,../libraries/AP_Limits/AP_Limit_GPSLock.cpp, ../libraries/AP_Limits/AP_Limit_Geofence.cpp, ../libraries/AP_Limits/AP_Limit_Altitude.cpp, ../libraries/AP_Limits/AP_Limit_Module.cpp
//		GOBJECT(limits,                 "LIM_",    AP_Limits),
//		GOBJECT(gpslock_limit,          "LIM_",    AP_Limit_GPSLock),
//		GOBJECT(geofence_limit,         "LIM_",    AP_Limit_Geofence),
//		GOBJECT(altitude_limit,         "LIM_",    AP_Limit_Altitude),
//#endif

//		GOBJECT(motors, "MOT_",         AP_Motors),
};


} /* namespace test */
