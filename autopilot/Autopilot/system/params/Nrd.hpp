/*
 * nrd.h
 *
 *  Created on: 17 janv. 2013
 *      Author: Aberzen
 */


//#include <math/Matrix3.hpp>

#ifndef NRD_H_
#define NRD_H_

#define SCALE_INFLUENCE_MATRIX  (10)
#define SCALE_TORSOR            (10)
#define SCALE_ATT_CTRL_GAIN     (15)

#define CNF_PWM_NUM_FROM_DEVICE     (8)
#define CNF_PWM_NUM_TO_DEVICE       (11)

#define CNF_NB_MOTORS     (4)

/* **************************************************** */
/* Physics */

#define PHYSICS_GRAVITY 		(9.81) 	/* [m/s^2] gravity acceleration norm */
#define PHYSICS_MASS 			(1.600) /* [Kg] mass */
#define PHYSICS_INERTIA_XX 		(0.0229623) /* [Kg.m^2] Diagonal inertia XX */
#define PHYSICS_INERTIA_YY 		(0.0448513) /* [Kg.m^2] Diagonal inertia YY */
#define PHYSICS_INERTIA_ZZ 		(0.0617154) /* [Kg.m^2] Diagonal inertia ZZ */

/* **************************************************** */
/* Global FSW parameter */

#define FSW_SYSTEM_TICK_PER_MSEC            (1) 	/* tick/ms */
#define FSW_TASK_CTRL_PERIOD_TICK           (10)	/* tick */
#define FSW_TASK_CTRL_PERIOD_TICK_PER_MSEC  (FSW_TASK_CTRL_PERIOD_TICK*FSW_SYSTEM_TICK_PER_MSEC)	/* tick/period/ms */
#define FSW_TASK_CTRL_PERIOD_TICK_PER_SEC   (((float)(FSW_TASK_CTRL_PERIOD_TICK*FSW_SYSTEM_TICK_PER_MSEC))/1000.)	/* tick/period/ms */

/* **************************************************** */
/* Attitude controller: angular attitude with rate bias */

#define CTRL_ATT_X_Kp		(2.9967466)
#define CTRL_ATT_X_Kd		(0.3147850)
#define CTRL_ATT_X_Ki		(0.)
#define CTRL_ATT_X_maxI		(0.)

#define CTRL_ATT_Y_Kp		(5.8534217)
#define CTRL_ATT_Y_Kd		(0.6148567)
#define CTRL_ATT_Y_Ki		(0.)
#define CTRL_ATT_Y_maxI		(0.)

#define CTRL_ATT_Z_Kp		(0.2707139)
#define CTRL_ATT_Z_Kd		(0.1809589)
#define CTRL_ATT_Z_Ki		(0.)
#define CTRL_ATT_Z_maxI		(0.)

/* **************************************************** */
/* Estimator: gain and other parameters for Kalman filter */

#define EST_GAIN_ACCO_ANGLE	( 0.029321531433505)  /* gainAcco / angle */
#define EST_GAIN_ACCO_DRIFT	( 0.00000043864908449)  /* gainAcco / drift */
#define EST_GAIN_COMPASS_ANGLE	( 0.029321531433505)  /* gainAcco / angle */
#define EST_GAIN_COMPASS_DRIFT	( 0.00000043864908449)  /* gainAcco / drift */

#define EST_MAX_CONSECUTIVE_MISSING_MEAS_GYRO (3)
#define EST_MAX_CONSECUTIVE_MISSING_MEAS_ACCO (3)
#define EST_MAX_CONSECUTIVE_MISSING_MEAS_COMPASS (3)
#define EST_MAX_CONSECUTIVE_MISSING_MEAS_BARO (3)
#define EST_MAX_CONSECUTIVE_MISSING_MEAS_GPS (3)

/* **************************************************** */
/* MAVLINK configuration */

#define CONFIG_MAVLINK_EEPROMREV	(0x200A)

#include <avr/pgmspace.h>
#include <gcs/param/ParameterMgt.hpp>

namespace system {

extern PROGMEM const mavlink::ParameterMgt::ParamInfo info[];
extern uint16_t paramCount;

}

#endif /* NRD_H_ */
