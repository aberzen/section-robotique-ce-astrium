/*
 * System.cpp
 *
 *  Created on: 23 mai 2015
 *      Author: Modélisme
 */


#include <gcs/channel/mavlink_bridge.hpp>

#include "System.hpp"

#include <infra/rtos/Task.hpp>
#include <system/params/NrdGen.hpp>


#define PARAM(type, name, addr, defVal) { MAV_PARAM_TYPE_##type, name, addr, {type : defVal} }
#define USB_MUX_PIN 23


namespace system {


const autom::Modulator::Parameter paramMod = {
		K_MOD_INFLMAT,
		/* frcMaxPos_B */
		K_MOD_FMAXPOS,
		/* frcMaxNeg_B */
		K_MOD_FMAXNEG,
		/* trqMaxPos_B */
		K_MOD_TMAXPOS,
		/* trqMaxNeg_B */
		K_MOD_TMAXNEG
};

#ifdef MODULATORPINV
autom::ModulatorPinv::DescentVector paramModModDesc = {};
autom::ModulatorPinv::PInvInflMat paramModModPInv = {};
#else
const autom::ModulatorLut::Parameter paramModLut =
{
	/* lutTrq */
	K_MOD_LUTTRQ,
	/* lutFrc */
	K_MOD_LUTFRC
};

#endif

const attitude::AttitudeManager::Parameter attMgrParam =
{
		/* ctrl */
		{
				/* ctrl */
				{
						/* axes */
						{
								{
										/* Kp */
										K_ATTCTRL_KP_0,
										/* Kd */
										K_ATTCTRL_KD_0,
										/* Ki; */
										K_ATTCTRL_KI_0,
										/* maxI */
										K_ATTCTRL_MAXI_0
								},
								{
										/* Kp */
										K_ATTCTRL_KP_1,
										/* Kd */
										K_ATTCTRL_KD_1,
										/* Ki; */
										K_ATTCTRL_KI_1,
										/* maxI */
										K_ATTCTRL_MAXI_1
								},
								{
										/* Kp */
										K_ATTCTRL_KP_2,
										/* Kd */
										K_ATTCTRL_KD_2,
										/* Ki; */
										K_ATTCTRL_KI_2,
										/* maxI */
										K_ATTCTRL_MAXI_2
								}
						}
				},
				/* maxCosAngOverTwoErr */
				K_ATTCTRL_MAXERRCOS,
				/* maxSinAngOverTwoErr */
				K_ATTCTRL_MAXERRSIN
		}
} ;

const navigation::NavigationManager::Parameter navMgrParam =
{
		/* paramCtrl */
		{
				/* paramCtrl */
				{
						/* axes */
						{
								{
										/* Kp */
										K_NAVCTRL_KP_0,
										/* Kd */
										K_NAVCTRL_KD_0,
										/* Ki; */
										K_NAVCTRL_KI_0,
										/* maxI */
										K_NAVCTRL_MAXI_0
								},
								{
										/* Kp */
										K_NAVCTRL_KP_1,
										/* Kd */
										K_NAVCTRL_KD_1,
										/* Ki; */
										K_NAVCTRL_KI_1,
										/* maxI */
										K_NAVCTRL_MAXI_1
								},
								{
										/* Kp */
										K_NAVCTRL_KP_2,
										/* Kd */
										K_NAVCTRL_KD_2,
										/* Ki; */
										K_NAVCTRL_KI_2,
										/* maxI */
										K_NAVCTRL_MAXI_2
								}
						}
				},
				/* mass */
				1.6,
		},
		/* paramDirectThrust */
		{
				/* unitThrust */
				{0, 0, 1755546}
		},
} ;

const hw::Radio::Parameter paramRadio =
{
		/* reversed */
		{0},
		/* pwmZero */
		{(MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1, (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1},
		/* pwmMin */
		{MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH, MIN_PULSEWIDTH},
		/* pwmMax */
		{MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH, MAX_PULSEWIDTH}
};

/** @brief Handlers for COM0 interrupts */
SerialHandler(0);


System::System()
: dataPool(),
  _mode(E_SYS_MODE_NONE),
  _buffRx0(&_buffRx0_buffer[0], SERIAL0_RX_LEN),
  _buffTx0(&_buffTx0_buffer[0], SERIAL0_TX_LEN),
  _spiBus(),
  _i2c(),
  _imu(
		  _spiBus,
		  hw::HalImuMpu6000::E_GYR_CNF_1000DPS,
		  hw::HalImuMpu6000::E_ACC_CNF_4G,
		  hw::HalImuMpu6000::E_UPT_FREQ_500HZ),
  _baro(_spiBus),
  _compass(_i2c),
  SerialPortConstParam(_com0, 0, _buffRx0, _buffTx0),
  _pwm(),
  _radio(paramRadio),
  _paramMgt(info, paramCount),
  _mavChannel(MAVLINK_COMM_0,_com0),
  _gcs(MAVLINK_COMM_0),
  _estimator(),
#ifdef MODULATORPINV
#else
  _modulator(
			paramMod,
			paramModLut),
#endif
  _attMgr(attMgrParam),
  _navMgr(navMgrParam),
  _timerArmMgt(0)
{

}

System::~System()
{

}

void System::initialize()
{
	_paramMgt.loadAllValues();

    /* Disable magnetometer */
	pinMode(63, OUTPUT);
	digitalWrite(63, HIGH);

	/* Short pause to ensure SPI reset */
	infra::Task::delay(10);

	_spiBus.initialize();

	/* Short pause to ensure SPI reset */
	infra::Task::delay(10);

	_i2c.begin();
	_i2c.setSpeed(false);
	// on the APM2 board we have a mux thet switches UART0 between
	// USB and the board header. If the right ArduPPM firmware is
	// installed we can detect if USB is connected using the
	// USB_MUX_PIN
	pinMode(USB_MUX_PIN, INPUT);

	bool usb_connected = !digitalRead(USB_MUX_PIN);

	if (usb_connected)
		_com0.open(115200);
	else
		_com0.open(57600);

	/* Set clock diviser */
	_spiBus.setClockDivider(SPI_CLOCK_DIV32); // 2MHZ SPI rate

	/* Short pause to ensure SPI reset */
	infra::Task::delay(10);

	_imu.initialize();
	_baro.initialize();

	/* Short pause to ensure SPI reset */
	infra::Task::delay(10);
	/* Set clock diviser */
	_spiBus.setClockDivider(SPI_CLOCK_DIV16); // 2MHZ SPI rate

	/* Short pause to ensure SPI reset */
	infra::Task::delay(10);
	_compass.initialize();

	/* pwm */
	_pwm.initialize();

	/* Estimator */
	_estimator.initialize();

	/* motors */
	_modulator.initialize();

	/* GCS */
	_gcs.initialize();

}

void System::execute()
{
	switch (_mode)
	{
	case E_SYS_MODE_INIT:
		/* Execute init */
		executeInitMode();
		break;
	case E_SYS_MODE_READY:
		/* Execute ready */
		executeReadyMode();
		break;
	case E_SYS_MODE_ARMED:
		/* Execute ready */
		executeArmedMode();
		break;
	case E_SYS_MODE_FAILSAFE:
		/* Execute failsafe */
		executeFailsafeMode();
		break;
	case E_SYS_MODE_NONE:
	default:
		/* Switch to init mode */
		setMode(E_SYS_MODE_INIT);
		break;
	}
}

/** @brief Execute init mode */
void System::executeInitMode()
{
	/* Process GCS new messages */
	_gcs.processNewMessages();

	/* Process sensors */
	processRawSensors();

	/* Post process sensors */
	postProcessSensors();

	/* Process estimation */
	processEstimation();

	/* Process GCS services */
	_gcs.processServices();

	/* Auto switch to ready */
	_timerArmMgt++;
	if (_timerArmMgt >= 1000)
		setMode(E_SYS_MODE_READY);
}

/** @brief Execute ready mode */
void System::executeReadyMode()
{
	/* Process GCS new messages */
	_gcs.processNewMessages();

	/* Process sensors */
	processRawSensors();

	/* Process radio */
	processRadio();

	/* Post process sensors */
	postProcessSensors();

	/* Process estimation */
	processEstimation();

	/* Process actuator */
	processActuators();

	/* Process GCS services */
	_gcs.processServices();


	/* Check if arm order is send */
	bool verifCond = true;
//	char message[50];
//	sprintf(message,"%d %d %d %d",
//			_radio.getSigned(RADIO_IDX_ROLL),
//			_radio.getSigned(RADIO_IDX_PITCH),
//			_radio.getUnsigned(RADIO_IDX_THRUST),
//			_radio.getSigned(RADIO_IDX_YAW));
//	mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_INFO, message);
	verifCond = verifCond && (math_abs(_radio.getSigned(RADIO_IDX_ROLL))<100);
	verifCond = verifCond && (math_abs(_radio.getSigned(RADIO_IDX_PITCH))<100);
	verifCond = verifCond && (_radio.getUnsigned(RADIO_IDX_THRUST)<100);
	verifCond = verifCond && ((_radio.getSignedMaxVal(RADIO_IDX_YAW)-100)<=_radio.getSigned(RADIO_IDX_YAW));
	// TODO add control mode verification (today only stabilized)

	/* Timer management */
	if (verifCond)
		_timerArmMgt++;
	else
		_timerArmMgt = 0;

	/* Check mode transition */
	if (_timerArmMgt >= CNF_TIMER_ARM)
		setMode(E_SYS_MODE_ARMED);

}

/** @brief Execute armed mode */
void System::executeArmedMode()
{
	/* Process GCS new messages */
	_gcs.processNewMessages();

	/* Process sensors */
	processRawSensors();

	/* Process radio */
	processRadio();

	/* Post process sensors */
	postProcessSensors();

	/* Process estimation */
	processEstimation();

	/* Process Navigation */
	processNavigation();

	/* Process Attitude */
	processAttitude();

	/* Process actuator */
	processActuators();

	/* Process GCS services */
	_gcs.processServices();


	/* Check transitions conditions */
	bool verifCond = true;
//	char message[50];
//	sprintf(message,"%d %d %d %d",
//			_radio.getSigned(RADIO_IDX_ROLL),
//			_radio.getSigned(RADIO_IDX_PITCH),
//			_radio.getUnsigned(RADIO_IDX_THRUST),
//			_radio.getSigned(RADIO_IDX_YAW));
//	mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_INFO, message);
	verifCond = verifCond && (math_abs(_radio.getSigned(RADIO_IDX_ROLL))<100);
	verifCond = verifCond && (math_abs(_radio.getSigned(RADIO_IDX_PITCH))<100);
	verifCond = verifCond && (_radio.getUnsigned(RADIO_IDX_THRUST)<100);
	verifCond = verifCond && (math_abs(_radio.getSigned(RADIO_IDX_YAW))<100);
	// TODO add control mode verification (today only stabilized)

	/* Timer management */
	if (verifCond)
		_timerArmMgt++;
	else
		_timerArmMgt = 0;

	/* Check mode transition */
	if (_timerArmMgt >= CNF_TIMER_ARM)
	{
		setMode(E_SYS_MODE_READY);
	}
}

/** @brief Execute fail safe mode */
void System::executeFailsafeMode()
{

}

/** @Brief Set new mode */
bool System::setMode(Mode mode)
{
	bool result = false;

	/* Check with current mode */
	if (_mode == mode)
		return true;

	switch (mode)
	{
	case E_SYS_MODE_INIT:
		/* Switch to initialization */
		result = switchToInitMode();
		break;
	case E_SYS_MODE_READY:
		/* Switch to ready */
		result = switchToReadyMode();
		break;
	case E_SYS_MODE_ARMED:
		/* Switch to ready */
		result = switchToArmedMode();
		break;
	case E_SYS_MODE_FAILSAFE:
		/* Switch to failsafe */
		result = switchToFailsafeMode();
		break;
	case E_SYS_MODE_NONE:
	default:
		/* Do nothing, invalid mode */
		break;
	}

	/* If transition accepted, switch to mode */
	if (result)
		_mode = mode;

	/* Return result */
	return result;
}


/** @brief Switch to init mode */
bool System::switchToInitMode()
{
	return true;
}

/** @brief Switch to ready mode */
bool System::switchToReadyMode()
{
	/* reset counter */
	_timerArmMgt = 0;

	return true;
}

/** @brief Switch to armed mode */
bool System::switchToArmedMode()
{
	bool result = true;

	/* Switch the attitude and nav to demanded modes */
	result &= _attMgr.getGuidanceManager().setMode(attitude::AttitudeGuidanceManager::E_MODE_AUTOSTAB_NOYAW);
	result &= _navMgr.setMode(navigation::NavigationManager::E_NAV_MODE_DIRECTTHRUST);

	if (result)
	{
		/* Arm motors */
		armMotor();

	}
	/* reset counter */
	_timerArmMgt = 0;

	return result;
}


/** @brief Switch to fail safe mode */
bool System::switchToFailsafeMode()
{
	/* Set commanded force and torque to zero */
	dataPool.ctrlTrqDemB(0,0,0);
	dataPool.ctrlFrcDemB(0,0,0);

	/* Disarm motors */
	disarmMotor();

	return true;
}



/** @brief Process sensors (raw) */
void System::processRawSensors()
{
	uint32_t now = micros();

	// Sample compass
	if (_compass.sample(dataPool.compassMagRaw_U))
	{
		dataPool.compassLastMeasDateUsec = now;
	}

	// Sample barometer
	if (_baro.sample(dataPool.baroPressRaw_U, dataPool.baroTempRaw_U))
	{
		dataPool.baroLastMeasDateUsec = now;
	}

	// Sample imu1
	if (_imu.sample(dataPool.imuRateRaw_U, dataPool.imuAccRaw_U,dataPool.imuTemp))
	{
		dataPool.imuLastMeasDateUsec = now;
	}
}

/** @brief Process RC (raw) */
void System::processRadio()
{
	_pwm.read(&dataPool.pwm_inputs[0]);
}

/** @brief Post process sensors */
void System::postProcessSensors()
{
//	char buffer[100];

	/* IMU gyro */
	dataPool.imuRate_B(
			((float) dataPool.imuRateRaw_U.x) * GYRO_SCALE_1000DPS,
			((float) dataPool.imuRateRaw_U.y) * GYRO_SCALE_1000DPS,
			((float) dataPool.imuRateRaw_U.z) * GYRO_SCALE_1000DPS);

	/* IMU acco */
	dataPool.imuAcc_B(
			((float) dataPool.imuAccRaw_U.x) * ACC_SCALE_4G,
			((float) dataPool.imuAccRaw_U.y) * ACC_SCALE_4G,
			((float) dataPool.imuAccRaw_U.z) * ACC_SCALE_4G);

	/* Compass */
	dataPool.compassMag_B(
			((float) (dataPool.compassMagRaw_U.x - 61)) * HALMAGHMC5883L_LSB_1_30GA,
			((float) (dataPool.compassMagRaw_U.y + 27)) * HALMAGHMC5883L_LSB_1_30GA,
			((float) (dataPool.compassMagRaw_U.z -  3)) * HALMAGHMC5883L_LSB_1_30GA);
}

/** @brief Process estimation */
void System::processEstimation()
{
	_estimator.update();
}

/** @brief Process actuators */
void System::processActuators()
{
	if (_radio.getUnsigned(RADIO_IDX_THRUST) < 50)
	{
		/* Inhibit motors when radio thrust is set to zero */
		for (uint8_t iMotor=0 ; iMotor<CNF_NB_MOTORS ; iMotor++)
		{
			dataPool.pwm_outputs[iMotor] = MIN_PULSEWIDTH;
		}
	}
	else
	{
		/* Compute motor inputs */
		_modulator.calcMotorCommand(
				dataPool.ctrlFrcDemB,
				dataPool.ctrlTrqDemB,
				&dataPool.pwm_outputs[0]);
	}

	/* write PWM outputs */
	_pwm.write(&dataPool.pwm_outputs[0]);

	/* Compute efforts produced by the modulator */
	_modulator.calcTorsor(
			&dataPool.pwm_outputs[0],
			dataPool.estForce_B,
			dataPool.estTorque_B);
}

/** @brief Process attitude controller */
void System::processAttitude()
{
	_attMgr.execute();
}

/** @brief Process navigation controller */
void System::processNavigation()
{
	_navMgr.execute();
}

/** @brief Arm motors */
void System::armMotor()
{
	/* Arm motors */
	for (uint8_t idx=0 ; idx<CNF_NB_MOTORS ; idx++)
	{
		_pwm.enable_out(idx);
	}
}

/** @brief Disarm motors */
void System::disarmMotor()
{
	/* Set out values to min PWM values and disarm */
	for (uint8_t iMotor=0 ; iMotor<CNF_NB_MOTORS ; iMotor++)
	{
		dataPool.pwm_outputs[iMotor] = MIN_PULSEWIDTH;
		_pwm.write(iMotor, dataPool.pwm_outputs[iMotor]);
		_pwm.disable_out(iMotor);
	}
}


PROGMEM const mavlink::ParameterMgt::ParamInfo info[] = {
		PARAM(UINT16,"RD_MIN_0", (void*)&paramRadio.pwmMin[0], MIN_PULSEWIDTH),
		PARAM(UINT16,"RD_MIN_1", (void*)&paramRadio.pwmMin[1], MIN_PULSEWIDTH),
		PARAM(UINT16,"RD_MIN_2", (void*)&paramRadio.pwmMin[2], MIN_PULSEWIDTH),
		PARAM(UINT16,"RD_MIN_3", (void*)&paramRadio.pwmMin[3], MIN_PULSEWIDTH),
		PARAM(UINT16,"RD_MIN_4", (void*)&paramRadio.pwmMin[4], MIN_PULSEWIDTH),
		PARAM(UINT16,"RD_MIN_5", (void*)&paramRadio.pwmMin[5], MIN_PULSEWIDTH),
		PARAM(UINT16,"RD_MIN_6", (void*)&paramRadio.pwmMin[6], MIN_PULSEWIDTH),
		PARAM(UINT16,"RD_MIN_7", (void*)&paramRadio.pwmMin[7], MIN_PULSEWIDTH),
		PARAM(UINT16,"RD_MAX_0", (void*)&paramRadio.pwmMax[0], MAX_PULSEWIDTH),
		PARAM(UINT16,"RD_MAX_1", (void*)&paramRadio.pwmMax[1], MAX_PULSEWIDTH),
		PARAM(UINT16,"RD_MAX_2", (void*)&paramRadio.pwmMax[2], MAX_PULSEWIDTH),
		PARAM(UINT16,"RD_MAX_3", (void*)&paramRadio.pwmMax[3], MAX_PULSEWIDTH),
		PARAM(UINT16,"RD_MAX_4", (void*)&paramRadio.pwmMax[4], MAX_PULSEWIDTH),
		PARAM(UINT16,"RD_MAX_5", (void*)&paramRadio.pwmMax[5], MAX_PULSEWIDTH),
		PARAM(UINT16,"RD_MAX_6", (void*)&paramRadio.pwmMax[6], MAX_PULSEWIDTH),
		PARAM(UINT16,"RD_MAX_7", (void*)&paramRadio.pwmMax[7], MAX_PULSEWIDTH),
		PARAM(UINT16,"RD_ZERO_0", (void*)&paramRadio.pwmZero[0], (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1),
		PARAM(UINT16,"RD_ZERO_1", (void*)&paramRadio.pwmZero[1], (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1),
		PARAM(UINT16,"RD_ZERO_2", (void*)&paramRadio.pwmZero[2], (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1),
		PARAM(UINT16,"RD_ZERO_3", (void*)&paramRadio.pwmZero[3], (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1),
		PARAM(UINT16,"RD_ZERO_4", (void*)&paramRadio.pwmZero[4], (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1),
		PARAM(UINT16,"RD_ZERO_5", (void*)&paramRadio.pwmZero[5], (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1),
		PARAM(UINT16,"RD_ZERO_6", (void*)&paramRadio.pwmZero[6], (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1),
		PARAM(UINT16,"RD_ZERO_7", (void*)&paramRadio.pwmZero[7], (MAX_PULSEWIDTH+MIN_PULSEWIDTH)>>1),
		PARAM(UINT16,"RD_REVERS", (void*)&paramRadio.reversed, 0)
};
uint16_t paramCount = 25;


System system = System();


} /* namespace system */
