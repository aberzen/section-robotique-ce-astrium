/*
 * MavServiceManager.cpp
 *
 *  Created on: 2 juin 2013
 *      Author: Aberzen
 */

#include "../include/MavServiceManager.hpp"
#include <Arduino.h>
#include <hw/serial/include/FastSerial.hpp>

namespace mavlink {

MavServiceManager::MavServiceManager(
		mavlink_channel_t port,
		uint8_t target_system,
		enum MAV_COMPONENT target_component,
		ModeMgt& modeMgt,
		ParameterMgt& paramMgt,
		WaypointMgt& wpMgt,
		HouseKeepingMgt& hkMgt,
		CommandMgt& cmdMgt,
		MountMgt& mountMgt,
		FenceMgt& fenceMgt,
		DigivcamMgt& digivcamMgt) :
				_port(port),
				_state(STATE_IDLE),
				_target_system(target_system),
				_target_component(target_component),
				_prevHeartbeatDate(0),
				_modeMgt(modeMgt),
				_paramMgt(paramMgt),
				_wpMgt(wpMgt),
				_hkMgt(hkMgt),
				_cmdMgt(cmdMgt),
				_mountMgt(mountMgt),
				_fenceMgt(fenceMgt),
				_digivcamMgt(digivcamMgt)
{
	_prevHeartbeatDate = millis();
}

MavServiceManager::~MavServiceManager()
{

}

/** @brief Execute the process */
infra::status MavServiceManager::initialize()
{
	return 0;
}

/** @brief Execute the process */
infra::status MavServiceManager::execute() {
	/* First look at message reception */
	{
		Channel* channel;
		mavlink_message_t msg;
		channel = Channel::getChannel(_port);
		if (channel != NULL)
		{
			// Check new messages
			if (channel->receiveMessage(&msg))
			{
				handleMessage(&msg);
			}
		}
	}

	/* Second process current transaction if any */
	{
		switch (_state)
		{
		case STATE_PARAM_REQUEST_LIST:
		{
			char param_id[MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN+1];
			_paramMgt.getInfo(_paramRequestListIdx,NULL,&param_id[0]);
			paramRequestRead(_paramRequestListIdx, _target_system, _target_component,&param_id[0]);
			/* Point to next parameter */
			_paramRequestListIdx++;

			if (_paramRequestListIdx>=_paramMgt.getParamCount())
			{
				/* Ended */
				_paramRequestListIdx = 0;

				/* Set to IDLE */
				_state = STATE_IDLE;
			}

			break;
		}
		case STATE_IDLE:
		default:
		{
			/* Nothing to do but heartbeat */
			uint16_t now = millis();
//			Serial.print("millis() == ");
//			Serial.print(now);
//			Serial.println();

			if ((now-_prevHeartbeatDate)>1000)
			{
//				Serial.print("Sending heartbeat\n");
				/* Emit a heartbeat */
				mavlink_msg_heartbeat_send(
						_port,
						(uint8_t) MAV_TYPE_QUADROTOR,
						(uint8_t) MAV_AUTOPILOT_GENERIC,
						(uint8_t) MAV_MODE_FLAG_TEST_ENABLED,
						(uint8_t) 0,
						(uint8_t) MAV_STATE_STANDBY
						);
				_prevHeartbeatDate = now;
			}
			else
			{
				/* Execute House keeping management service */
				_hkMgt.execute();
			}
			break;
		}
		}
	}
	return 0;
}

void MavServiceManager::handleMessage(mavlink_message_t* msg)
{
	switch (msg->msgid)
	{

	case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
	{
		requestDataStream(
				mavlink_msg_request_data_stream_get_req_message_rate(msg),
				mavlink_msg_request_data_stream_get_target_system(msg),
				mavlink_msg_request_data_stream_get_target_component(msg),
				mavlink_msg_request_data_stream_get_req_stream_id(msg),
				mavlink_msg_request_data_stream_get_start_stop(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_COMMAND_LONG:
	{
		commandLong(
				mavlink_msg_command_long_get_param1(msg),
				mavlink_msg_command_long_get_param2(msg),
				mavlink_msg_command_long_get_param3(msg),
				mavlink_msg_command_long_get_param4(msg),
				mavlink_msg_command_long_get_param5(msg),
				mavlink_msg_command_long_get_param6(msg),
				mavlink_msg_command_long_get_param7(msg),
				mavlink_msg_command_long_get_command(msg),
				mavlink_msg_command_long_get_target_system(msg),
				mavlink_msg_command_long_get_target_component(msg),
				mavlink_msg_command_long_get_confirmation(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_SET_MODE:
	{

		setMode(
				mavlink_msg_set_mode_get_custom_mode(msg),
				mavlink_msg_set_mode_get_target_system(msg),
				mavlink_msg_set_mode_get_base_mode(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
	{
		missionRequestList(
				mavlink_msg_mission_request_list_get_target_system(msg),
				mavlink_msg_mission_request_list_get_target_component(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_MISSION_REQUEST:
	{
		missionRequest(
				mavlink_msg_mission_request_get_seq(msg),
				mavlink_msg_mission_request_get_target_system(msg),
				mavlink_msg_mission_request_get_target_component(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_MISSION_ACK:
	{
		missionAck(
				mavlink_msg_mission_ack_get_target_system(msg),
				mavlink_msg_mission_ack_get_target_component(msg),
				mavlink_msg_mission_ack_get_type(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
	{
		paramRequestList(
				mavlink_msg_param_request_list_get_target_system(msg),
				mavlink_msg_param_request_list_get_target_component(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
	{
		char param_id[MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN+1];
		mavlink_msg_param_request_read_get_param_id(msg, param_id);
		paramRequestRead(
				mavlink_msg_param_request_read_get_param_index(msg),
				mavlink_msg_param_request_read_get_target_system(msg),
				mavlink_msg_param_request_read_get_target_component(msg),
				param_id
		);
		break;
	}
	case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
	{
		missionClearAll(
				mavlink_msg_mission_clear_all_get_target_system(msg),
				mavlink_msg_mission_clear_all_get_target_component(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
	{
		missionSetCurrent(
				mavlink_msg_mission_set_current_get_seq(msg),
				mavlink_msg_mission_set_current_get_target_system(msg),
				mavlink_msg_mission_set_current_get_target_component(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_MISSION_COUNT:
	{
		missionCount(
				mavlink_msg_mission_count_get_count(msg),
				mavlink_msg_mission_count_get_target_system(msg),
				mavlink_msg_mission_count_get_target_component(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_SET_MAG_OFFSETS:
	{
		setMagOffsets(
				mavlink_msg_set_mag_offsets_get_mag_ofs_x(msg),
				mavlink_msg_set_mag_offsets_get_mag_ofs_y(msg),
				mavlink_msg_set_mag_offsets_get_mag_ofs_z(msg),
				mavlink_msg_set_mag_offsets_get_target_system(msg),
				mavlink_msg_set_mag_offsets_get_target_component(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_MISSION_ITEM:
	{
		missionItem(
				mavlink_msg_mission_item_get_param1(msg),
				mavlink_msg_mission_item_get_param2(msg),
				mavlink_msg_mission_item_get_param3(msg),
				mavlink_msg_mission_item_get_param4(msg),
				mavlink_msg_mission_item_get_x(msg),
				mavlink_msg_mission_item_get_y(msg),
				mavlink_msg_mission_item_get_z(msg),
				mavlink_msg_mission_item_get_seq(msg),
				mavlink_msg_mission_item_get_command(msg),
				mavlink_msg_mission_item_get_target_system(msg),
				mavlink_msg_mission_item_get_target_component(msg),
				mavlink_msg_mission_item_get_frame(msg),
				mavlink_msg_mission_item_get_current(msg),
				mavlink_msg_mission_item_get_autocontinue(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_PARAM_SET:
	{
		char param_id[MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN+1];
		mavlink_msg_param_set_get_param_id(msg, param_id);
		paramSet(
				mavlink_msg_param_set_get_param_value(msg),
				mavlink_msg_param_set_get_target_system(msg),
				mavlink_msg_param_set_get_target_component(msg),
				param_id,
				mavlink_msg_param_set_get_param_type(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
	{
		rcChannelsOverride(
				mavlink_msg_rc_channels_override_get_chan1_raw(msg),
				mavlink_msg_rc_channels_override_get_chan2_raw(msg),
				mavlink_msg_rc_channels_override_get_chan3_raw(msg),
				mavlink_msg_rc_channels_override_get_chan4_raw(msg),
				mavlink_msg_rc_channels_override_get_chan5_raw(msg),
				mavlink_msg_rc_channels_override_get_chan6_raw(msg),
				mavlink_msg_rc_channels_override_get_chan7_raw(msg),
				mavlink_msg_rc_channels_override_get_chan8_raw(msg),
				mavlink_msg_rc_channels_override_get_target_system(msg),
				mavlink_msg_rc_channels_override_get_target_component(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_HIL_STATE:
	{
		hilState(
				mavlink_msg_hil_state_get_time_usec(msg),
				mavlink_msg_hil_state_get_roll(msg),
				mavlink_msg_hil_state_get_pitch(msg),
				mavlink_msg_hil_state_get_yaw(msg),
				mavlink_msg_hil_state_get_rollspeed(msg),
				mavlink_msg_hil_state_get_pitchspeed(msg),
				mavlink_msg_hil_state_get_yawspeed(msg),
				mavlink_msg_hil_state_get_lat(msg),
				mavlink_msg_hil_state_get_lon(msg),
				mavlink_msg_hil_state_get_alt(msg),
				mavlink_msg_hil_state_get_vx(msg),
				mavlink_msg_hil_state_get_vy(msg),
				mavlink_msg_hil_state_get_vz(msg),
				mavlink_msg_hil_state_get_xacc(msg),
				mavlink_msg_hil_state_get_yacc(msg),
				mavlink_msg_hil_state_get_zacc(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_HEARTBEAT:
	{
		heartbeat(
				mavlink_msg_heartbeat_get_custom_mode(msg),
				mavlink_msg_heartbeat_get_type(msg),
				mavlink_msg_heartbeat_get_autopilot(msg),
				mavlink_msg_heartbeat_get_base_mode(msg),
				mavlink_msg_heartbeat_get_system_status(msg),
				mavlink_msg_heartbeat_get_mavlink_version(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_RAW_IMU:
	{
		rawImu(
				mavlink_msg_raw_imu_get_time_usec(msg),
				mavlink_msg_raw_imu_get_xacc(msg),
				mavlink_msg_raw_imu_get_yacc(msg),
				mavlink_msg_raw_imu_get_zacc(msg),
				mavlink_msg_raw_imu_get_xgyro(msg),
				mavlink_msg_raw_imu_get_ygyro(msg),
				mavlink_msg_raw_imu_get_zgyro(msg),
				mavlink_msg_raw_imu_get_xmag(msg),
				mavlink_msg_raw_imu_get_ymag(msg),
				mavlink_msg_raw_imu_get_zmag(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_RAW_PRESSURE:
	{
		rawPressure(
				mavlink_msg_raw_pressure_get_time_usec(msg),
				mavlink_msg_raw_pressure_get_press_abs(msg),
				mavlink_msg_raw_pressure_get_press_diff1(msg),
				mavlink_msg_raw_pressure_get_press_diff2(msg),
				mavlink_msg_raw_pressure_get_temperature(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:
	{
		digicamConfigure(
				mavlink_msg_digicam_configure_get_extra_value(msg),
				mavlink_msg_digicam_configure_get_shutter_speed(msg),
				mavlink_msg_digicam_configure_get_target_system(msg),
				mavlink_msg_digicam_configure_get_target_component(msg),
				mavlink_msg_digicam_configure_get_mode(msg),
				mavlink_msg_digicam_configure_get_aperture(msg),
				mavlink_msg_digicam_configure_get_iso(msg),
				mavlink_msg_digicam_configure_get_exposure_type(msg),
				mavlink_msg_digicam_configure_get_command_id(msg),
				mavlink_msg_digicam_configure_get_engine_cut_off(msg),
				mavlink_msg_digicam_configure_get_extra_param(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_DIGICAM_CONTROL:
	{
		digicamControl(
				mavlink_msg_digicam_control_get_extra_value(msg),
				mavlink_msg_digicam_control_get_target_system(msg),
				mavlink_msg_digicam_control_get_target_component(msg),
				mavlink_msg_digicam_control_get_session(msg),
				mavlink_msg_digicam_control_get_zoom_pos(msg),
				mavlink_msg_digicam_control_get_zoom_step(msg),
				mavlink_msg_digicam_control_get_focus_lock(msg),
				mavlink_msg_digicam_control_get_shot(msg),
				mavlink_msg_digicam_control_get_command_id(msg),
				mavlink_msg_digicam_control_get_extra_param(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
	{
		mountConfigure(
				mavlink_msg_mount_configure_get_target_system(msg),
				mavlink_msg_mount_configure_get_target_component(msg),
				mavlink_msg_mount_configure_get_mount_mode(msg),
				mavlink_msg_mount_configure_get_stab_roll(msg),
				mavlink_msg_mount_configure_get_stab_pitch(msg),
				mavlink_msg_mount_configure_get_stab_yaw(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_MOUNT_CONTROL:
	{
		mountControl(
				mavlink_msg_mount_control_get_input_a(msg),
				mavlink_msg_mount_control_get_input_b(msg),
				mavlink_msg_mount_control_get_input_c(msg),
				mavlink_msg_mount_control_get_target_system(msg),
				mavlink_msg_mount_control_get_target_component(msg),
				mavlink_msg_mount_control_get_save_position(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_MOUNT_STATUS:
	{
		mountStatus(
				mavlink_msg_mount_status_get_pointing_a(msg),
				mavlink_msg_mount_status_get_pointing_b(msg),
				mavlink_msg_mount_status_get_pointing_c(msg),
				mavlink_msg_mount_status_get_target_system(msg),
				mavlink_msg_mount_status_get_target_component(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_RADIO:
	{
		radio(
				mavlink_msg_radio_get_rxerrors(msg),
				mavlink_msg_radio_get_fixed(msg),
				mavlink_msg_radio_get_rssi(msg),
				mavlink_msg_radio_get_remrssi(msg),
				mavlink_msg_radio_get_txbuf(msg),
				mavlink_msg_radio_get_noise(msg),
				mavlink_msg_radio_get_remnoise(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_FENCE_POINT:
	{
		fencePoint(
				mavlink_msg_fence_point_get_lat(msg),
				mavlink_msg_fence_point_get_lng(msg),
				mavlink_msg_fence_point_get_target_system(msg),
				mavlink_msg_fence_point_get_target_component(msg),
				mavlink_msg_fence_point_get_idx(msg),
				mavlink_msg_fence_point_get_count(msg)
		);
		break;
	}
	case MAVLINK_MSG_ID_FENCE_FETCH_POINT:
	{
		fenceFetchPoint(
				mavlink_msg_fence_fetch_point_get_target_system(msg),
				mavlink_msg_fence_fetch_point_get_target_component(msg),
				mavlink_msg_fence_fetch_point_get_idx(msg)
		);
		break;
	}
	default:
	{
		// TODO reply unsupported
		break;
	}
	}
}


void MavServiceManager::requestDataStream(
		uint16_t req_message_rate,
		uint8_t target_system,
		uint8_t target_component,
		uint8_t req_stream_id,
		uint8_t start_stop
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
		_hkMgt.setDataStream(
				req_message_rate,
				req_stream_id,
				start_stop
		);
}

void MavServiceManager::commandLong(
		float param1,
		float param2,
		float param3,
		float param4,
		float param5,
		float param6,
		float param7,
		uint16_t command,
		uint8_t target_system,
		uint8_t target_component,
		uint8_t confirmation
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
		_cmdMgt.cmd(
				param1,
				param2,
				param3,
				param4,
				param5,
				param6,
				param7,
				command,
				confirmation
		);

}

void MavServiceManager::setMode(
		uint32_t custom_mode,
		uint8_t target_system,
		uint8_t base_mode
)
{
	if (checkSystem(target_system))
		_modeMgt.setMode(
				custom_mode,
				base_mode
				);
}


void MavServiceManager::missionRequestList(
		uint8_t target_system,
		uint8_t target_component
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
		_wpMgt.requestList();
}

void MavServiceManager::missionRequest(
		uint16_t seq,
		uint8_t target_system,
		uint8_t target_component
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
		_wpMgt.request(seq);
}

void MavServiceManager::missionAck(
		uint8_t target_system,
		uint8_t target_component,
		uint8_t type
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
		_wpMgt.ack(type);
}

void MavServiceManager::paramRequestList(
		uint8_t target_system,
		uint8_t target_component
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
	{
		/* Check if all conditions are met to switch to STATE_PARAM_REQUEST_LIST state */
		//TODO put here verifications if some ...

		/* switch to STATE_PARAM_REQUEST_LIST state */
		_state = STATE_PARAM_REQUEST_LIST;
		/* initialize with parameter index 0 */
		_paramRequestListIdx = 0;
	}
}

void MavServiceManager::paramRequestRead(
		int16_t param_index,
		uint8_t target_system,
		uint8_t target_component,
		const char *param_id
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
	{
		ParameterMgt::Value value;
		float fVal;
		enum MAV_PARAM_TYPE type;
		_paramMgt.read(param_index,value);
		_paramMgt.getInfo(param_index,&type,NULL);
		switch (type)
		{
		case MAV_PARAM_TYPE_UINT8:
			type = MAV_PARAM_TYPE_UINT8;
			fVal = (float) value.UINT8;
			break;
		case MAV_PARAM_TYPE_UINT16:
			type = MAV_PARAM_TYPE_UINT16;
			fVal = (float) value.UINT16;
			break;
		case MAV_PARAM_TYPE_UINT32:
			type = MAV_PARAM_TYPE_UINT32;
			fVal = (float) value.UINT32;
			break;
		case MAV_PARAM_TYPE_INT8:
			type = MAV_PARAM_TYPE_INT8;
			fVal = (float) value.INT8;
			break;
		case MAV_PARAM_TYPE_INT16:
			type = MAV_PARAM_TYPE_INT16;
			fVal = (float) value.INT16;
			break;
		case MAV_PARAM_TYPE_INT32:
			type = MAV_PARAM_TYPE_INT32;
			fVal = (float) value.INT32;
			break;
		case MAV_PARAM_TYPE_REAL32:
			fVal = (float) value.REAL32;
			break;
		case MAV_PARAM_TYPE_REAL64:
		case MAV_PARAM_TYPE_INT64:
		case MAV_PARAM_TYPE_UINT64:
		default:
			/* Not supported */
			/* Ended */
			return;
			break;
		}
		mavlink_msg_param_value_send(
				_port,
				param_id,
				fVal,
				(uint8_t) type,
				_paramMgt.getParamCount(),
				_paramRequestListIdx);
	}
}

void MavServiceManager::missionClearAll(
		uint8_t target_system,
		uint8_t target_component
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
		_wpMgt.clearAll();
}



void MavServiceManager::missionSetCurrent(
		uint16_t seq,
		uint8_t target_system,
		uint8_t target_component
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
		_wpMgt.setCurrent(seq);
}



void MavServiceManager::missionCount(
		uint16_t count,
		uint8_t target_system,
		uint8_t target_component
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
		_wpMgt.setCount(count);
}


void MavServiceManager::missionItem(
		float param1,
		float param2,
		float param3,
		float param4,
		float x,
		float y,
		float z,
		uint16_t seq,
		uint16_t command,
		uint8_t target_system,
		uint8_t target_component,
		uint8_t frame,
		uint8_t current,
		uint8_t autocontinue
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
		_wpMgt.setItem(
				param1,
				param2,
				param3,
				param4,
				x,
				y,
				z,
				seq,
				command,
				frame,
				current,
				autocontinue);
}


void MavServiceManager::setMagOffsets(
		int16_t mag_ofs_x,
		int16_t mag_ofs_y,
		int16_t mag_ofs_z,
		uint8_t target_system,
		uint8_t target_component
)
{
	// TODO Pourquoi n'est ce pas un paramètre à part entière?
}


void MavServiceManager::paramSet(
		float param_value,
		uint8_t target_system,
		uint8_t target_component,
		const char *param_id,
		uint8_t param_type
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
	{
		uint8_t idx;
		enum MAV_PARAM_TYPE type;
		ParameterMgt::Value value;
		_paramMgt.find(param_id, idx);
		_paramMgt.getInfo(idx, &type, NULL);
		switch (type)
		{
		case MAV_PARAM_TYPE_UINT8:
			value.UINT8 = (uint8_t) param_value;
			break;
		case MAV_PARAM_TYPE_UINT16:
			value.UINT16 = (uint16_t) param_value;
			break;
		case MAV_PARAM_TYPE_UINT32:
			value.UINT32 = (uint32_t) param_value;
			break;
		case MAV_PARAM_TYPE_INT8:
			value.INT8 = (int8_t) param_value;
			break;
		case MAV_PARAM_TYPE_INT16:
			value.INT16 = (int16_t) param_value;
			break;
		case MAV_PARAM_TYPE_INT32:
			value.INT32 = (int32_t) param_value;
			break;
		case MAV_PARAM_TYPE_REAL32:
			value.REAL32 = param_value;
			break;
		case MAV_PARAM_TYPE_REAL64:
		case MAV_PARAM_TYPE_INT64:
		case MAV_PARAM_TYPE_UINT64:
		default:
			/* Not supported */
			return;
			/* Ended */
			break;
		}
		/* Update the value */
		_paramMgt.write(idx,value);

		/* Send the value */
		paramRequestRead(idx,target_system,target_component,param_id);
	}
}


void MavServiceManager::rcChannelsOverride(
		uint16_t chan1_raw,
		uint16_t chan2_raw,
		uint16_t chan3_raw,
		uint16_t chan4_raw,
		uint16_t chan5_raw,
		uint16_t chan6_raw,
		uint16_t chan7_raw,
		uint16_t chan8_raw,
		uint8_t target_system,
		uint8_t target_component
)
{
	// TODO Part of Hil implementation
}


void MavServiceManager::hilState(
		uint64_t time_usec,
		float roll,
		float pitch,
		float yaw,
		float rollspeed,
		float pitchspeed,
		float yawspeed,
		int32_t lat,
		int32_t lon,
		int32_t alt,
		int16_t vx,
		int16_t vy,
		int16_t vz,
		int16_t xacc,
		int16_t yacc,
		int16_t zacc
)
{
	// TODO Part of Hil implementation
}



void MavServiceManager::heartbeat(
		uint32_t custom_mode,
		uint8_t type,
		uint8_t autopilot,
		uint8_t base_mode,
		uint8_t system_status,
		uint8_t mavlink_version
)
{
	// TODO log ground control system heartbeat information
}

void MavServiceManager::rawImu(
		uint64_t time_usec,
		int16_t xacc,
		int16_t yacc,
		int16_t zacc,
		int16_t xgyro,
		int16_t ygyro,
		int16_t zgyro,
		int16_t xmag,
		int16_t ymag,
		int16_t zmag
)
{
	// TODO Part of Hil implementation
}



void MavServiceManager::rawPressure(
		uint64_t time_usec,
		int16_t press_abs,
		int16_t press_diff1,
		int16_t press_diff2,
		int16_t temperature
)
{
	// TODO Part of Hil implementation
}


void MavServiceManager::digicamConfigure(
		float extra_value,
		uint16_t shutter_speed,
		uint8_t target_system,
		uint8_t target_component,
		uint8_t mode,
		uint8_t aperture,
		uint8_t iso,
		uint8_t exposure_type,
		uint8_t command_id,
		uint8_t engine_cut_off,
		uint8_t extra_param
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
		_digivcamMgt.configure(
				extra_value,
				shutter_speed,
				mode,
				aperture,
				iso,
				exposure_type,
				command_id,
				engine_cut_off,
				extra_param);
}


void MavServiceManager::digicamControl(
		float extra_value,
		uint8_t target_system,
		uint8_t target_component,
		uint8_t session,
		uint8_t zoom_pos,
		int8_t zoom_step,
		uint8_t focus_lock,
		uint8_t shot,
		uint8_t command_id,
		uint8_t extra_param
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
		_digivcamMgt.control(
				extra_value,
				session,
				zoom_pos,
				zoom_step,
				focus_lock,
				shot,
				command_id,
				extra_param);
}


void MavServiceManager::mountConfigure(
		uint8_t target_system,
		uint8_t target_component,
		uint8_t mount_mode,
		uint8_t stab_roll,
		uint8_t stab_pitch,
		uint8_t stab_yaw
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
		_mountMgt.configure(
				mount_mode,
				stab_roll,
				stab_pitch,
				stab_yaw);
}


void MavServiceManager::mountControl(
		int32_t input_a,
		int32_t input_b,
		int32_t input_c,
		uint8_t target_system,
		uint8_t target_component,
		uint8_t save_position
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
		_mountMgt.control(
				input_a,
				input_b,
				input_c,
				save_position);
}


void MavServiceManager::mountStatus(
		int32_t pointing_a,
		int32_t pointing_b,
		int32_t pointing_c,
		uint8_t target_system,
		uint8_t target_component
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
		_mountMgt.status(
				pointing_a,
				pointing_b,
				pointing_c);
}


void MavServiceManager::radio(
		uint16_t rxerrors,
		uint16_t fixed,
		uint8_t rssi,
		uint8_t remrssi,
		uint8_t txbuf,
		uint8_t noise,
		uint8_t remnoise
)
{
// TODO radio ???
}


void MavServiceManager::fencePoint(
		float lat,
		float lng,
		uint8_t target_system,
		uint8_t target_component,
		uint8_t idx,
		uint8_t count
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
		_fenceMgt.setPoint(
				lat,
				lng,
				idx,
				count);
}


void MavServiceManager::fenceFetchPoint(
		uint8_t target_system,
		uint8_t target_component,
		uint8_t idx
)
{
	if (checkSystem(target_system) && checkComponent(target_component))
		_fenceMgt.fetchPoint(idx);
}



bool MavServiceManager::checkSystem(uint8_t target_system)
{
	return true;
}

bool MavServiceManager::checkComponent(uint8_t target_component)
{
	return true;
}



} /* namespace mavlink */
