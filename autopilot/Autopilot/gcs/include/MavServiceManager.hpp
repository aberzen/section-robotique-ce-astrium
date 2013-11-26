/*
 * MavServiceManager.hpp
 *
 *  Created on: 2 juin 2013
 *      Author: Aberzen
 */

#ifndef MAVSERVICEMANAGER_HPP_
#define MAVSERVICEMANAGER_HPP_

#include <avr/pgmspace.h>

#include <arch/include/Process.hpp>
#include "Channel.hpp"

#include "ModeMgt.hpp"
#include "ParameterMgt.hpp"
#include "WaypointMgt.hpp"
#include "HouseKeepingMgt.hpp"
#include "CommandMgt.hpp"
#include "MountMgt.hpp"
#include "FenceMgt.hpp"
#include "DigivcamMgt.hpp"

namespace mavlink {

class MavServiceManager : arch::Process {

protected:
	typedef enum {
		STATE_IDLE = 0,
		STATE_PARAM_REQUEST_LIST
	} State;
public:
	MavServiceManager(
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
			DigivcamMgt& digivcamMgt
			);
	virtual ~MavServiceManager();

	/** @brief Execute the process */
	virtual status initialize();

	/** @brief Execute the process */
	virtual status execute();
protected:
	/* ---------------------------------------------------------------- *
	 * GLOBAL SECTION                                                   *
	 * ---------------------------------------------------------------- */
	/** @brief Mavlink port used for this service management */
	mavlink_channel_t _port;

	/** @brief internal state for transactions */
	State _state;

	uint8_t _target_system;
	enum MAV_COMPONENT _target_component;

	/* ---------------------------------------------------------------- *
	 * HEARTBEAT MANAGEMENT SECTION                                     *
	 * ---------------------------------------------------------------- */
	uint16_t _prevHeartbeatDate;

	/* ---------------------------------------------------------------- *
	 * MODE MANAGEMENT SECTION                                          *
	 * ---------------------------------------------------------------- */
	ModeMgt& _modeMgt;


	/* ---------------------------------------------------------------- *
	 * PARAM MANAGEMENT SECTION                                         *
	 * ---------------------------------------------------------------- */
	ParameterMgt& _paramMgt;
	uint8_t _paramRequestListIdx;


	/* ---------------------------------------------------------------- *
	 * WAYPOINT MANAGEMENT SECTION                                      *
	 * ---------------------------------------------------------------- */
	WaypointMgt& _wpMgt;
	HouseKeepingMgt& _hkMgt;
	CommandMgt& _cmdMgt;
	MountMgt& _mountMgt;
	FenceMgt& _fenceMgt;
	DigivcamMgt& _digivcamMgt;

protected:
	void handleMessage(mavlink_message_t* msg);

	void requestDataStream(
			uint16_t req_message_rate,
			uint8_t target_system,
			uint8_t target_component,
			uint8_t req_stream_id,
			uint8_t start_stop
	);

	void commandLong(
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
	);

	void setMode(
			uint32_t custom_mode,
			uint8_t target_system,
			uint8_t base_mode
	);

	void missionRequestList(
			uint8_t target_system,
			uint8_t target_component
	);

	void missionRequest(
			uint16_t seq,
			uint8_t target_system,
			uint8_t target_component
	);

	void missionAck(
			uint8_t target_system,
			uint8_t target_component,
			uint8_t type
	);

	void paramRequestList(
			uint8_t target_system,
			uint8_t target_component
	);

	void paramRequestRead(
			int16_t param_index,
			uint8_t target_system,
			uint8_t target_component,
			const char *param_id
	);

	void missionClearAll(
			uint8_t target_system,
			uint8_t target_component
	);


	void missionSetCurrent(
			uint16_t seq,
			uint8_t target_system,
			uint8_t target_component
	);


	void missionCount(
			uint16_t count,
			uint8_t target_system,
			uint8_t target_component
	);

	void missionItem(
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
	);

	void setMagOffsets(
			int16_t mag_ofs_x,
			int16_t mag_ofs_y,
			int16_t mag_ofs_z,
			uint8_t target_system,
			uint8_t target_component
	);

	void paramSet(
			float param_value,
			uint8_t target_system,
			uint8_t target_component,
			const char *param_id,
			uint8_t param_type
	);

	void rcChannelsOverride(
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
	);

	void hilState(
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
	);


	void heartbeat(
			uint32_t custom_mode,
			uint8_t type,
			uint8_t autopilot,
			uint8_t base_mode,
			uint8_t system_status,
			uint8_t mavlink_version
	);

	void rawImu(
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
	);


	void rawPressure(
			uint64_t time_usec,
			int16_t press_abs,
			int16_t press_diff1,
			int16_t press_diff2,
			int16_t temperature
	);

	void digicamConfigure(
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
	);

	void digicamControl(
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
	);

	void mountConfigure(
			uint8_t target_system,
			uint8_t target_component,
			uint8_t mount_mode,
			uint8_t stab_roll,
			uint8_t stab_pitch,
			uint8_t stab_yaw
	);

	void mountControl(
			int32_t input_a,
			int32_t input_b,
			int32_t input_c,
			uint8_t target_system,
			uint8_t target_component,
			uint8_t save_position
	);

	void mountStatus(
			int32_t pointing_a,
			int32_t pointing_b,
			int32_t pointing_c,
			uint8_t target_system,
			uint8_t target_component
	);

	void radio(
			uint16_t rxerrors,
			uint16_t fixed,
			uint8_t rssi,
			uint8_t remrssi,
			uint8_t txbuf,
			uint8_t noise,
			uint8_t remnoise
	);


	void fencePoint(
			float lat,
			float lng,
			uint8_t target_system,
			uint8_t target_component,
			uint8_t idx,
			uint8_t count
	);

	void fenceFetchPoint(
			uint8_t target_system,
			uint8_t target_component,
			uint8_t idx
	);

	bool checkSystem(uint8_t target_system);
	bool checkComponent(uint8_t target_component);

};

} /* namespace mavlink */
#endif /* MAVSERVICEMANAGER_HPP_ */
