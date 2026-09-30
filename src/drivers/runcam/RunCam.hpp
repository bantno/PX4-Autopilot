/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file RunCam.hpp
 *
 * Driver for cameras speaking the RunCam Device Protocol over UART
 * (RunCam Split 4, Split 3, Hybrid, Thumb Pro, Racer, ...).
 *
 * Protocol reference: https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol
 *
 * Frame layout (both directions): 0xCC | command | payload... | CRC8 (DVB-S2 poly 0xD5 over all preceding bytes)
 */

#pragma once

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>

using device::Serial;
using namespace time_literals;

class RunCam : public ModuleBase<RunCam>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	RunCam(const char *device);
	~RunCam() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	/** Requests that the shell (or anything else) can hand to the work-queue thread. */
	enum class Request : int {
		None = 0,
		RecordStart,
		RecordStop,
		RecordToggle,
		WifiButton,
		PowerButton,
		ChangeMode,
		QueryInfo,
	};

	void request(Request r);

	/**
	 * Settings access, called from the shell. Blocks the caller (not the work queue)
	 * until the camera answers or times out, then prints the result.
	 * @param command CMD_GET_SETTINGS (list children of a folder), CMD_READ_SETTING_DETAIL or CMD_WRITE_SETTING
	 * @param value new setting value, only for CMD_WRITE_SETTING
	 */
	int querySettings(uint8_t command, uint8_t setting_id, const uint8_t *value = nullptr, size_t value_len = 0);

	// device setting access (protocol v1.0 "settings access" feature)
	static constexpr uint8_t CMD_GET_SETTINGS = 0x10;
	static constexpr uint8_t CMD_READ_SETTING_DETAIL = 0x11;
	static constexpr uint8_t CMD_WRITE_SETTING = 0x13;

private:
	static constexpr uint32_t BAUDRATE = 115200;

	static constexpr uint8_t PROTOCOL_HEADER = 0xCC;

	// command ids
	static constexpr uint8_t CMD_GET_DEVICE_INFO = 0x00;
	static constexpr uint8_t CMD_CAMERA_CONTROL = 0x01;

	// camera control actions
	static constexpr uint8_t CAM_CTRL_SIMULATE_WIFI_BTN = 0x00;
	static constexpr uint8_t CAM_CTRL_SIMULATE_POWER_BTN = 0x01;
	static constexpr uint8_t CAM_CTRL_CHANGE_MODE = 0x02;
	static constexpr uint8_t CAM_CTRL_START_RECORDING = 0x03;
	static constexpr uint8_t CAM_CTRL_STOP_RECORDING = 0x04;

	// feature bits reported by GET_DEVICE_INFO
	static constexpr uint16_t FEATURE_SIMULATE_POWER_BUTTON = (1 << 0);
	static constexpr uint16_t FEATURE_SIMULATE_WIFI_BUTTON = (1 << 1);
	static constexpr uint16_t FEATURE_CHANGE_MODE = (1 << 2);
	static constexpr uint16_t FEATURE_SIMULATE_5_KEY_OSD_CABLE = (1 << 3);
	static constexpr uint16_t FEATURE_DEVICE_SETTINGS_ACCESS = (1 << 4);
	static constexpr uint16_t FEATURE_DISPLAY_PORT = (1 << 5);
	static constexpr uint16_t FEATURE_START_RECORDING = (1 << 6);
	static constexpr uint16_t FEATURE_STOP_RECORDING = (1 << 7);
	static constexpr uint16_t FEATURE_CMS_MENU = (1 << 8);
	static constexpr uint16_t FEATURE_FC_ATTITUDE = (1 << 9);

	static constexpr unsigned INFO_RESPONSE_LEN = 5; // header, protocol version, features lo, features hi, crc
	static constexpr hrt_abstime INFO_TIMEOUT_US = 500_ms;
	static constexpr hrt_abstime INFO_RETRY_US = 2_s;
	static constexpr hrt_abstime RUN_INTERVAL_US = 50_ms;
	static constexpr hrt_abstime MIN_COMMAND_GAP_US = 300_ms; // camera button emulation needs settle time

	// setting responses: 0xCC | remaining chunks | data length | data... | CRC, up to 12 chunks of 62 bytes
	static constexpr unsigned QUERY_MAX_CHUNKS = 12;
	static constexpr unsigned QUERY_MAX_CHUNK_DATA = 62;
	static constexpr hrt_abstime QUERY_TIMEOUT_US = 1_s;
	static constexpr unsigned WRITE_RESPONSE_LEN = 4; // header, result code, menu changed flag, crc

	enum QueryState : int {
		QUERY_IDLE = 0,
		QUERY_CLAIMED,   // owner is filling in the request, the work queue must not start it yet
		QUERY_REQUESTED,
		QUERY_WAITING,
		QUERY_DONE,
		QUERY_FAILED,
	};

	// Recording state detection. Protocol v1 cannot report whether the camera is recording, but the
	// SD card free space (reserved setting 3, "free/total" in MB) falls while it records.
	static constexpr uint8_t SETTING_SDCARD_CAPACITY = 3;
	static constexpr hrt_abstime SD_POLL_INTERVAL_US = 1_s;
	static constexpr hrt_abstime SD_STEADY_WINDOW_US = 5_s;  // free space unchanged this long = stopped
	static constexpr hrt_abstime SD_STALE_US = 3_s;          // no reading this long = detection unavailable
	static constexpr hrt_abstime REC_SETTLE_US = 7_s;        // wait after a button press before judging it
	static constexpr hrt_abstime REC_DECIDE_TIMEOUT_US = 10_s; // longest wait for the state before giving up
	static constexpr int REC_MAX_ATTEMPTS = 3; // explicit start while unknown + two power button presses

	enum class RecState : int {
		Unknown = 0,
		Recording,
		Stopped,
	};

	void Run() override;

	bool openPort();
	bool sendFrame(uint8_t command, const uint8_t *payload, size_t payload_len);
	bool sendCameraControl(uint8_t action);
	void sendDeviceInfoRequest();
	void pollDeviceInfoResponse();
	void handleSettingsQuery();
	void sendQueryChunk();
	void pollQueryResponse();
	void failQuery(const char *reason);
	void finishQuery();
	bool startInternalQuery(uint8_t command, uint8_t setting_id);
	void handleSdCapacity(bool ok);
	bool detectionAvailable() const;
	RecState detectedState() const;
	bool setRecordingTarget(RecState target);
	void updateRecording();
	void printSettingsList() const;
	void printSettingDetail() const;
	void printWriteResult() const;
	static uint8_t crc8DvbS2(const uint8_t *data, size_t len);

	bool startRecording();
	bool stopRecording();
	bool toggleRecording();

	// fallback when the recording state cannot be detected: act on the last commanded state
	bool legacyStartRecording();
	bool legacyStopRecording();
	bool legacyToggleRecording();

	void handleRequest();
	void handleRcSwitch();
	void handleArming();

	char _device[20] {};
	Serial *_uart{nullptr};

	px4::atomic<int> _pending_request{static_cast<int>(Request::None)};

	// device info state
	bool _device_info_valid{false};
	bool _info_pending{false};
	hrt_abstime _info_sent_time{0};
	uint8_t _protocol_version{0};
	uint16_t _features{0};
	uint8_t _rx_buf[16] {};
	unsigned _rx_len{0};

	// settings probe: the shell thread fills the request and waits, the work queue runs the exchange
	px4::atomic<int> _query_state{QUERY_IDLE};
	uint8_t _query_command{0};
	uint8_t _query_setting_id{0};
	uint8_t _query_value[QUERY_MAX_CHUNK_DATA - 1] {};
	uint8_t _query_value_len{0};
	uint8_t _query_chunk{0};
	uint8_t _query_chunks_left{0};
	hrt_abstime _query_sent_time{0};
	uint8_t _query_rx[QUERY_MAX_CHUNK_DATA + 4] {};
	unsigned _query_rx_len{0};
	uint8_t _query_data[QUERY_MAX_CHUNKS * QUERY_MAX_CHUNK_DATA] {};
	unsigned _query_data_len{0};
	const char *_query_error{nullptr};
	bool _query_internal{false};                  // issued by the work queue itself (SD polling), not the shell
	px4::atomic<bool> _query_shell_waiting{false}; // a shell is waiting for a DONE/FAILED result

	// SD card free space tracking
	bool _sd_valid{false};
	uint32_t _sd_free_mb{0};
	uint32_t _sd_total_mb{0};
	hrt_abstime _sd_last_poll{0};
	hrt_abstime _sd_last_reading{0};
	hrt_abstime _sd_steady_since{0};  // last free space drop, or when tracking (re)started
	bool _sd_drop_seen{false};

	// requested recording state, driven to by pressing the camera buttons and confirmed from the SD card
	RecState _rec_target{RecState::Unknown}; // Unknown = nothing pending
	int _rec_attempts{0};
	hrt_abstime _rec_target_time{0};
	hrt_abstime _rec_last_action{0};

	// tracked camera state (protocol v1 has no recording-state query; this is what we last commanded)
	bool _recording{false};
	bool _recording_known{false};
	hrt_abstime _last_command_time{0};

	// RC switch edge detection
	bool _rc_switch_valid{false};
	bool _rc_switch_high{false};

	// arming edge detection
	bool _armed_valid{false};
	bool _armed{false};

	uint32_t _bytes_tx{0};
	uint32_t _bytes_rx{0};
	uint32_t _crc_errors{0};

	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _comms_error_perf{perf_alloc(PC_COUNT, MODULE_NAME": comms errors")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RUNCAM_RC_CH>) _param_runcam_rc_ch,
		(ParamInt<px4::params::RUNCAM_ARM_REC>) _param_runcam_arm_rec
	)
};
