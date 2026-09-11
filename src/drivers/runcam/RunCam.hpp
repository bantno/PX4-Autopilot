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

	void Run() override;

	bool openPort();
	bool sendFrame(uint8_t command, const uint8_t *payload, size_t payload_len);
	bool sendCameraControl(uint8_t action);
	void sendDeviceInfoRequest();
	void pollDeviceInfoResponse();
	static uint8_t crc8DvbS2(const uint8_t *data, size_t len);

	bool startRecording();
	bool stopRecording();
	bool toggleRecording();

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
