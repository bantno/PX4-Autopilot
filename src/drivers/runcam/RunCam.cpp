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

#include "RunCam.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <string.h>

RunCam::RunCam(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	strncpy(_device, device, sizeof(_device) - 1);
	_device[sizeof(_device) - 1] = '\0';
}

RunCam::~RunCam()
{
	if (_uart) {
		_uart->close();
		delete _uart;
	}

	perf_free(_cycle_perf);
	perf_free(_comms_error_perf);
}

int RunCam::task_spawn(int argc, char *argv[])
{
	const char *device_name = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;

		default:
			print_usage("unrecognized flag");
			return PX4_ERROR;
		}
	}

	if (device_name == nullptr || device_name[0] == '\0') {
		print_usage("no device specified");
		return PX4_ERROR;
	}

	if (!Serial::validatePort(device_name)) {
		PX4_ERR("invalid device %s", device_name);
		return PX4_ERROR;
	}

	RunCam *instance = new RunCam(device_name);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->ScheduleNow();

	return PX4_OK;
}

void RunCam::request(Request r)
{
	_pending_request.store(static_cast<int>(r));
	ScheduleNow();
}

bool RunCam::openPort()
{
	if (_uart == nullptr) {
		_uart = new Serial(_device, BAUDRATE);

		if (_uart == nullptr) {
			PX4_ERR("alloc failed for %s", _device);
			return false;
		}
	}

	if (_uart->isOpen()) {
		return true;
	}

	if (!_uart->setBaudrate(BAUDRATE)) {
		PX4_ERR("failed to set %lu baud on %s", (unsigned long)BAUDRATE, _device);
		return false;
	}

	if (!_uart->open()) {
		PX4_ERR("failed to open %s", _device);
		return false;
	}

	_uart->flush();
	PX4_INFO("opened %s at %lu baud", _device, (unsigned long)BAUDRATE);
	return true;
}

uint8_t RunCam::crc8DvbS2(const uint8_t *data, size_t len)
{
	uint8_t crc = 0;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];

		for (int b = 0; b < 8; b++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0xD5;

			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

bool RunCam::sendFrame(uint8_t command, const uint8_t *payload, size_t payload_len)
{
	if (_uart == nullptr || !_uart->isOpen()) {
		return false;
	}

	uint8_t buf[16];

	if (payload_len + 3 > sizeof(buf)) {
		return false;
	}

	size_t len = 0;
	buf[len++] = PROTOCOL_HEADER;
	buf[len++] = command;

	for (size_t i = 0; i < payload_len; i++) {
		buf[len++] = payload[i];
	}

	buf[len] = crc8DvbS2(buf, len);
	len++;

	const ssize_t written = _uart->write(buf, len);

	if (written != (ssize_t)len) {
		perf_count(_comms_error_perf);
		return false;
	}

	_bytes_tx += len;
	return true;
}

bool RunCam::sendCameraControl(uint8_t action)
{
	const hrt_abstime now = hrt_absolute_time();

	if (now - _last_command_time < MIN_COMMAND_GAP_US) {
		PX4_WARN("command too soon after previous one, ignored");
		return false;
	}

	const bool ok = sendFrame(CMD_CAMERA_CONTROL, &action, 1);

	if (ok) {
		_last_command_time = now;
	}

	return ok;
}

void RunCam::sendDeviceInfoRequest()
{
	_rx_len = 0;
	_uart->flush();

	if (sendFrame(CMD_GET_DEVICE_INFO, nullptr, 0)) {
		_info_pending = true;
		_info_sent_time = hrt_absolute_time();
	}
}

void RunCam::pollDeviceInfoResponse()
{
	if (!_info_pending) {
		return;
	}

	uint8_t byte;

	while (_uart->read(&byte, 1) == 1) {
		_bytes_rx++;

		if (_rx_len == 0 && byte != PROTOCOL_HEADER) {
			continue; // hunt for the header
		}

		_rx_buf[_rx_len++] = byte;

		if (_rx_len == INFO_RESPONSE_LEN) {
			if (crc8DvbS2(_rx_buf, INFO_RESPONSE_LEN - 1) == _rx_buf[INFO_RESPONSE_LEN - 1]) {
				_protocol_version = _rx_buf[1];
				_features = (uint16_t)_rx_buf[2] | ((uint16_t)_rx_buf[3] << 8);
				_device_info_valid = true;
				_info_pending = false;
				PX4_INFO("device found: protocol v%u, features 0x%04x", _protocol_version, _features);

			} else {
				_crc_errors++;
				perf_count(_comms_error_perf);
				// drop the first byte and keep scanning for the header
				memmove(_rx_buf, _rx_buf + 1, INFO_RESPONSE_LEN - 1);
				_rx_len = INFO_RESPONSE_LEN - 1;
			}

			if (!_info_pending) {
				_rx_len = 0;
				return;
			}
		}
	}

	if (_info_pending && hrt_elapsed_time(&_info_sent_time) > INFO_TIMEOUT_US) {
		_info_pending = false;
		_rx_len = 0;
	}
}

bool RunCam::startRecording()
{
	bool ok;

	if (_device_info_valid && (_features & FEATURE_START_RECORDING)) {
		ok = sendCameraControl(CAM_CTRL_START_RECORDING);

	} else {
		// older firmware: the power button toggles recording
		if (_recording_known && _recording) {
			return true; // already recording (as far as we know)
		}

		ok = sendCameraControl(CAM_CTRL_SIMULATE_POWER_BTN);
	}

	if (ok) {
		_recording = true;
		_recording_known = true;
	}

	return ok;
}

bool RunCam::stopRecording()
{
	bool ok;

	if (_device_info_valid && (_features & FEATURE_STOP_RECORDING)) {
		ok = sendCameraControl(CAM_CTRL_STOP_RECORDING);

	} else {
		if (_recording_known && !_recording) {
			return true;
		}

		ok = sendCameraControl(CAM_CTRL_SIMULATE_POWER_BTN);
	}

	if (ok) {
		_recording = false;
		_recording_known = true;
	}

	return ok;
}

bool RunCam::toggleRecording()
{
	if (_recording_known) {
		return _recording ? stopRecording() : startRecording();
	}

	// unknown state: the power button is a toggle on every RunCam Device Protocol camera
	const bool ok = sendCameraControl(CAM_CTRL_SIMULATE_POWER_BTN);

	if (ok) {
		_recording = !_recording;
		_recording_known = false; // still unknown, we only know it flipped
	}

	return ok;
}

void RunCam::handleRequest()
{
	const Request r = static_cast<Request>(_pending_request.load());
	_pending_request.store(static_cast<int>(Request::None));

	switch (r) {
	case Request::None:
		break;

	case Request::RecordStart:
		PX4_INFO("start recording: %s", startRecording() ? "sent" : "failed");
		break;

	case Request::RecordStop:
		PX4_INFO("stop recording: %s", stopRecording() ? "sent" : "failed");
		break;

	case Request::RecordToggle:
		PX4_INFO("toggle recording: %s", toggleRecording() ? "sent" : "failed");
		break;

	case Request::WifiButton:
		PX4_INFO("wifi button: %s", sendCameraControl(CAM_CTRL_SIMULATE_WIFI_BTN) ? "sent" : "failed");
		break;

	case Request::PowerButton:
		PX4_INFO("power button: %s", sendCameraControl(CAM_CTRL_SIMULATE_POWER_BTN) ? "sent" : "failed");
		_recording_known = false;
		break;

	case Request::ChangeMode:
		PX4_INFO("change mode: %s", sendCameraControl(CAM_CTRL_CHANGE_MODE) ? "sent" : "failed");
		_recording_known = false;
		break;

	case Request::QueryInfo:
		_device_info_valid = false;
		sendDeviceInfoRequest();
		break;
	}
}

void RunCam::handleRcSwitch()
{
	const int channel = _param_runcam_rc_ch.get();

	if (channel < 1 || channel > 6) {
		_rc_switch_valid = false;
		return;
	}

	manual_control_setpoint_s manual;

	if (!_manual_control_setpoint_sub.update(&manual)) {
		return;
	}

	if (!manual.valid) {
		_rc_switch_valid = false;
		return;
	}

	float value = 0.f;

	switch (channel) {
	case 1: value = manual.aux1; break;

	case 2: value = manual.aux2; break;

	case 3: value = manual.aux3; break;

	case 4: value = manual.aux4; break;

	case 5: value = manual.aux5; break;

	case 6: value = manual.aux6; break;
	}

	if (!PX4_ISFINITE(value)) {
		return;
	}

	// hysteresis around the centre so a 3-position switch middle does nothing
	bool high = _rc_switch_high;

	if (value > 0.5f) {
		high = true;

	} else if (value < -0.5f) {
		high = false;
	}

	if (!_rc_switch_valid) {
		// first valid sample: only act if the switch is already in the "record" position
		_rc_switch_valid = true;
		_rc_switch_high = high;

		if (high) {
			PX4_INFO("RC switch high at startup, start recording: %s", startRecording() ? "sent" : "failed");
		}

		return;
	}

	if (high != _rc_switch_high) {
		_rc_switch_high = high;

		if (high) {
			PX4_INFO("RC switch: start recording: %s", startRecording() ? "sent" : "failed");

		} else {
			PX4_INFO("RC switch: stop recording: %s", stopRecording() ? "sent" : "failed");
		}
	}
}

void RunCam::handleArming()
{
	const int mode = _param_runcam_arm_rec.get();

	if (mode == 0) {
		_armed_valid = false;
		return;
	}

	actuator_armed_s armed;

	if (!_actuator_armed_sub.update(&armed)) {
		return;
	}

	if (!_armed_valid) {
		_armed_valid = true;
		_armed = armed.armed;
		return; // no edge yet
	}

	if (armed.armed != _armed) {
		_armed = armed.armed;

		if (_armed) {
			PX4_INFO("armed: start recording: %s", startRecording() ? "sent" : "failed");

		} else if (mode == 2) {
			PX4_INFO("disarmed: stop recording: %s", stopRecording() ? "sent" : "failed");
		}
	}
}

void RunCam::Run()
{
	if (should_exit()) {
		ScheduleClear();

		if (_uart) {
			_uart->close();
			delete _uart;
			_uart = nullptr;
		}

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	if (!openPort()) {
		perf_end(_cycle_perf);
		ScheduleDelayed(1_s);
		return;
	}

	// discover the camera (retried until it answers, e.g. while it is still booting)
	if (!_device_info_valid && !_info_pending && hrt_elapsed_time(&_info_sent_time) > INFO_RETRY_US) {
		sendDeviceInfoRequest();
	}

	pollDeviceInfoResponse();

	handleRequest();
	handleRcSwitch();
	handleArming();

	perf_end(_cycle_perf);
	ScheduleDelayed(RUN_INTERVAL_US);
}

int RunCam::print_status()
{
	PX4_INFO("device: %s (%s)", _device, (_uart && _uart->isOpen()) ? "open" : "closed");

	if (_device_info_valid) {
		PX4_INFO("camera: protocol v%u, features 0x%04x", _protocol_version, _features);
		PX4_INFO("  power button: %s, wifi button: %s, change mode: %s",
			 (_features & FEATURE_SIMULATE_POWER_BUTTON) ? "yes" : "no",
			 (_features & FEATURE_SIMULATE_WIFI_BUTTON) ? "yes" : "no",
			 (_features & FEATURE_CHANGE_MODE) ? "yes" : "no");
		PX4_INFO("  start recording: %s, stop recording: %s, 5-key OSD: %s",
			 (_features & FEATURE_START_RECORDING) ? "yes" : "no",
			 (_features & FEATURE_STOP_RECORDING) ? "yes" : "no",
			 (_features & FEATURE_SIMULATE_5_KEY_OSD_CABLE) ? "yes" : "no");

	} else {
		PX4_INFO("camera: not detected (%s)", _info_pending ? "waiting for reply" : "will retry");
	}

	if (_recording_known) {
		PX4_INFO("recording: %s (as last commanded)", _recording ? "yes" : "no");

	} else {
		PX4_INFO("recording: unknown");
	}

	PX4_INFO("RC switch: %s, record on arm: %d", _param_runcam_rc_ch.get() > 0 ? "enabled" : "disabled",
		 (int)_param_runcam_arm_rec.get());
	PX4_INFO("bytes tx: %lu, rx: %lu, crc errors: %lu", (unsigned long)_bytes_tx, (unsigned long)_bytes_rx,
		 (unsigned long)_crc_errors);

	perf_print_counter(_cycle_perf);
	perf_print_counter(_comms_error_perf);

	return 0;
}

int RunCam::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return PX4_ERROR;
	}

	if (argc < 1) {
		return print_usage("missing command");
	}

	RunCam *instance = get_instance();
	const char *verb = argv[0];

	if (!strcmp(verb, "record")) {
		if (argc < 2) {
			return print_usage("record needs on|off|toggle");
		}

		if (!strcmp(argv[1], "on") || !strcmp(argv[1], "start")) {
			instance->request(Request::RecordStart);

		} else if (!strcmp(argv[1], "off") || !strcmp(argv[1], "stop")) {
			instance->request(Request::RecordStop);

		} else if (!strcmp(argv[1], "toggle")) {
			instance->request(Request::RecordToggle);

		} else {
			return print_usage("record needs on|off|toggle");
		}

		return PX4_OK;
	}

	if (!strcmp(verb, "wifi")) {
		instance->request(Request::WifiButton);
		return PX4_OK;
	}

	if (!strcmp(verb, "power")) {
		instance->request(Request::PowerButton);
		return PX4_OK;
	}

	if (!strcmp(verb, "mode")) {
		instance->request(Request::ChangeMode);
		return PX4_OK;
	}

	if (!strcmp(verb, "detect")) {
		instance->request(Request::QueryInfo);
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int RunCam::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for cameras that speak the RunCam Device Protocol over UART
(RunCam Split 4, Split 3, Hybrid, Thumb Pro, Racer, ...). The camera's
UART runs at 115200 baud, 8N1.

Recording can be started/stopped from the shell, from an RC AUX switch
(RUNCAM_RC_CH) or automatically on arming (RUNCAM_ARM_REC).

The camera does not report its recording state over protocol v1, so the
"recording" shown by `status` is the last state commanded by this driver.

### Examples
Start the driver on TELEM2 and start recording:
$ runcam start -d /dev/ttyS4
$ runcam record on
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("runcam", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("camera");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Serial device connected to the camera", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("record", "Start/stop video recording");
	PRINT_MODULE_USAGE_ARG("on|off|toggle", "Recording action", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("wifi", "Simulate the WiFi button");
	PRINT_MODULE_USAGE_COMMAND_DESCR("power", "Simulate the power button (toggles recording on Split)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("mode", "Cycle the camera mode (video/photo/OSD)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("detect", "Re-query the camera's protocol version and features");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int runcam_main(int argc, char *argv[])
{
	return RunCam::main(argc, argv);
}
