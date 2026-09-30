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
#include <px4_platform_common/time.h>

#include <stdlib.h>
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

void RunCam::failQuery(const char *reason)
{
	_query_error = reason;

	if (_query_internal) {
		_query_internal = false;
		handleSdCapacity(false);
		_query_state.store(QUERY_IDLE);
		return;
	}

	_query_state.store(QUERY_FAILED);
}

void RunCam::finishQuery()
{
	if (_query_internal) {
		_query_internal = false;
		handleSdCapacity(true);
		_query_state.store(QUERY_IDLE);
		return;
	}

	_query_state.store(QUERY_DONE);
}

bool RunCam::startInternalQuery(uint8_t command, uint8_t setting_id)
{
	int expected = QUERY_IDLE;

	if (!_query_state.compare_exchange(&expected, QUERY_CLAIMED)) {
		return false; // a shell query owns the link
	}

	_query_internal = true;
	_query_command = command;
	_query_setting_id = setting_id;
	_query_value_len = 0;
	_query_state.store(QUERY_REQUESTED);
	handleSettingsQuery();
	return true;
}

void RunCam::sendQueryChunk()
{
	_query_rx_len = 0;
	_uart->flush();

	// reads: setting id, chunk index. writes: setting id, new value
	uint8_t payload[1 + sizeof(_query_value)] {_query_setting_id, _query_chunk};
	size_t payload_len = 2;

	if (_query_command == CMD_WRITE_SETTING) {
		memcpy(&payload[1], _query_value, _query_value_len);
		payload_len = 1 + _query_value_len;
	}

	if (!sendFrame(_query_command, payload, payload_len)) {
		failQuery("write failed");
		return;
	}

	_query_sent_time = hrt_absolute_time();
	_query_state.store(QUERY_WAITING);
}

void RunCam::pollQueryResponse()
{
	uint8_t byte;

	while (_uart->read(&byte, 1) == 1) {
		_bytes_rx++;

		if (_query_rx_len == 0 && byte != PROTOCOL_HEADER) {
			continue; // hunt for the header
		}

		_query_rx[_query_rx_len++] = byte;

		if (_query_command == CMD_WRITE_SETTING) {
			if (_query_rx_len < WRITE_RESPONSE_LEN) {
				continue;
			}

			if (crc8DvbS2(_query_rx, WRITE_RESPONSE_LEN - 1) != _query_rx[WRITE_RESPONSE_LEN - 1]) {
				_crc_errors++;
				perf_count(_comms_error_perf);
				failQuery("CRC error in reply");
				return;
			}

			_query_data[0] = _query_rx[1]; // result code
			_query_data[1] = _query_rx[2]; // menu items changed
			_query_data_len = 2;
			finishQuery();
			return;
		}

		if (_query_rx_len < 3) {
			continue;
		}

		const unsigned data_len = _query_rx[2];

		if (data_len > QUERY_MAX_CHUNK_DATA) {
			failQuery("bad length in reply");
			return;
		}

		if (_query_rx_len < data_len + 4) {
			continue;
		}

		if (crc8DvbS2(_query_rx, _query_rx_len - 1) != _query_rx[_query_rx_len - 1]) {
			_crc_errors++;
			perf_count(_comms_error_perf);
			failQuery("CRC error in reply");
			return;
		}

		if (_query_chunk == 0) {
			// the first reply says how many more chunks follow
			_query_chunks_left = _query_rx[1];

			if (_query_chunks_left >= QUERY_MAX_CHUNKS) {
				failQuery("too many chunks in reply");
				return;
			}
		}

		if (_query_data_len + data_len > sizeof(_query_data)) {
			failQuery("reply too large");
			return;
		}

		memcpy(&_query_data[_query_data_len], &_query_rx[3], data_len);
		_query_data_len += data_len;

		if (_query_chunks_left == 0) {
			finishQuery();
			return;
		}

		_query_chunks_left--;
		_query_chunk++;
		sendQueryChunk();
		return;
	}

	if (hrt_elapsed_time(&_query_sent_time) > QUERY_TIMEOUT_US) {
		failQuery(_query_rx_len > 0 ? "incomplete reply" : "no reply");
	}
}

void RunCam::handleSettingsQuery()
{
	switch (_query_state.load()) {
	case QUERY_REQUESTED:
		_query_chunk = 0;
		_query_chunks_left = 0;
		_query_data_len = 0;
		_query_error = nullptr;
		sendQueryChunk();
		break;

	case QUERY_WAITING:
		pollQueryResponse();
		break;

	case QUERY_DONE:
	case QUERY_FAILED:

		// result of a shell query whose shell stopped waiting: release the link
		if (!_query_shell_waiting.load()) {
			_query_state.store(QUERY_IDLE);
		}

		break;

	default:
		break;
	}
}

int RunCam::querySettings(uint8_t command, uint8_t setting_id, const uint8_t *value, size_t value_len)
{
	if (value_len > sizeof(_query_value)) {
		PX4_ERR("value too long");
		return PX4_ERROR;
	}

	if (!_device_info_valid) {
		PX4_ERR("camera not detected");
		return PX4_ERROR;
	}

	if (!(_features & FEATURE_DEVICE_SETTINGS_ACCESS)) {
		PX4_WARN("camera does not advertise settings access, trying anyway");
	}

	_query_shell_waiting.store(true);

	// the SD card poll holds the link for at most one short exchange, wait for it
	const hrt_abstime claim_deadline = hrt_absolute_time() + 2 * QUERY_TIMEOUT_US;
	bool claimed = false;

	while (!claimed && hrt_absolute_time() < claim_deadline) {
		int expected = QUERY_IDLE;
		claimed = _query_state.compare_exchange(&expected, QUERY_CLAIMED);

		if (!claimed) {
			px4_usleep(20_ms);
		}
	}

	if (!claimed) {
		_query_shell_waiting.store(false);
		PX4_ERR("another settings query is in progress");
		return PX4_ERROR;
	}

	_query_internal = false;
	_query_command = command;
	_query_setting_id = setting_id;

	if (value_len > 0) {
		memcpy(_query_value, value, value_len);
	}

	_query_value_len = value_len;
	_query_state.store(QUERY_REQUESTED);
	ScheduleNow();

	// worst case: every chunk takes the full timeout
	const hrt_abstime deadline = hrt_absolute_time() + QUERY_MAX_CHUNKS * QUERY_TIMEOUT_US + 1_s;
	int state = _query_state.load();

	while ((state == QUERY_CLAIMED || state == QUERY_REQUESTED || state == QUERY_WAITING)
	       && hrt_absolute_time() < deadline) {
		px4_usleep(20_ms);
		state = _query_state.load();
	}

	int ret = PX4_OK;

	if (state == QUERY_DONE) {
		if (command == CMD_WRITE_SETTING) {
			printWriteResult();

			if (_query_data[0] != 0) {
				ret = PX4_ERROR;
			}

		} else {
			PX4_INFO("setting %u: %u bytes in %u chunk(s)", setting_id, _query_data_len, _query_chunk + 1);

			if (command == CMD_GET_SETTINGS) {
				printSettingsList();

			} else {
				printSettingDetail();
			}
		}

	} else if (state == QUERY_FAILED) {
		PX4_ERR("query failed: %s (chunk %u)", _query_error ? _query_error : "unknown", _query_chunk);

		if (_query_rx_len > 0) {
			PX4_INFO_RAW("raw reply:");

			for (unsigned i = 0; i < _query_rx_len; i++) {
				PX4_INFO_RAW(" %02x", _query_rx[i]);
			}

			PX4_INFO_RAW("\n");
		}

		ret = PX4_ERROR;

	} else {
		// the work queue is still running the exchange, it releases the link once it finishes
		_query_shell_waiting.store(false);
		PX4_ERR("query timed out");
		return PX4_ERROR;
	}

	_query_state.store(QUERY_IDLE);
	_query_shell_waiting.store(false);
	return ret;
}

void RunCam::printWriteResult() const
{
	static constexpr const char *results[] {"success", "invalid", "no device", "device busy"};
	const uint8_t code = _query_data[0];

	if (code == 0) {
		PX4_INFO("write setting %u: success%s", _query_setting_id,
			 _query_data[1] ? " (camera menu changed, re-list settings)" : "");

	} else {
		PX4_ERR("write setting %u: %s (code %u)", _query_setting_id,
			code < (sizeof(results) / sizeof(results[0])) ? results[code] : "unknown error", code);
	}
}

void RunCam::printSettingsList() const
{
	// entries: id (u8) | name (NUL terminated) | value (NUL terminated)
	unsigned pos = 0;
	unsigned count = 0;

	while (pos < _query_data_len) {
		const uint8_t id = _query_data[pos++];
		const char *name = (const char *)&_query_data[pos];
		const size_t name_len = strnlen(name, _query_data_len - pos);
		pos += name_len + 1;

		if (pos > _query_data_len) {
			PX4_WARN("truncated entry for id %u", id);
			break;
		}

		const char *value = (const char *)&_query_data[pos];
		const size_t value_len = strnlen(value, _query_data_len - pos);
		pos += value_len + 1;

		if (pos > _query_data_len) {
			PX4_WARN("truncated value for id %u", id);
			break;
		}

		PX4_INFO_RAW("  %3u  %-20.*s = %.*s\n", id, (int)name_len, name, (int)value_len, value);
		count++;
	}

	PX4_INFO("%u setting(s). Use 'runcam setting <id>' for details, 'runcam settings <id>' to open a folder",
		 count);
}

void RunCam::printSettingDetail() const
{
	if (_query_data_len == 0) {
		PX4_WARN("empty reply");
		return;
	}

	// multi-byte fields are little endian
	const uint8_t *d = _query_data;
	const unsigned len = _query_data_len;
	auto u16 = [d](unsigned i) { return (uint32_t)d[i] | ((uint32_t)d[i + 1] << 8); };
	auto u32 = [d](unsigned i) {
		return (uint32_t)d[i] | ((uint32_t)d[i + 1] << 8) | ((uint32_t)d[i + 2] << 16) | ((uint32_t)d[i + 3] << 24);
	};

	switch (d[0]) {
	case 0: // uint8
	case 1: // int8
		if (len >= 5) {
			PX4_INFO_RAW("  type: %s, value: %u, min: %u, max: %u, step: %u\n", d[0] == 0 ? "uint8" : "int8",
				     d[1], d[2], d[3], d[4]);
		}

		break;

	case 2: // uint16
	case 3: // int16
		if (len >= 8) {
			PX4_INFO_RAW("  type: %s, value: %lu, min: %lu, max: %lu, step: %u\n", d[0] == 2 ? "uint16" : "int16",
				     (unsigned long)u16(1), (unsigned long)u16(3), (unsigned long)u16(5), d[7]);
		}

		break;

	case 8: // float, sent as scaled integers
		if (len >= 18) {
			PX4_INFO_RAW("  type: float, raw value: %lu, min: %lu, max: %lu, decimal point: %u, step: %lu\n",
				     (unsigned long)u32(1), (unsigned long)u32(5), (unsigned long)u32(9), d[13], (unsigned long)u32(14));
		}

		break;

	case 9: { // text selection: current index, then "option0;option1;..."
			if (len < 2) {
				break;
			}

			const unsigned current = d[1];
			PX4_INFO_RAW("  type: text selection, current index: %u\n", current);

			unsigned index = 0;
			unsigned start = 2;

			for (unsigned i = 2; i <= len; i++) {
				if (i == len || d[i] == ';' || d[i] == '\0') {
					if (i > start) {
						PX4_INFO_RAW("  %s %2u: %.*s\n", index == current ? "->" : "  ", index, (int)(i - start),
							     (const char *)&d[start]);
						index++;
					}

					if (i == len || d[i] == '\0') {
						break;
					}

					start = i + 1;
				}
			}
		}
		break;

	case 10: // string: value, then max length
		PX4_INFO_RAW("  type: string, value: \"%.*s\"\n", (int)strnlen((const char *)&d[1], len - 1), (const char *)&d[1]);
		break;

	case 11:
		PX4_INFO_RAW("  type: folder, list its contents with 'runcam settings <id>'\n");
		break;

	case 12: // info
		PX4_INFO_RAW("  type: info, value: \"%.*s\"\n", (int)strnlen((const char *)&d[1], len - 1), (const char *)&d[1]);
		break;

	default:
		PX4_INFO_RAW("  type: unknown (%u)\n", d[0]);
		break;
	}

	PX4_INFO_RAW("  raw:");

	for (unsigned i = 0; i < len; i++) {
		PX4_INFO_RAW(" %02x", d[i]);
	}

	PX4_INFO_RAW("\n");
}

bool RunCam::legacyStartRecording()
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

bool RunCam::legacyStopRecording()
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

bool RunCam::legacyToggleRecording()
{
	if (_recording_known) {
		return _recording ? legacyStopRecording() : legacyStartRecording();
	}

	// unknown state: the power button is a toggle on every RunCam Device Protocol camera
	const bool ok = sendCameraControl(CAM_CTRL_SIMULATE_POWER_BTN);

	if (ok) {
		_recording = !_recording;
		_recording_known = false; // still unknown, we only know it flipped
	}

	return ok;
}

void RunCam::handleSdCapacity(bool ok)
{
	const hrt_abstime now = hrt_absolute_time();
	unsigned long free_mb = 0;
	unsigned long total_mb = 0;

	// reply: type (info = 12), then "free/total" in MB
	if (ok && _query_data_len >= 2 && _query_data[0] == 12) {
		char text[QUERY_MAX_CHUNK_DATA + 1] {};
		const unsigned text_len = _query_data_len - 1;
		memcpy(text, &_query_data[1], text_len < QUERY_MAX_CHUNK_DATA ? text_len : QUERY_MAX_CHUNK_DATA);
		char *end = nullptr;
		free_mb = strtoul(text, &end, 10);
		ok = (end != text) && (*end == '/');

		if (ok) {
			const char *total_str = end + 1;
			total_mb = strtoul(total_str, &end, 10);
			ok = (end != total_str) && (total_mb > 0);
		}

	} else {
		ok = false;
	}

	if (!ok) {
		return; // detection goes stale on its own after SD_STALE_US
	}

	const bool restart = !_sd_valid || (now - _sd_last_reading > SD_STALE_US)
			     || (free_mb > _sd_free_mb) || (total_mb != _sd_total_mb);

	if (restart) {
		// first reading, gap in readings, or card changed / files deleted: start over
		_sd_steady_since = now;
		_sd_drop_seen = false;

	} else if (free_mb < _sd_free_mb) {
		_sd_steady_since = now;
		_sd_drop_seen = true;
	}

	_sd_valid = true;
	_sd_free_mb = free_mb;
	_sd_total_mb = total_mb;
	_sd_last_reading = now;
}

bool RunCam::detectionAvailable() const
{
	return _sd_valid && (hrt_elapsed_time(&_sd_last_reading) < SD_STALE_US);
}

RunCam::RecState RunCam::detectedState() const
{
	if (!detectionAvailable()) {
		return RecState::Unknown;
	}

	const hrt_abstime steady_for = hrt_elapsed_time(&_sd_steady_since);

	if (steady_for >= SD_STEADY_WINDOW_US) {
		return RecState::Stopped;
	}

	// falling recently: recording. Not yet steady long enough after (re)starting tracking: unknown
	return _sd_drop_seen ? RecState::Recording : RecState::Unknown;
}

bool RunCam::setRecordingTarget(RecState target)
{
	if (!detectionAvailable()) {
		// no SD card reading (no card, or no settings access): act blind on the last commanded state
		return (target == RecState::Recording) ? legacyStartRecording() : legacyStopRecording();
	}

	_rec_target = target;
	_rec_attempts = 0;
	_rec_target_time = hrt_absolute_time();
	_rec_last_action = 0;
	updateRecording();
	return true;
}

void RunCam::updateRecording()
{
	if (_rec_target == RecState::Unknown) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const RecState current = detectedState();
	const bool want_recording = (_rec_target == RecState::Recording);

	if (current == _rec_target) {
		PX4_INFO("recording %s (confirmed from SD card)", want_recording ? "started" : "stopped");
		_recording = want_recording;
		_recording_known = true;
		_rec_target = RecState::Unknown;
		return;
	}

	if (_rec_last_action != 0 && now - _rec_last_action < REC_SETTLE_US) {
		return; // give the last button press time to show up on the SD card
	}

	if (now - _last_command_time < MIN_COMMAND_GAP_US) {
		return; // another command was just sent, sendCameraControl() would reject this one
	}

	if (_rec_attempts >= REC_MAX_ATTEMPTS) {
		PX4_WARN("camera did not %s recording after %d attempts", want_recording ? "start" : "stop", _rec_attempts);
		_rec_target = RecState::Unknown;
		return;
	}

	// The power button toggles, so it is only pressed when the camera is known to be in the other state.
	// While the state is still unknown an explicit start is harmless to try, but the Split 4 advertises
	// it and ignores it (bench test 2026-09-30), so once the state is known the power button is used.
	const bool explicit_start = want_recording && (current == RecState::Unknown)
				    && _device_info_valid && (_features & FEATURE_START_RECORDING);

	if (current == RecState::Unknown && !explicit_start) {
		if (now - _rec_target_time > REC_DECIDE_TIMEOUT_US) {
			PX4_WARN("recording state unknown, not pressing the power button");
			_rec_target = RecState::Unknown;
		}

		return; // wait for the SD card reading to settle
	}

	const bool ok = explicit_start ? sendCameraControl(CAM_CTRL_START_RECORDING)
			: sendCameraControl(CAM_CTRL_SIMULATE_POWER_BTN);

	if (ok) {
		_rec_attempts++;
		_rec_last_action = now;
	}
}

bool RunCam::startRecording()
{
	return setRecordingTarget(RecState::Recording);
}

bool RunCam::stopRecording()
{
	return setRecordingTarget(RecState::Stopped);
}

bool RunCam::toggleRecording()
{
	const RecState current = detectedState();

	if (current == RecState::Unknown) {
		return legacyToggleRecording();
	}

	return setRecordingTarget(current == RecState::Recording ? RecState::Stopped : RecState::Recording);
}

void RunCam::handleRequest()
{
	const Request r = static_cast<Request>(_pending_request.load());
	_pending_request.store(static_cast<int>(Request::None));

	switch (r) {
	case Request::None:
		break;

	case Request::RecordStart:
		PX4_INFO("start recording: %s", startRecording() ? "ok" : "failed");
		break;

	case Request::RecordStop:
		PX4_INFO("stop recording: %s", stopRecording() ? "ok" : "failed");
		break;

	case Request::RecordToggle:
		PX4_INFO("toggle recording: %s", toggleRecording() ? "ok" : "failed");
		break;

	case Request::WifiButton:
		PX4_INFO("wifi button: %s", sendCameraControl(CAM_CTRL_SIMULATE_WIFI_BTN) ? "sent" : "failed");
		break;

	case Request::PowerButton:
		PX4_INFO("power button: %s", sendCameraControl(CAM_CTRL_SIMULATE_POWER_BTN) ? "sent" : "failed");
		_recording_known = false;
		_rec_target = RecState::Unknown;
		break;

	case Request::ChangeMode:
		PX4_INFO("change mode: %s", sendCameraControl(CAM_CTRL_CHANGE_MODE) ? "sent" : "failed");
		_recording_known = false;
		_rec_target = RecState::Unknown;
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
	handleSettingsQuery();

	// poll the SD card free space to tell whether the camera is recording
	if (_device_info_valid && (_features & FEATURE_DEVICE_SETTINGS_ACCESS)
	    && hrt_elapsed_time(&_sd_last_poll) >= SD_POLL_INTERVAL_US
	    && startInternalQuery(CMD_READ_SETTING_DETAIL, SETTING_SDCARD_CAPACITY)) {
		_sd_last_poll = hrt_absolute_time();
	}

	handleRequest();
	handleRcSwitch();
	handleArming();
	updateRecording();

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

	if (detectionAvailable()) {
		const RecState state = detectedState();
		const float steady_s = hrt_elapsed_time(&_sd_steady_since) * 1e-6f;

		PX4_INFO("recording: %s (SD card %lu/%lu MB free, %s)",
			 state == RecState::Recording ? "yes" : (state == RecState::Stopped ? "no" : "unknown"),
			 (unsigned long)_sd_free_mb, (unsigned long)_sd_total_mb,
			 state == RecState::Recording ? "falling" : (state == RecState::Stopped ? "steady" : "measuring"));

		if (state == RecState::Unknown) {
			PX4_INFO("  steady for %.1f s, decides after %.0f s", (double)steady_s, (double)(SD_STEADY_WINDOW_US * 1e-6f));
		}

	} else if (_recording_known) {
		PX4_INFO("recording: %s (as last commanded, no SD card reading)", _recording ? "yes" : "no");

	} else {
		PX4_INFO("recording: unknown (no SD card reading)");
	}

	if (_rec_target != RecState::Unknown) {
		PX4_INFO("  %s requested, %d button press(es) so far",
			 _rec_target == RecState::Recording ? "start" : "stop", _rec_attempts);
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

	if (!strcmp(verb, "settings")) {
		const int parent = (argc >= 2) ? atoi(argv[1]) : 0;

		if (parent < 0 || parent > 255) {
			return print_usage("setting id must be 0-255");
		}

		return instance->querySettings(CMD_GET_SETTINGS, (uint8_t)parent);
	}

	if (!strcmp(verb, "setting")) {
		if (argc < 2) {
			return print_usage("setting needs an id");
		}

		const int id = atoi(argv[1]);

		if (id < 0 || id > 255) {
			return print_usage("setting id must be 0-255");
		}

		return instance->querySettings(CMD_READ_SETTING_DETAIL, (uint8_t)id);
	}

	if (!strcmp(verb, "set")) {
		if (argc < 3) {
			return print_usage("set needs an id and a value");
		}

		const int id = atoi(argv[1]);
		const int value = atoi(argv[2]);

		if (id < 0 || id > 255 || value < 0 || value > 255) {
			return print_usage("id and value must be 0-255");
		}

		// single byte: option index for text selections, or a uint8 value
		const uint8_t byte = (uint8_t)value;
		return instance->querySettings(CMD_WRITE_SETTING, (uint8_t)id, &byte, 1);
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

The camera does not report its recording state over protocol v1. Instead the
driver polls the SD card free space (settings access) once a second: falling
means recording, unchanged for 5 s means stopped. Record commands set a target
state; the power button is only pressed when the camera is known to be in the
other state, and the result is confirmed from the SD card and retried. The
explicit start command is only tried while the state is still unknown (the
Split 4 advertises it but ignores it). Without
an SD card reading the driver falls back to acting on the last commanded state.
A full SD card reads as "stopped".

`settings` and `setting` read the protocol's device settings (list a
settings folder, read one setting), and `set` writes a single-byte setting
(the option index of a text selection, or a uint8). Which settings a camera
exposes, if any, is up to its firmware.

### Examples
Start the driver on TELEM2 and start recording:
$ runcam start -d /dev/ttyS4
$ runcam record on

List the camera's top-level settings, then read setting 2:
$ runcam settings
$ runcam setting 2

Select option 1 (PAL) of setting 2:
$ runcam set 2 1
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
	PRINT_MODULE_USAGE_COMMAND_DESCR("settings", "List camera settings (read only)");
	PRINT_MODULE_USAGE_ARG("<id>", "Parent folder id (default 0 = top level)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("setting", "Read one camera setting's type, value and options");
	PRINT_MODULE_USAGE_ARG("<id>", "Setting id", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "Write a single-byte camera setting");
	PRINT_MODULE_USAGE_ARG("<id> <value>", "Setting id and option index / value", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("detect", "Re-query the camera's protocol version and features");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int runcam_main(int argc, char *argv[])
{
	return RunCam::main(argc, argv);
}
