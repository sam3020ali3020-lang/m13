/**
 * @file XqpowerCan.cpp
 * @brief XQPOWER CAN Servo Driver for PX4
 *
 * NuttX:   Direct STM32H7 FDCAN register access (MicoAir H743)
 *          FDCAN clock: HSE = 8 MHz, Bitrate: 500 kbps, SP=87.5%
 *
 * Android: SLCAN via USB-Serial (Waveshare USB_CAN_A / CH340)
 *          USB FD passed from Java via JNI -> set_usb_fd()
 *          CH340 configured via direct Linux USB ioctl
 *          SLCAN protocol: tIIILDD..DD\r for standard CAN frames
 */

#include "XqpowerCan.hpp"

#include <string.h>
#include <math.h>
#include <errno.h>

#ifdef __PX4_POSIX
#include <android/log.h>
#define TAG_CAN "XqpowerSLCAN"
#define LOGI_CAN(...) __android_log_print(ANDROID_LOG_INFO,  TAG_CAN, __VA_ARGS__)
#define LOGE_CAN(...) __android_log_print(ANDROID_LOG_ERROR, TAG_CAN, __VA_ARGS__)

/* Static members */
std::atomic<int> XqpowerCan::s_usb_fd{-1};
std::atomic<int> XqpowerCan::s_consumed_fd{-1};

void XqpowerCan::set_usb_fd(int fd)
{
	LOGI_CAN("set_usb_fd(%d)", fd);
	s_usb_fd.store(fd);
}

/* JNI-callable wrapper (used from px4_jni.cpp) */
namespace xqpower_can_jni {
void set_usb_fd(int fd) { XqpowerCan::set_usb_fd(fd); }
}

/* ===== SLCAN hex helpers ===== */
static char hex_char(uint8_t nibble)
{
	return nibble < 10 ? '0' + nibble : 'A' + nibble - 10;
}

static uint8_t hex_val(char c)
{
	if (c >= '0' && c <= '9') { return c - '0'; }
	if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
	if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
	return 0;
}
#endif /* __PX4_POSIX */

XqpowerCan::XqpowerCan() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

XqpowerCan::~XqpowerCan()
{
	ScheduleClear();

	if (_report_started) {
		for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
			servo_stop_report(i);
		}
	}

	can_close();
}

bool XqpowerCan::init()
{
#ifdef __PX4_POSIX
	/* On Android, USB FD may not be available yet at startup.
	 * Try to open now; if it fails, Run() will keep retrying
	 * via the reconnect handler (lines ~963-997). */
	if (!can_open()) {
		PX4_WARN("CAN not available yet — will connect when USB_CAN_A is plugged in");
	}
#else
	if (!can_open()) {
		PX4_ERR("Failed to open CAN bus");
		return false;
	}
#endif

	/* Load parameters */
	updateParams();
	_node_ids[0] = (uint8_t)_param_node1.get();
	_node_ids[1] = (uint8_t)_param_node2.get();
	_node_ids[2] = (uint8_t)_param_node3.get();
	_node_ids[3] = (uint8_t)_param_node4.get();
	_angle_limit   = _param_angle_limit.get();
	_reverse_mask  = (uint8_t)(_param_reverse.get() & 0x0F);

	/* Detect simulation mode once at startup from SYS_HITL.
	 * In real flight (SYS_HITL=0) we MUST ignore actuator_outputs_sim
	 * because its values are in RADIANS — if any stale publisher feeds it,
	 * xqpower_can would scale by 57.3 (deg/rad) instead of XQCAN_LIMIT,
	 * sending ~2.87x the intended deflection to the servos. */
	int32_t hitl_val = 0;
	param_get(param_find("SYS_HITL"), &hitl_val);
	_sim_mode = (hitl_val == 1);

	/* Cross-check deflection limit with the acados solver.
	 * The solver has delta_max = 20° baked in (see m130_ocp_setup.py::delta_max).
	 * XQCAN_LIMIT must match, otherwise commands get clipped and wind-up occurs. */
	{
		const float solver_limit_deg = 20.0f;   // matches SOLVER_DELTA_MAX_RAD (0.3490658...)
		if (fabsf(solver_limit_deg - _angle_limit) > 0.5f) {
			PX4_WARN("Limit MISMATCH: solver delta_max=%.1f deg but XQCAN_LIMIT=%.2f deg — wind-up risk!",
				 (double)solver_limit_deg, (double)_angle_limit);
		}
	}

	PX4_INFO("XQPOWER CAN: Nodes=[0x%02X,0x%02X,0x%02X,0x%02X] Limit=%.1f deg Rev=0x%X Mode=%s",
		 _node_ids[0], _node_ids[1], _node_ids[2], _node_ids[3],
		 (double)_angle_limit, _reverse_mask,
		 _sim_mode ? "HITL/SITL (sim topic allowed)"
			   : "REAL FLIGHT (sim topic IGNORED)");

	/* Run servo init sequence only if CAN is already open */
	if (_can_initialized) {
		_servo_init_sequence();
	}

	/* Run at 100 Hz on all platforms */
	ScheduleOnInterval(10000);  /* 100 Hz */

	return true;
}

/* ========================== CAN Transport Layer ============================ */

#ifdef __PX4_POSIX
/* -------------------- USB-Serial via direct ioctl (Android) -------------------- */

/*
 * Direct Linux USB ioctl — bypasses libusb entirely.
 * This is exactly what Android's UsbDeviceConnection.bulkTransfer() and
 * controlTransfer() use internally, and matches the proven ServoController app.
 *
 * libusb_bulk_transfer was found to be unreliable on Android with CH340 chips,
 * while direct ioctl (used by Java's UsbDeviceConnection) works correctly.
 */

int XqpowerCan::usb_control_msg(uint8_t reqtype, uint8_t req, uint16_t val,
				uint16_t idx, void *data, uint16_t len, uint32_t timeout_ms)
{
	if (_usb_fd < 0) { return -1; }

	struct usbdevfs_ctrltransfer ct;
	ct.bRequestType = reqtype;
	ct.bRequest     = req;
	ct.wValue       = val;
	ct.wIndex       = idx;
	ct.wLength      = len;
	ct.timeout      = timeout_ms;
	ct.data         = data;

	return ioctl(_usb_fd, USBDEVFS_CONTROL, &ct);
}

bool XqpowerCan::ch340_set_baudrate(uint32_t baud)
{
	if (_usb_fd < 0) { return false; }

	/*
	 * CH340/CH341 initialization sequence
	 * (matching usb-serial-for-android Ch34xSerialDriver exactly)
	 */
	int r;
	uint8_t buf[8];

	/* Step 1: Init — get version + read status */
	r = usb_control_msg(0xC0, 0x5F, 0, 0, buf, 8, 1000);
	LOGI_CAN("CH340 get_version: r=%d", r);

	r = usb_control_msg(0xC0, 0xA1, 0, 0, buf, 8, 1000);
	LOGI_CAN("CH340 init status: r=%d", r);

	/* Step 2: Calculate and set baud rate */
	long factor;
	int divisor;

	if (baud == 921600) {
		divisor = 7;
		factor = 0xf300;

	} else {
		factor = (long)(1532620800L / baud);
		divisor = 3;

		while ((factor > 0xfff0) && (divisor > 0)) {
			factor >>= 3;
			divisor--;
		}

		factor = 0x10000 - factor;
	}

	divisor |= 0x0080;
	int val1 = (int)((factor & 0xff00) | divisor);
	int val2 = (int)(factor & 0xff);

	r = usb_control_msg(0x40, 0x9A, 0x1312, val1, nullptr, 0, 1000);
	LOGI_CAN("CH340 baud reg1: r=%d", r);
	r = usb_control_msg(0x40, 0x9A, 0x0f2c, val2, nullptr, 0, 1000);
	LOGI_CAN("CH340 baud reg2: r=%d", r);

	/* Step 3: Set LCR: 8N1 */
	r = usb_control_msg(0x40, 0x9A, 0x2518, 0x00C3, nullptr, 0, 1000);
	LOGI_CAN("CH340 LCR 8N1: r=%d", r);

	/* Step 4: Read LCR back */
	r = usb_control_msg(0xC0, 0x95, 0x2518, 0, buf, 8, 1000);
	LOGI_CAN("CH340 read LCR: r=%d", r);

	/* Step 5: Set handshake — DTR + RTS active */
	r = usb_control_msg(0x40, 0xA4, 0, 0xFF9F, nullptr, 0, 1000);
	LOGI_CAN("CH340 handshake DTR+RTS: r=%d", r);

	/* Step 6: Read modem status */
	r = usb_control_msg(0xC0, 0x95, 0x0706, 0, buf, 8, 1000);
	LOGI_CAN("CH340 modem status: r=%d", r);

	LOGI_CAN("CH340 baud=%u (val1=0x%04X val2=0x%04X) — init complete", baud, val1, val2);
	return true;
}

bool XqpowerCan::usb_serial_open(int fd)
{
	if (_usb_fd >= 0) {
		usb_serial_close();
	}

	_usb_fd = fd;

	/* Detach kernel driver and claim interfaces (needed for CDC-ACM devices like CAN_LIN_Tool) */
	for (unsigned int iface = 0; iface < 3; iface++) {
		struct usbdevfs_ioctl disc;
		disc.ifno = iface;
		disc.ioctl_code = USBDEVFS_DISCONNECT;
		disc.data = nullptr;
		ioctl(_usb_fd, USBDEVFS_IOCTL, &disc); // detach kernel driver (ignore error)
		ioctl(_usb_fd, USBDEVFS_CLAIMINTERFACE, &iface); // claim
	}

	/* Detect device type by trying CH340 init — if it fails, assume CDC (CAN_LIN_Tool) */
	uint8_t ver_buf[8];
	int r = usb_control_msg(0xC0, 0x5F, 0, 0, ver_buf, 8, 500);

	if (r >= 0) {
		/* CH340 detected — Waveshare USB_CAN_A */
		_is_canlin_tool = false;
		_bulk_ep_out = 0x02;
		_bulk_ep_in  = 0x82;

		if (!ch340_set_baudrate(2000000)) {
			LOGE_CAN("CH340 baud rate setup failed");
			_usb_fd = -1;
			return false;
		}

		LOGI_CAN("Waveshare CH340 opened: fd=%d ep_out=0x%02X ep_in=0x%02X", fd, _bulk_ep_out, _bulk_ep_in);

	} else {
		/* CDC device — CAN_LIN_Tool V6.0 (VID=0x2E3C) */
		_is_canlin_tool = true;

		/* Try common CDC bulk endpoints */
		const int try_ep_out[] = {0x03, 0x02, 0x01};
		const int try_ep_in[]  = {0x83, 0x82, 0x81};
		_bulk_ep_out = 0x03;
		_bulk_ep_in  = 0x83;

		/* Test which endpoint works by trying a small write */
		uint8_t test[] = {0x03, 0x02, 0x03}; // "receive all" command — harmless
		for (int i = 0; i < 3; i++) {
			_bulk_ep_out = try_ep_out[i];
			_bulk_ep_in  = try_ep_in[i];
			struct usbdevfs_bulktransfer bulk;
			bulk.ep = _bulk_ep_out;
			bulk.len = sizeof(test);
			bulk.timeout = 200;
			bulk.data = test;
			int wr = ioctl(_usb_fd, USBDEVFS_BULK, &bulk);
			if (wr >= 0) {
				LOGI_CAN("CAN_LIN_Tool CDC: ep_out=0x%02X works (wrote %d)", _bulk_ep_out, wr);
				break;
			}
			LOGI_CAN("CAN_LIN_Tool CDC: ep_out=0x%02X failed, trying next...", _bulk_ep_out);
		}

		LOGI_CAN("CAN_LIN_Tool CDC opened: fd=%d ep_out=0x%02X ep_in=0x%02X", fd, _bulk_ep_out, _bulk_ep_in);
	}

	return true;
}

void XqpowerCan::usb_serial_close()
{
	if (_usb_fd >= 0) {
		unsigned int iface_num = 0;
		ioctl(_usb_fd, USBDEVFS_RELEASEINTERFACE, &iface_num);
		/* Don't close(fd) — owned by Java UsbDeviceConnection */
		_usb_fd = -1;
	}
}

int XqpowerCan::slcan_write(const char *data, int len)
{
	if (_usb_fd < 0) { return -1; }

	struct usbdevfs_bulktransfer bulk;
	bulk.ep      = _bulk_ep_out;
	bulk.len     = len;
	bulk.timeout = 50;
	bulk.data    = (void *)data;

	int r = ioctl(_usb_fd, USBDEVFS_BULK, &bulk);

	if (r < 0) {
		int e = errno;
		_tx_fail_count++;

		static int tx_err_log = 0;

		if (tx_err_log < 10) {
			LOGE_CAN("slcan_write ioctl failed: errno=%d (%s) len=%d", e, strerror(e), len);
			tx_err_log++;
		}

		return -1;
	}

	return r;  /* returns number of bytes transferred */
}

int XqpowerCan::slcan_read(char *buf, int maxlen)
{
	if (_usb_fd < 0) { return -1; }

	struct usbdevfs_bulktransfer bulk;
	bulk.ep      = _bulk_ep_in;
	bulk.len     = maxlen;
	bulk.timeout = 50;   /* 50ms — matches ServoController's 100ms, allows data to arrive */
	bulk.data    = buf;

	int r = ioctl(_usb_fd, USBDEVFS_BULK, &bulk);

	if (r < 0) {
		int e = errno;

		if (e == ETIMEDOUT || e == EAGAIN || e == EWOULDBLOCK) {
			_read_zero_count++;
			return 0;  /* No data available */
		}

		_read_err_count++;

		/* Log unexpected errors (max 10 times) */
		static int rx_err_log = 0;

		if (rx_err_log < 10) {
			LOGE_CAN("slcan_read ioctl failed: errno=%d (%s)", e, strerror(e));
			rx_err_log++;
		}

		return -1;
	}

	_read_data_count++;
	return r;
}

bool XqpowerCan::can_open()
{
	int fd = s_usb_fd.load();

	if (fd < 0) {
		PX4_ERR("No USB FD available — connect Waveshare USB_CAN_A first");
		return false;
	}

	if (!usb_serial_open(fd)) {
		return false;
	}

	s_consumed_fd.store(fd, std::memory_order_release);

	/* Flush any stale data from the receive buffer */
	char flush[64];
	slcan_read(flush, sizeof(flush));

	_slcan_rxpos = 0;

	if (_is_canlin_tool) {
		/* CAN_LIN_Tool V6.0: configure CAN baud rate 500K */
		uint8_t cmd_500k[] = {0x03, 0x01, 0xF4, 0x01, 0x00, 0x00, 0x12, 0x00, 0x00, 0x05, 0x00};
		slcan_write((const char *)cmd_500k, sizeof(cmd_500k));
		px4_usleep(100000); // 100ms

		/* Save config */
		uint8_t cmd_save[] = {0x03, 0x05};
		slcan_write((const char *)cmd_save, sizeof(cmd_save));
		px4_usleep(100000);

		/* Enable 120Ω termination resistor */
		uint8_t cmd_120r[] = {0x06, 0x01};
		slcan_write((const char *)cmd_120r, sizeof(cmd_120r));
		px4_usleep(50000);

		/* Receive all (disable filters) */
		uint8_t cmd_rcv_all[] = {0x03, 0x02, 0x03};
		slcan_write((const char *)cmd_rcv_all, sizeof(cmd_rcv_all));
		px4_usleep(50000);

		/* Flush responses */
		slcan_read(flush, sizeof(flush));

		PX4_INFO("CAN_LIN_Tool V6.0 opened (CDC, 500 kbps CAN, 120Ω ON)");
	} else {
		PX4_INFO("Waveshare USB_CAN_A opened (direct ioctl, 500 kbps CAN)");
	}

	_can_initialized = true;
	return true;
}

void XqpowerCan::can_close()
{
	usb_serial_close();
	_can_initialized = false;
}

int XqpowerCan::can_send(uint16_t id, const uint8_t *data, uint8_t dlc)
{
	if (!_can_initialized || _usb_fd < 0 || dlc > 8) {
		return -1;
	}

	int pos = 0;
	int ret;

	if (_is_canlin_tool) {
		/*
		 * CAN_LIN_Tool V6.0 protocol (20 bytes fixed):
		 *   [0]     command = 0x01 (CAN frame)
		 *   [1-4]   standard_id (4 bytes, little-endian)
		 *   [5-8]   extended_id (4 bytes, little-endian) — 0 for standard
		 *   [9]     id_type: 0x00=standard
		 *   [10]    frame_type: 0x00=data
		 *   [11]    dlc
		 *   [12-19] data[8] — padded with 0x00
		 */
		uint8_t buf[20] = {};
		buf[0] = 0x01;                           /* command: CAN */
		buf[1] = (uint8_t)(id & 0xFF);           /* standard_id LE */
		buf[2] = (uint8_t)((id >> 8) & 0xFF);
		buf[3] = 0x00;
		buf[4] = 0x00;
		/* buf[5..8] = extended_id = 0 */
		buf[9]  = 0x00;                          /* id_type: standard */
		buf[10] = 0x00;                          /* frame_type: data */
		buf[11] = dlc;

		for (int i = 0; i < dlc; i++) {
			buf[12 + i] = data[i];
		}

		pos = 20;
		ret = slcan_write((const char *)buf, 20);

	} else {
		/*
		 * Waveshare USB_CAN_A binary protocol:
		 *   0xAA [INFO] [ID_LO] [ID_HI] [DATA...] 0x55
		 */
		uint8_t buf[16];
		buf[pos++] = 0xAA;
		buf[pos++] = (uint8_t)(0xC0 | dlc);
		buf[pos++] = (uint8_t)(id & 0xFF);
		buf[pos++] = (uint8_t)((id >> 8) & 0xFF);

		for (int i = 0; i < dlc; i++) {
			buf[pos++] = data[i];
		}

		buf[pos++] = 0x55;
		ret = slcan_write((const char *)buf, pos);
	}

	/* CH340 USB buffer صغير — delay بين الأوامر لمنع overflow على سيرفو 3 و 4 */
	px4_usleep(500);  // 0.5ms

	if (ret != pos) {
		_tx_fail_count++;
		LOGE_CAN("can_send FAIL: id=0x%03X dlc=%d wrote=%d/%d", id, dlc, ret, pos);
		return -1;
	}

	/* Log first few sends to PX4 console for debugging */
	static int send_log_count = 0;
	if (send_log_count < 5) {
		PX4_INFO("CAN TX: id=0x%03X dlc=%d [%02X %02X %02X %02X %02X %02X %02X %02X]",
			 id, dlc, data[0], data[1], data[2], data[3],
			 dlc > 4 ? data[4] : 0, dlc > 5 ? data[5] : 0,
			 dlc > 6 ? data[6] : 0, dlc > 7 ? data[7] : 0);
		send_log_count++;
	}

	return 0;
}

int XqpowerCan::can_receive()
{
	if (!_can_initialized || _usb_fd < 0) {
		return 0;
	}

	/* Read available bytes from USB-Serial */
	char tmp[256];
	int n = slcan_read(tmp, sizeof(tmp));

	if (n <= 0) {
		return 0;
	}

	/* Append to ring buffer */
	for (int i = 0; i < n; i++) {
		if (_slcan_rxpos < (int)sizeof(_slcan_rxbuf)) {
			_slcan_rxbuf[_slcan_rxpos++] = tmp[i];
		}
	}

	/* Debug: dump first raw bytes received from CAN_LIN_Tool */
	if (_is_canlin_tool) {
		static int rx_dump_count = 0;
		if (rx_dump_count < 5 && n > 0) {
			char hex[128] = {};
			int show = n < 40 ? n : 40;
			for (int i = 0; i < show; i++) {
				snprintf(hex + i*3, 4, "%02X ", (uint8_t)tmp[i]);
			}
			LOGI_CAN("RX RAW (%d bytes): %s", n, hex);
			rx_dump_count++;
		}
	}

	int count = 0;
	int rd = 0;

	if (_is_canlin_tool) {
		/*
		 * CAN_LIN_Tool V6.0 receive protocol — same 20-byte structure as TX
		 * but RX raw dump shows different layout:
		 *   [0]     DLC (0x08)
		 *   [1]     flags/count
		 *   [2-3]   CAN ID (little-endian, includes 0x580 base for SDO responses)
		 *   [4-7]   extended ID (zeros for standard)
		 *   [8]     DLC again
		 *   [9-16]  data[8]
		 *
		 * Alternative: scan for known CAN IDs (0x581-0x584, 0x000)
		 * in the byte stream since exact framing is unclear.
		 */

		/* Scan for 0x81 0x05 / 0x82 0x05 / 0x83 0x05 / 0x84 0x05 patterns
		 * (little-endian 0x0581-0x0584 = SDO responses from servos) */
		while (rd + 17 <= _slcan_rxpos) {
			uint8_t b0 = (uint8_t)_slcan_rxbuf[rd];

			/* Look for DLC=0x08 or DLC=0x02 as frame start */
			if (b0 != 0x08 && b0 != 0x02) {
				rd++;
				continue;
			}

			/* Check if [rd+2..rd+3] looks like a valid CAN ID */
			if (rd + 17 > _slcan_rxpos) break;

			uint16_t can_id = (uint16_t)((uint8_t)_slcan_rxbuf[rd + 2] |
						     ((uint8_t)_slcan_rxbuf[rd + 3] << 8));

			uint8_t dlc = b0;  /* first byte is DLC */

			if (dlc > 8) { rd++; continue; }

			/* Validate CAN ID — XQPOWER uses 0x000, 0x580+node, 0x180+node, 0x600+node */
			bool valid_id = (can_id == 0x000) ||
					(can_id >= 0x180 && can_id <= 0x184) ||
					(can_id >= 0x580 && can_id <= 0x584) ||
					(can_id >= 0x600 && can_id <= 0x604);

			if (!valid_id) { rd++; continue; }

			/* Frame layout: [DLC] [flags] [ID_LO] [ID_HI] [ext0..ext3] [DLC2] [data0..data7]
			 * Total = 1 + 1 + 2 + 4 + 1 + 8 = 17 bytes */
			uint8_t data[8] = {};
			for (int j = 0; j < 8 && j < dlc; j++) {
				data[j] = (uint8_t)_slcan_rxbuf[rd + 9 + j];
			}

			servo_process_rx(can_id, data, dlc);
			count++;
			rd += 17;
		}

	} else {
		/*
		 * Waveshare binary frames:
		 *   0xAA [INFO] [ID_LO] [ID_HI] [DATA...] 0x55
		 */
		while (rd < _slcan_rxpos) {
			if ((uint8_t)_slcan_rxbuf[rd] != 0xAA) { rd++; continue; }

			if (rd + 4 >= _slcan_rxpos) { break; }

			uint8_t info = (uint8_t)_slcan_rxbuf[rd + 1];
			bool is_extended = (info & 0x20) != 0;
			uint8_t dlc = info & 0x0F;

			if (dlc > 8) { rd++; continue; }

			int id_size = is_extended ? 4 : 2;
			int frame_size = 1 + 1 + id_size + dlc + 1;

			if (rd + frame_size > _slcan_rxpos) { break; }
			if ((uint8_t)_slcan_rxbuf[rd + frame_size - 1] != 0x55) { rd++; continue; }

			uint16_t can_id = (uint16_t)((uint8_t)_slcan_rxbuf[rd + 2] |
						     ((uint8_t)_slcan_rxbuf[rd + 3] << 8));

			uint8_t data[8] = {};
			int data_start = rd + 1 + 1 + id_size;
			for (int j = 0; j < dlc; j++) {
				data[j] = (uint8_t)_slcan_rxbuf[data_start + j];
			}

			servo_process_rx(can_id, data, dlc);
			count++;
			rd += frame_size;
		}
	}

	/* Compact buffer: move unprocessed data to front */
	if (rd > 0) {
		int remaining = _slcan_rxpos - rd;

		if (remaining > 0) {
			memmove(_slcan_rxbuf, _slcan_rxbuf + rd, remaining);
		}

		_slcan_rxpos = remaining;
	}

	return count;
}

#else /* NuttX — FDCAN register access */

bool XqpowerCan::can_open()
{
	/* Map FDCAN1 registers */
	_fdcan = reinterpret_cast<volatile FDCAN_Regs_t *>(FDCAN1_BASE_ADDR);

	/* Step 1: Enable FDCAN peripheral clock via RCC */
	volatile uint32_t *rcc_apb1henr  = reinterpret_cast<volatile uint32_t *>(RCC_BASE_ADDR + RCC_APB1HENR_OFFSET);
	volatile uint32_t *rcc_apb1hrstr = reinterpret_cast<volatile uint32_t *>(RCC_BASE_ADDR + RCC_APB1HRSTR_OFFSET);

	*rcc_apb1henr  |= RCC_FDCANEN_BIT;
	*rcc_apb1hrstr |= RCC_FDCANRST_BIT;
	*rcc_apb1hrstr &= ~RCC_FDCANRST_BIT;

	for (volatile int i = 0; i < 1000; i++) {}

	/* Step 2: Exit sleep mode and enter Init mode */
	_fdcan->CCCR &= ~FDCAN_CCCR_CSR;

	for (int timeout = 0; timeout < 10000; timeout++) {
		if ((_fdcan->CCCR & FDCAN_CCCR_CSA) == 0) { break; }
	}

	if (_fdcan->CCCR & FDCAN_CCCR_CSA) {
		PX4_ERR("FDCAN1: failed to exit sleep mode");
		return false;
	}

	_fdcan->CCCR |= FDCAN_CCCR_INIT;

	for (int timeout = 0; timeout < 10000; timeout++) {
		if (_fdcan->CCCR & FDCAN_CCCR_INIT) { break; }
	}

	if (!(_fdcan->CCCR & FDCAN_CCCR_INIT)) {
		PX4_ERR("FDCAN1: failed to enter init mode");
		return false;
	}

	_fdcan->CCCR |= FDCAN_CCCR_CCE;

	/* Step 3: Bit timing for 500 kbps (HSE = 8 MHz, SP = 87.5%) */
	_fdcan->NBTP = (0U  << FDCAN_NBTP_NSJW_Pos)   |
		       (12U << FDCAN_NBTP_NTSEG1_Pos)  |
		       (1U  << FDCAN_NBTP_TSEG2_Pos)   |
		       (0U  << FDCAN_NBTP_NBRP_Pos);

	_fdcan->DBTP = (0U << 0) | (12U << 8) | (1U << 4) | (0U << 16);
	_fdcan->CCCR &= ~FDCAN_CCCR_FDOE;
	_fdcan->IE = 0;

	/* Step 4: Message RAM */
	uint32_t ram_offset = 0;

	_fdcan->SIDFC = 0;
	_fdcan->XIDFC = 0;
	_fdcan->RXESC = 0;
	_fdcan->TXESC = 0;

	_rx_fifo0_base = SRAMCAN_BASE_ADDR + ram_offset * 4;
	_fdcan->RXF0C = (ram_offset << FDCAN_RXF0C_F0SA_Pos) |
			(XQCAN_RX_FIFO0_ELEMENTS << FDCAN_RXF0C_F0S_Pos);
	ram_offset += XQCAN_RX_FIFO0_ELEMENTS * FDCAN_ELEMENT_WORDS;

	_tx_fifo_base = SRAMCAN_BASE_ADDR + ram_offset * 4;
	_fdcan->TXBC = (ram_offset << FDCAN_TXBC_TBSA_Pos) |
		       (XQCAN_TX_FIFO_ELEMENTS << FDCAN_TXBC_TFQS_Pos);
	_fdcan->TXBC &= ~FDCAN_TXBC_TFQM;

	/* Step 5: Accept all standard frames, reject extended */
	_fdcan->GFC = 0;
	_fdcan->GFC |= (0x3U << FDCAN_GFC_ANFE_Pos);

	_fdcan->IR = 0xFFFFFFFF;

	/* Step 6: Exit Init mode */
	_fdcan->CCCR &= ~FDCAN_CCCR_INIT;

	for (int timeout = 0; timeout < 10000; timeout++) {
		if ((_fdcan->CCCR & FDCAN_CCCR_INIT) == 0) { break; }
	}

	if (_fdcan->CCCR & FDCAN_CCCR_INIT) {
		PX4_ERR("FDCAN1: failed to exit init mode");
		return false;
	}

	_can_initialized = true;
	PX4_INFO("FDCAN1 initialized: 500 kbps, SP=87.5%%");
	return true;
}

void XqpowerCan::can_close()
{
	if (_can_initialized && _fdcan) {
		_fdcan->CCCR |= FDCAN_CCCR_INIT;
		_can_initialized = false;
	}
}

int XqpowerCan::can_send(uint16_t id, const uint8_t *data, uint8_t dlc)
{
	if (!_can_initialized || !_fdcan || dlc > 8) {
		return -1;
	}

	if (_fdcan->TXFQS & FDCAN_TXFQS_TFQF) {
		_tx_fail_count++;

		if (_tx_fail_count % 100 == 1) {
			PX4_WARN("xqpower: CAN TX FIFO full — %u commands dropped", (unsigned)_tx_fail_count);
		}

		return -1;
	}

	uint8_t index = (_fdcan->TXFQS & FDCAN_TXFQS_TFQPI_Msk) >> FDCAN_TXFQS_TFQPI_Pos;
	volatile uint32_t *txbuf = reinterpret_cast<volatile uint32_t *>(
					   _tx_fifo_base + index * FDCAN_ELEMENT_BYTES);

	txbuf[0] = (uint32_t)(id & 0x7FF) << 18;
	txbuf[1] = (uint32_t)(dlc & 0xF) << 16;

	txbuf[2] = (uint32_t)(data[0])       |
		   (uint32_t)(data[1]) << 8  |
		   (uint32_t)(data[2]) << 16 |
		   (uint32_t)(data[3]) << 24;

	if (dlc > 4) {
		txbuf[3] = (uint32_t)(data[4])       |
			   (uint32_t)(data[5]) << 8  |
			   (uint32_t)(data[6]) << 16 |
			   (uint32_t)(data[7]) << 24;
	} else {
		txbuf[3] = 0;
	}

	_fdcan->TXBAR = 1U << index;

	return 0;
}

int XqpowerCan::can_receive()
{
	if (!_can_initialized || !_fdcan) {
		return 0;
	}

	int count = 0;

	while ((_fdcan->RXF0S & FDCAN_RXF0S_F0FL_Msk) > 0) {
		uint8_t gi = (_fdcan->RXF0S & FDCAN_RXF0S_F0GI_Msk) >> FDCAN_RXF0S_F0GI_Pos;
		volatile uint32_t *rxbuf = reinterpret_cast<volatile uint32_t *>(
						   _rx_fifo0_base + gi * FDCAN_ELEMENT_BYTES);

		uint32_t r0 = rxbuf[0];
		uint32_t r1 = rxbuf[1];
		uint32_t r2 = rxbuf[2];
		uint32_t r3 = rxbuf[3];

		if (r0 & (1U << 30)) {
			_fdcan->RXF0A = gi;
			continue;
		}

		uint16_t can_id = (uint16_t)((r0 >> 18) & 0x7FF);
		uint8_t dlc = (uint8_t)((r1 >> 16) & 0xF);

		if (dlc > 8) { dlc = 8; }

		uint8_t data[8];
		data[0] = (uint8_t)(r2);
		data[1] = (uint8_t)(r2 >> 8);
		data[2] = (uint8_t)(r2 >> 16);
		data[3] = (uint8_t)(r2 >> 24);
		data[4] = (uint8_t)(r3);
		data[5] = (uint8_t)(r3 >> 8);
		data[6] = (uint8_t)(r3 >> 16);
		data[7] = (uint8_t)(r3 >> 24);

		_fdcan->RXF0A = gi;

		servo_process_rx(can_id, data, dlc);
		count++;
	}

	return count;
}

#endif /* __PX4_POSIX / NuttX */

/* ========================== XQPOWER Protocol =============================== */
/* Identical on both platforms — uses can_send()/can_receive() abstraction */

void XqpowerCan::servo_set_position(int index, float angle_deg)
{
	if (index < 0 || index >= XQPOWER_MAX_SERVOS) {
		return;
	}

	if (_reverse_mask & (1u << index)) {
		angle_deg = -angle_deg;
	}

	if (angle_deg >  _angle_limit) { angle_deg =  _angle_limit; }
	if (angle_deg < -_angle_limit) { angle_deg = -_angle_limit; }

	/* Convert to XQPOWER raw units: 18 units per degree (confirmed by hardware test) */
	int16_t position = (int16_t)(angle_deg * 18.0f);

	uint8_t data[8];
	data[0] = XQPOWER_SDO_WRITE;
	data[1] = 0x03;
	data[2] = 0x60;
	data[3] = 0x00;
	data[4] = (uint8_t)(position & 0xFF);
	data[5] = (uint8_t)((position >> 8) & 0xFF);
	data[6] = 0x00;
	data[7] = 0x00;

	uint16_t can_id = XQPOWER_SDO_TX_BASE + _node_ids[index];

	/* Log first position commands to PX4 console */
	static int pos_log_count = 0;
	if (pos_log_count < 8) {
		PX4_INFO("SET_POS[%d]: %.1f deg raw=%d -> id=0x%03X [%02X %02X %02X %02X %02X %02X %02X %02X]",
			 index, (double)angle_deg, position, can_id,
			 data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
		pos_log_count++;
	}

	can_send(can_id, data, 8);
}

void XqpowerCan::servo_set_report_interval(int index, uint8_t interval_ms)
{
	if (index < 0 || index >= XQPOWER_MAX_SERVOS) {
		return;
	}

	if (interval_ms < 10) { interval_ms = 10; }

	uint8_t data[8] = {0};
	data[0] = XQPOWER_SDO_WRITE;
	data[1] = 0x00;
	data[2] = 0x22;
	data[3] = 0x00;
	data[4] = interval_ms;

	uint16_t can_id = XQPOWER_SDO_TX_BASE + _node_ids[index];
	can_send(can_id, data, 8);
}

void XqpowerCan::servo_read_position(int index)
{
	if (index < 0 || index >= XQPOWER_MAX_SERVOS) {
		return;
	}

	uint8_t data[8] = {0};
	data[0] = XQPOWER_SDO_READ;
	data[1] = 0x02;
	data[2] = 0x60;

	uint16_t can_id = XQPOWER_SDO_TX_BASE + _node_ids[index];
	can_send(can_id, data, 8);
}

void XqpowerCan::servo_start_report(int index)
{
	if (index < 0 || index >= XQPOWER_MAX_SERVOS) {
		return;
	}

	/* NMT Start Node — CAN ID 0x000, data=[0x01, node_id]
	 * Transitions servo from Pre-Operational to Operational state.
	 * Without this, the servo ignores position write commands (0x6003). */
	uint8_t data[2] = {0x01, _node_ids[index]};
	can_send(0x000, data, 2);
}

void XqpowerCan::servo_stop_report(int index)
{
	if (index < 0 || index >= XQPOWER_MAX_SERVOS) {
		return;
	}

	/* NMT Stop Node — CAN ID 0x000 */
	uint8_t data[2] = {0x02, _node_ids[index]};
	can_send(0x000, data, 2);
}

void XqpowerCan::servo_read_status(int index)
{
	if (index < 0 || index >= XQPOWER_MAX_SERVOS) {
		return;
	}

	uint8_t data[8] = {0};
	data[0] = XQPOWER_SDO_READ;
	data[1] = 0x05;  /* Index 0x6005 (temp+current) — low byte */
	data[2] = 0x60;  /* Index 0x6005 — high byte */

	uint16_t can_id = XQPOWER_SDO_TX_BASE + _node_ids[index];
	can_send(can_id, data, 8);
}

void XqpowerCan::servo_process_rx(uint16_t id, const uint8_t *data, uint8_t dlc)
{
	if (dlc < 2) {
		return;
	}

	uint8_t node_id;
	bool is_sdo;

	if (id >= XQPOWER_SDO_RX_BASE + 1 && id <= XQPOWER_SDO_RX_BASE + 127) {
		node_id = (uint8_t)(id - XQPOWER_SDO_RX_BASE);
		is_sdo = true;

	} else if (id >= XQPOWER_PDO_RX_BASE + 1 && id <= XQPOWER_PDO_RX_BASE + 127) {
		node_id = (uint8_t)(id - XQPOWER_PDO_RX_BASE);
		is_sdo = false;

	} else {
		return;
	}

	int idx = -1;

	for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
		if (_node_ids[i] == node_id) {
			idx = i;
			break;
		}
	}

	if (idx < 0) {
		return;
	}

	_feedback[idx].online = true;
	_feedback[idx].last_update_us = hrt_absolute_time();

	if (is_sdo && dlc >= 4) {
		uint8_t cmd = data[0];

		/*
		 * XQPOWER sends both SDO responses AND auto-report on CAN ID 0x580+node.
		 * Distinguish by checking if data[0] is a valid SDO response command byte.
		 * SDO responses: 0x42, 0x43, 0x4B, 0x4F (read), 0x60 (write ACK), 0x80 (abort)
		 * Auto-report:   data[0] is position_low byte (any value)
		 */
		bool is_sdo_response = (cmd == 0x42 || cmd == 0x43 || cmd == 0x4B || cmd == 0x4F ||
					cmd == 0x60 || cmd == 0x80);

		if (is_sdo_response) {
			_sdo_rx_count++;
			uint16_t index = (uint16_t)(data[1] | (data[2] << 8));

			/* SDO write ACK */
			static int sdo_ack_log = 0;
			if (cmd == 0x60 && sdo_ack_log < 8) {
				PX4_INFO("SDO ACK node=0x%02X idx=0x%04X", node_id, index);
				sdo_ack_log++;
			}

			/* SDO Abort — servo REJECTED our command */
			static int sdo_abort_log = 0;
			if (cmd == 0x80 && sdo_abort_log < 8) {
				uint32_t abort_code = 0;
				if (dlc >= 8) {
					abort_code = (uint32_t)data[4] | ((uint32_t)data[5] << 8) |
						     ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);
				}
				PX4_ERR("SDO ABORT node=0x%02X idx=0x%04X abort=0x%08X", node_id, index, abort_code);
				sdo_abort_log++;
			}

			/* SDO read responses */
			if (cmd == 0x42 || cmd == 0x4F || cmd == 0x4B || cmd == 0x43) {
				if (index == 0x6002 && dlc >= 6) {
					int16_t pos_raw = (int16_t)(data[4] | (data[5] << 8));
					_feedback[idx].position_deg = pos_raw / 18.0f;

				} else if (index == 0x6005 && dlc >= 7) {
					_feedback[idx].current_mA = (uint16_t)(data[4] | (data[5] << 8));
					_feedback[idx].temperature_01C = (int16_t)(data[6]);
				}
			}

		} else {
			/* Auto-report position data on 0x580+node (not SDO format) */
			_pdo_rx_count++;
			int16_t pos_raw = (int16_t)(data[0] | (data[1] << 8));

			if (pos_raw >= -800 && pos_raw <= 800) {
				_feedback[idx].position_deg = pos_raw / 18.0f;
			}
		}

	} else {
		/* PDO from 0x180+node (standard CANopen PDO) */
		_pdo_rx_count++;
		int16_t pos_raw = (int16_t)(data[0] | (data[1] << 8));

		if (pos_raw >= -800 && pos_raw <= 800) {
			_feedback[idx].position_deg = pos_raw / 18.0f;
		}
	}
}

/*
 * Servo initialization sequence — warm up adapter, start NMT, configure reports.
 * Called from init() if CAN is already open, or from Run() on late USB connect.
 */
void XqpowerCan::_servo_init_sequence()
{
	/*
	 * Step 0: Send 5 raw test frames to warm up Waveshare adapter CAN TX.
	 *         These go to non-existent node 0x25 so no servo is affected.
	 */
	for (int t = 0; t < 5; t++) {
		uint8_t test_frame[] = {
			0xAA,                         /* Header */
			0xC8,                         /* INFO: 0xC0 | 8 */
			0x25, 0x06,                   /* CAN ID 0x0625 (little-endian) */
			0x22, 0x03, 0x60, 0x00,       /* SDO write idx=0x6003 */
			0xE8, 0x03, 0x00, 0x00,       /* Value 1000 */
			0x55                          /* Footer */
		};
		slcan_write((const char *)test_frame, sizeof(test_frame));
		px4_usleep(200000);  /* 200ms between test frames */
	}

	PX4_INFO("Adapter warm-up: 5 test frames sent");

	/* Step 1: NMT Start Node — put each servo into Operational state */
	for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
		servo_start_report(i);
		px4_usleep(50000);
	}

	PX4_INFO("NMT Start sent to all servos");

	/* Step 2: Configure report interval (from XQCAN_FB_MS param).
	 * Driver clamps <10 ms up to 10 ms; we clamp >255 ms down so the
	 * uint8_t CAN payload byte doesn't wrap.
	 */
	int32_t fb_ms_param = _param_fb_interval_ms.get();
	if (fb_ms_param < 10)  { fb_ms_param = 10; }
	if (fb_ms_param > 255) { fb_ms_param = 255; }
	const uint8_t fb_interval_ms = (uint8_t)fb_ms_param;
	PX4_INFO("Configuring PDO auto-report interval: %u ms (XQCAN_FB_MS)",
		 (unsigned)fb_interval_ms);
	for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
		servo_set_report_interval(i, fb_interval_ms);
		px4_usleep(20000);
	}

	_report_started = true;
	PX4_INFO("Servo init sequence complete — reports started");
}

/* ========================== PX4 Work Item ================================== */

void XqpowerCan::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

#ifdef __PX4_POSIX

	/* Check if USB FD changed (late connect or reconnect) */
	int current_fd  = s_usb_fd.load(std::memory_order_acquire);
	int consumed_fd = s_consumed_fd.load(std::memory_order_relaxed);

	if (current_fd != consumed_fd) {
		if (current_fd >= 0) {
			PX4_INFO("xqpower: USB connecting (fd=%d)", current_fd);
			can_close();

			if (usb_serial_open(current_fd)) {
				s_consumed_fd.store(current_fd, std::memory_order_release);
				_slcan_rxpos = 0;

				/* Flush stale data */
				char flush[64];
				slcan_read(flush, sizeof(flush));

				_can_initialized = true;
				PX4_INFO("xqpower: Waveshare USB_CAN_A connected");

				/* Run full servo init (warmup + NMT + reports) */
				_servo_init_sequence();

			} else {
				_can_initialized = false;
			}

		} else {
			/* FD set to -1: USB disconnected */
			can_close();
			s_consumed_fd.store(-1, std::memory_order_release);
		}
	}

	if (!_can_initialized) {
		return;
	}

#endif

	/* Read any available CAN frames (feedback) */
	int rx_count = can_receive();

	/* Check armed state */
	actuator_armed_s armed;

	if (_actuator_armed_sub.update(&armed)) {
		bool was_armed = _armed;
		_armed = armed.armed;

		if (_armed && !was_armed) {
			for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
				servo_set_position(i, 0.0f);
				_cmd_deg[i] = 0.0f;
			}

			PX4_INFO("xqpower: armed — fins zeroed");
		}

		if (!_armed && was_armed) {
			for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
				servo_set_position(i, 0.0f);
				_cmd_deg[i] = 0.0f;
			}

			PX4_INFO("xqpower: disarmed — fins zeroed");
		}
	}

	/*
	 * CAN TX every cycle (50 Hz).
	 * USB_CAN_A at 2 Mbps: 5 frames × 0.1ms = 0.5ms per 20ms cycle = 2.5%
	 */
	const bool tx_cycle = true;

	/* Read fin commands (same priority system on both platforms) */
	if (_override_active.load(std::memory_order_acquire)) {
		for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
			const float a = _override_angles[i].load(std::memory_order_relaxed);
			_cmd_deg[i] = a;

			if (tx_cycle) {
				servo_set_position(i, a);
			}
		}

	} else if (_armed) {
		/*
		 * Unified single-path: actuator_servos (normalized [-1,+1]) in ALL modes.
		 *
		 * RocketMPC now publishes actuator_servos in both HITL and real flight.
		 * actuator_outputs_sim (radians) is still published by RocketMPC for
		 * SimulatorMavlink lockstep but is NO LONGER read here — removing the
		 * dual-encoding ambiguity and making HITL/real behaviour identical.
		 *
		 * Priority:
		 *   1. actuator_servos  — rocket_mpc (NORMALIZED, same in HITL + real)
		 *   2. actuator_outputs — legacy PWM-style (fallback)
		 */

		// Drain actuator_outputs_sim to avoid stale messages building up in the
		// uORB queue (rocket_mpc still publishes it for lockstep).  We read but
		// discard the payload.
		{ actuator_outputs_s _discard; _actuator_outputs_sim_sub.update(&_discard); }

		actuator_servos_s servos;

		if (_actuator_servos_sub.update(&servos)) {
			for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
				float val = servos.control[i];

				if (!PX4_ISFINITE(val)) {
					continue;
				}

				/* actuator_servos.control[] is normalized [-1,+1]; scale to degrees */
				float angle_deg = val * _angle_limit;
				_cmd_deg[i] = angle_deg;

				if (tx_cycle) {
					servo_set_position(i, angle_deg);
				}
			}

		} else {
			actuator_outputs_s outputs;

			if (_actuator_outputs_sub.update(&outputs)) {
				for (int i = 0; i < XQPOWER_MAX_SERVOS && i < (int)outputs.noutputs; i++) {
					float normalized = outputs.output[i] / 500.0f - 1.0f;
					float angle = normalized * _angle_limit;

					if (tx_cycle) {
						servo_set_position(i, angle);
					}
				}
			}
		}

	} else {
		/* Disarmed: send zero every 50 cycles (~1 Hz at 50Hz) */
		if (++_disarm_zero_count >= 50) {
			_disarm_zero_count = 0;

			for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
				servo_set_position(i, 0.0f);
				_cmd_deg[i] = 0.0f;
			}
		}
	}

	/* Publish servo feedback every cycle (50 Hz) */
	{
		debug_array_s dbg{};
		dbg.timestamp = hrt_absolute_time();
		dbg.id = 1;
		strncpy(dbg.name, "SRV_FB", sizeof(dbg.name));
		uint8_t online_mask = 0;

		for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
			dbg.data[i]     = _cmd_deg[i];
			dbg.data[i + 4] = _feedback[i].position_deg;
			dbg.data[i + 8] = _cmd_deg[i] - _feedback[i].position_deg;

			if (_feedback[i].online) { online_mask |= (1u << i); }
		}

		dbg.data[12] = (float)online_mask;
		dbg.data[13] = (float)_tx_fail_count;

		/*
		 * Per-servo feedback age (ms since last CAN frame for that servo).
		 * Consumed by rocket_mpc (see RocketMPC.cpp feedback-only block)
		 * to gate the SRV_FB back-solve on CAN_FRESH_US=120 ms.  Online
		 * mask alone is not enough — a servo can be marked online (its
		 * 2-second online timeout has not elapsed) while its last
		 * position update is already 200 ms old, which would let a stale
		 * value drive _de_act. We publish a clamped millisecond counter:
		 *   - 0 when last_update_us == 0 (no frame yet received) or in the
		 *     future relative to now (clock skew guard).
		 *   - capped at UINT16_MAX ms to fit a float with full resolution.
		 */
		const hrt_abstime fb_now = dbg.timestamp;

		for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
			uint32_t age_ms = 0;

			if (_feedback[i].last_update_us != 0
			    && fb_now >= _feedback[i].last_update_us) {
				const hrt_abstime age_us = fb_now - _feedback[i].last_update_us;
				uint64_t age_ms_u64 = age_us / 1000ULL;

				if (age_ms_u64 > 65535ULL) { age_ms_u64 = 65535ULL; }

				age_ms = (uint32_t)age_ms_u64;
			} else {
				/* No frame yet — mark stale-to-consumer with max age. */
				age_ms = 65535u;
			}

			dbg.data[14 + i] = (float)age_ms;
		}

		_servo_fb_pub.publish(dbg);
	}

	/* Position polling: 1 servo per TX cycle (round-robin, ~5 Hz per servo) */
	if (tx_cycle) {
		servo_read_position(_pos_servo_idx);
		_pos_servo_idx = (_pos_servo_idx + 1) % XQPOWER_MAX_SERVOS;
	}

	/* Status polling (round-robin, every 2.5s at 50Hz) */
	if (++_status_cycle_count >= 125) {
		servo_read_status(_status_servo_idx);
		_status_servo_idx = (_status_servo_idx + 1) % XQPOWER_MAX_SERVOS;
		_status_cycle_count = 0;
	}

	/* Diagnostic: log RX count after 5 seconds */
	_total_rx += rx_count;

	if (++_rx_diag_count >= 250) {
		PX4_INFO("CAN RX: %d frames (SDO=%u PDO=%u) | S0=%.1f S1=%.1f S2=%.1f S3=%.1f deg | TXfail=%u | read: data=%d zero=%d err=%d",
			 _total_rx, _sdo_rx_count, _pdo_rx_count,
			 (double)_feedback[0].position_deg,
			 (double)_feedback[1].position_deg,
			 (double)_feedback[2].position_deg,
			 (double)_feedback[3].position_deg,
			 _tx_fail_count,
			 _read_data_count, _read_zero_count, _read_err_count);
		_rx_diag_count = 0;
	}

	/* Check servo online timeout (2 seconds) */
	hrt_abstime now = hrt_absolute_time();

	for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
		if (_feedback[i].online && (now - _feedback[i].last_update_us) > 2000000) {
			_feedback[i].online = false;
		}
	}
}

/* ========================== Module Interface ================================ */

int XqpowerCan::task_spawn(int argc, char *argv[])
{
	XqpowerCan *instance = new XqpowerCan();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init()) {
		return PX4_OK;
	}

	PX4_ERR("init failed");
	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int XqpowerCan::custom_command(int argc, char *argv[])
{
	XqpowerCan *obj = get_instance();

	if (argc > 0) {

		/* ---- move <servo> <angle> ---- */
		if (!strcmp(argv[0], "move") && argc >= 3) {
			if (obj == nullptr) { PX4_ERR("not running"); return PX4_ERROR; }

			int servo = atoi(argv[1]);
			float angle = (float)atof(argv[2]);

			if (servo < 0 || servo >= XQPOWER_MAX_SERVOS) {
				PX4_ERR("servo index 0..%d", XQPOWER_MAX_SERVOS - 1);
				return PX4_ERROR;
			}

			obj->set_override_angle(servo, angle);
			obj->enable_override();
			PX4_INFO("Servo[%d] -> %.1f deg (override ON)", servo, (double)angle);
			obj->servo_set_position(servo, angle);
			return PX4_OK;
		}

		/* ---- move_all <angle> ---- */
		if (!strcmp(argv[0], "move_all") && argc >= 2) {
			if (obj == nullptr) { PX4_ERR("not running"); return PX4_ERROR; }

			float angle = (float)atof(argv[1]);
			PX4_INFO("All servos -> %.1f deg (override ON)", (double)angle);

			for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
				obj->set_override_angle(i, angle);
				obj->servo_set_position(i, angle);
				px4_usleep(20000); /* 20ms between sends */
			}

			obj->enable_override();
			return PX4_OK;
		}

		/* ---- zero ---- */
		if (!strcmp(argv[0], "zero")) {
			if (obj == nullptr) { PX4_ERR("not running"); return PX4_ERROR; }

			PX4_INFO("All servos -> 0 deg (override ON)");

			for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
				obj->set_override_angle(i, 0.0f);
				obj->servo_set_position(i, 0.0f);
			}

			obj->enable_override();
			return PX4_OK;
		}

		/* ---- calibrate ---- */
		if (!strcmp(argv[0], "calibrate")) {
			if (obj == nullptr) { PX4_ERR("not running"); return PX4_ERROR; }

			PX4_INFO("Calibrating zero point: setting current position as 0 for all servos...");

			for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
				/* SDO write: Index 0x3009 SubIndex 0x00, Value=1 (zero calibrate) */
				uint8_t cal_data[8] = {0x22, 0x09, 0x30, 0x00, 0x01, 0x00, 0x00, 0x00};
				uint16_t can_id = XQPOWER_SDO_TX_BASE + obj->_node_ids[i];
				obj->can_send(can_id, cal_data, 8);
			}

			px4_usleep(500000); /* Wait 500ms for calibration */

			/* Save settings to flash */
			for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
				uint8_t save_data[8] = {0x22, 0x10, 0x10, 0x01, 0x73, 0x61, 0x76, 0x65}; /* "save" */
				uint16_t can_id = XQPOWER_SDO_TX_BASE + obj->_node_ids[i];
				obj->can_send(can_id, save_data, 8);
			}

			PX4_INFO("Zero calibration complete and saved. Current position is now 0 deg.");
			return PX4_OK;
		}

		/* ---- release ---- */
		if (!strcmp(argv[0], "release")) {
			if (obj == nullptr) { PX4_ERR("not running"); return PX4_ERROR; }

			obj->disable_override();
			PX4_INFO("Manual override released — returning to normal control");
			return PX4_OK;
		}

		/* ---- test ---- */
		if (!strcmp(argv[0], "test")) {
			if (obj == nullptr) { PX4_ERR("not running"); return PX4_ERROR; }

			PX4_INFO("Testing servos: 0 deg");

			for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
				obj->servo_set_position(i, 0.0f);
			}

			px4_usleep(1000000);

			PX4_INFO("Testing servos: +10 deg");

			for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
				obj->servo_set_position(i, 10.0f);
			}

			px4_usleep(1000000);

			PX4_INFO("Testing servos: -10 deg");

			for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
				obj->servo_set_position(i, -10.0f);
			}

			px4_usleep(1000000);

			PX4_INFO("Testing servos: 0 deg");

			for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
				obj->servo_set_position(i, 0.0f);
			}

			PX4_INFO("Test complete");
			return PX4_OK;
		}

		/* ---- sweep <angle> ---- */
		if (!strcmp(argv[0], "sweep") && argc >= 2) {
			if (obj == nullptr) { PX4_ERR("not running"); return PX4_ERROR; }

			float max_angle = (float)atof(argv[1]);
			PX4_INFO("Sweep: 0 -> +%.1f -> -%.1f -> 0", (double)max_angle, (double)max_angle);

			for (float a = 0.0f; a <= max_angle; a += 1.0f) {
				for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
					obj->servo_set_position(i, a);
				}

				px4_usleep(30000);
			}

			for (float a = max_angle; a >= -max_angle; a -= 1.0f) {
				for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
					obj->servo_set_position(i, a);
				}

				px4_usleep(30000);
			}

			for (float a = -max_angle; a <= 0.0f; a += 1.0f) {
				for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
					obj->servo_set_position(i, a);
				}

				px4_usleep(30000);
			}

			PX4_INFO("Sweep complete");
			return PX4_OK;
		}

		/* ---- status ---- */
		if (!strcmp(argv[0], "status")) {
			if (obj == nullptr) { PX4_ERR("not running"); return PX4_ERROR; }

			return obj->print_status();
		}
	}

	return print_usage("unknown command");
}

int XqpowerCan::print_status()
{
	PX4_INFO("XQPOWER CAN Servo Driver");

#ifdef __PX4_POSIX
	PX4_INFO("  Transport: Waveshare USB_CAN_A (binary protocol)");
	PX4_INFO("  USB: %s (fd=%d)", _usb_fd >= 0 ? "CONNECTED" : "NOT CONNECTED", _usb_fd);
	PX4_INFO("  CAN: %s", _can_initialized ? "OPEN" : "CLOSED");
#else
	PX4_INFO("  Transport: FDCAN1 (STM32H7 direct register)");
	PX4_INFO("  FDCAN1: %s", _can_initialized ? "OK" : "NOT INITIALIZED");
#endif

	PX4_INFO("  Armed: %s", _armed ? "YES" : "NO");
	PX4_INFO("  Override: %s", _override_active.load(std::memory_order_acquire) ? "ACTIVE" : "off");
	PX4_INFO("  Angle limit: %.1f deg", (double)_angle_limit);
	PX4_INFO("  Reverse mask: 0x%X (S0=%s S1=%s S2=%s S3=%s)",
		 _reverse_mask,
		 (_reverse_mask & 1) ? "REV" : "---",
		 (_reverse_mask & 2) ? "REV" : "---",
		 (_reverse_mask & 4) ? "REV" : "---",
		 (_reverse_mask & 8) ? "REV" : "---");
	PX4_INFO("  TX fail count: %u", (unsigned)_tx_fail_count);

#ifndef __PX4_POSIX

	if (_can_initialized && _fdcan) {
		uint32_t ecr = _fdcan->ECR;
		uint32_t psr = _fdcan->PSR;
		PX4_INFO("  ECR: TEC=%lu REC=%lu", (unsigned long)(ecr & 0xFF), (unsigned long)((ecr >> 8) & 0x7F));
		PX4_INFO("  PSR: LEC=%lu ACT=%lu", (unsigned long)(psr & 0x7), (unsigned long)((psr >> 3) & 0x3));
	}

#endif

	for (int i = 0; i < XQPOWER_MAX_SERVOS; i++) {
		PX4_INFO("  Servo[%d] NodeID=0x%02X: %s  Pos=%.1f deg  Cmd=%.1f deg  I=%d mA  T=%.1f C",
			 i, _node_ids[i],
			 _feedback[i].online ? "ONLINE " : "OFFLINE",
			 (double)_feedback[i].position_deg,
			 (double)_cmd_deg[i],
			 _feedback[i].current_mA,
			 (double)(_feedback[i].temperature_01C / 10.0f));
	}

	return PX4_OK;
}

int XqpowerCan::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
XQPOWER CAN Bus Servo driver.
Controls up to 4 XQPOWER servos via CAN bus using CANopen SDO/PDO protocol.
NuttX: Direct FDCAN1 register access at 500 kbps.
Android: SLCAN via Waveshare USB_CAN_A (CH340) at 500 kbps.

### Examples
Start the driver:
$ xqpower_can start

Move servo 0 to 15 degrees:
$ xqpower_can move 0 15

Move all servos to -20 degrees:
$ xqpower_can move_all -20

Zero all servos:
$ xqpower_can zero

Smooth sweep all servos to +/-25 degrees:
$ xqpower_can sweep 25

Auto test (0 -> +10 -> -10 -> 0):
$ xqpower_can test

Show servo status:
$ xqpower_can status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("xqpower_can", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("move", "Move one servo and hold: move <servo_idx> <angle_deg>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("move_all", "Move all servos and hold: move_all <angle_deg>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("zero", "Zero all servos and hold (0 deg)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("calibrate", "Set current position as new zero reference (saves to flash)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("release", "Release manual override, return to normal control");
	PRINT_MODULE_USAGE_COMMAND_DESCR("sweep", "Smooth sweep: sweep <max_angle_deg>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Auto test: 0 -> +10 -> -10 -> 0 deg");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/* ========================== Module Entry Point ============================= */

extern "C" __EXPORT int xqpower_can_main(int argc, char *argv[])
{
	return XqpowerCan::main(argc, argv);
}
