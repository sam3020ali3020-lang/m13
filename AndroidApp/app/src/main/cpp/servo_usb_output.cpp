/**
 * servo_usb_output.cpp — Phase 11.2
 * يقرأ actuator_servos من PX4 uORB ويرسل أوامر XQPOWER عبر USB (libusb)
 *
 * البروتوكول: XQPOWER 5-byte per servo (من مشروع oc_p_servo_can_feedback)
 * الأجهزة: CP2102 (VID=0x10C4) عبر USB-C OTG Hub
 */
#include "servo_usb_output.h"

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/actuator_armed.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>

#include <libusb.h>

#include <android/log.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <cmath>
#include <unistd.h>
#include <cstring>

#define TAG_SERVO "ServoUSB"
#define LOGI_S(...) __android_log_print(ANDROID_LOG_INFO,  TAG_SERVO, __VA_ARGS__)
#define LOGE_S(...) __android_log_print(ANDROID_LOG_ERROR, TAG_SERVO, __VA_ARGS__)

// ===== XQPOWER Protocol Constants (من مشروع الصديق) =====
static const uint16_t SERVO_IDS[4] = {0x601, 0x602, 0x603, 0x604};
// MAX_ANGLE_DEG is the *hardware* deflection range (±25°) used for encoding
// into the 14-bit XQPOWER position register and as the final safety clamp in
// encode_servo().  It is intentionally decoupled from the *control* scaling
// limit used below (the XQCAN_LIMIT parameter, typically 20° to match
// SOLVER_DELTA_MAX_RAD).  Do not conflate them: the register scale must stay
// at 25° or the servo position mapping breaks.
static constexpr float MAX_ANGLE_DEG = 25.0f;
static constexpr int XQPOWER_CENTER = 8191;   // 14-bit center
static constexpr float XQPOWER_SCALE = 327.64f; // degrees → position units (= 8191 / 25.0)

// Fallback scaling limit if XQCAN_LIMIT cannot be read.  Matches
// SOLVER_DELTA_MAX_RAD (~0.3491 rad = 20°) from the acados solver.
static constexpr float DEFAULT_SCALING_LIMIT_DEG = 20.0f;

// ===== CP2102 USB-Serial Constants =====
static constexpr uint16_t CP2102_VID = 0x10C4;
static constexpr int CP2102_BAUD = 115200;
// CP2102 control transfer requests
static constexpr uint8_t CP210X_SET_BAUDRATE = 0x1E;
static constexpr uint8_t CP210X_IFC_ENABLE  = 0x00;
static constexpr uint8_t CP210X_SET_LINE_CTL = 0x03;
// Line control: 8N1 = 0x0800
static constexpr uint16_t CP210X_LINE_8N1 = 0x0800;

// ===== State =====
static std::atomic<bool> s_servo_running{false};
static std::thread s_servo_thread;
static std::atomic<int> s_usb_fd{-1};
static std::mutex s_usb_mutex;

// libusb state (protected by s_usb_mutex)
static libusb_context *s_ctx = nullptr;
static libusb_device_handle *s_dev_handle = nullptr;
static int s_bulk_endpoint_out = -1;

// ===== XQPOWER Encoder =====
// Encodes one servo command into 5 bytes
static void encode_servo(uint8_t* buf, uint16_t servoId, float angle_deg) {
    // Clamp to ±MAX_ANGLE_DEG
    if (angle_deg > MAX_ANGLE_DEG)  angle_deg = MAX_ANGLE_DEG;
    if (angle_deg < -MAX_ANGLE_DEG) angle_deg = -MAX_ANGLE_DEG;

    // Convert to 14-bit position (0–16383)
    int position = (int)(angle_deg * XQPOWER_SCALE + XQPOWER_CENTER);
    if (position < 0) position = 0;
    if (position > 16383) position = 16383;

    // 5-byte XQPOWER frame
    buf[0] = 0x80 | 0x08 | ((servoId >> 7) & 0x03); // syncId
    buf[1] = servoId & 0x7F;                          // id
    buf[2] = (position >> 7) & 0x7F;                  // posH
    buf[3] = position & 0x7F;                          // posL
    buf[4] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3];       // XOR checksum
}

// ===== USB Open/Close =====
// Initialize CP2102: set baud rate, 8N1, enable interface
static bool cp2102_configure(libusb_device_handle *h) {
    int r;

    // Enable interface
    r = libusb_control_transfer(h, 0x41, CP210X_IFC_ENABLE, 0x0001, 0, nullptr, 0, 1000);
    if (r < 0) {
        LOGE_S("CP2102 IFC_ENABLE failed: %s", libusb_error_name(r));
        return false;
    }

    // Set baud rate (115200)
    uint32_t baud = CP2102_BAUD;
    r = libusb_control_transfer(h, 0x41, CP210X_SET_BAUDRATE, 0, 0,
                                (unsigned char*)&baud, sizeof(baud), 1000);
    if (r < 0) {
        LOGE_S("CP2102 SET_BAUDRATE failed: %s", libusb_error_name(r));
        return false;
    }

    // Set line control (8N1)
    r = libusb_control_transfer(h, 0x41, CP210X_SET_LINE_CTL, CP210X_LINE_8N1, 0,
                                nullptr, 0, 1000);
    if (r < 0) {
        LOGE_S("CP2102 SET_LINE_CTL failed: %s", libusb_error_name(r));
        return false;
    }

    LOGI_S("CP2102 configured: %d baud, 8N1", CP2102_BAUD);
    return true;
}

static bool usb_open(int fd) {
    std::lock_guard<std::mutex> lock(s_usb_mutex);

    if (s_dev_handle) {
        libusb_close(s_dev_handle);
        s_dev_handle = nullptr;
    }

    if (!s_ctx) {
        int r = libusb_init(&s_ctx);
        if (r < 0) {
            LOGE_S("libusb_init failed: %s", libusb_error_name(r));
            return false;
        }
    }

    // Wrap Android file descriptor into libusb handle
    int r = libusb_wrap_sys_device(s_ctx, (intptr_t)fd, &s_dev_handle);
    if (r < 0 || !s_dev_handle) {
        LOGE_S("libusb_wrap_sys_device(fd=%d) failed: %s", fd, libusb_error_name(r));
        return false;
    }

    // Claim interface 0
    libusb_detach_kernel_driver(s_dev_handle, 0); // ignore error
    r = libusb_claim_interface(s_dev_handle, 0);
    if (r < 0) {
        LOGE_S("libusb_claim_interface failed: %s", libusb_error_name(r));
        libusb_close(s_dev_handle);
        s_dev_handle = nullptr;
        return false;
    }

    // Find bulk OUT endpoint
    libusb_device *dev = libusb_get_device(s_dev_handle);
    struct libusb_config_descriptor *config = nullptr;
    libusb_get_active_config_descriptor(dev, &config);
    s_bulk_endpoint_out = -1;

    if (config) {
        for (int i = 0; i < config->bNumInterfaces && s_bulk_endpoint_out < 0; i++) {
            const struct libusb_interface *iface = &config->interface[i];
            for (int j = 0; j < iface->num_altsetting && s_bulk_endpoint_out < 0; j++) {
                const struct libusb_interface_descriptor *alt = &iface->altsetting[j];
                for (int k = 0; k < alt->bNumEndpoints; k++) {
                    const struct libusb_endpoint_descriptor *ep = &alt->endpoint[k];
                    if ((ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK
                        && (ep->bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT) {
                        s_bulk_endpoint_out = ep->bEndpointAddress;
                        break;
                    }
                }
            }
        }
        libusb_free_config_descriptor(config);
    }

    if (s_bulk_endpoint_out < 0) {
        LOGE_S("No bulk OUT endpoint found");
        libusb_release_interface(s_dev_handle, 0);
        libusb_close(s_dev_handle);
        s_dev_handle = nullptr;
        return false;
    }

    // Configure CP2102
    if (!cp2102_configure(s_dev_handle)) {
        libusb_release_interface(s_dev_handle, 0);
        libusb_close(s_dev_handle);
        s_dev_handle = nullptr;
        return false;
    }

    LOGI_S("USB opened: fd=%d, bulk_out=0x%02X", fd, s_bulk_endpoint_out);
    return true;
}

static void usb_close() {
    std::lock_guard<std::mutex> lock(s_usb_mutex);
    if (s_dev_handle) {
        libusb_release_interface(s_dev_handle, 0);
        libusb_close(s_dev_handle);
        s_dev_handle = nullptr;
        LOGI_S("USB closed");
    }
    s_bulk_endpoint_out = -1;
}

// ===== Send servo frame =====
static bool usb_send(const uint8_t* data, int len) {
    std::lock_guard<std::mutex> lock(s_usb_mutex);
    if (!s_dev_handle || s_bulk_endpoint_out < 0) return false;

    int transferred = 0;
    int r = libusb_bulk_transfer(s_dev_handle, s_bulk_endpoint_out,
                                 (unsigned char*)data, len, &transferred, 50);
    if (r < 0) {
        LOGE_S("bulk_transfer failed: %s", libusb_error_name(r));
        return false;
    }
    return true;
}

// ===== Main Loop =====
// Priority system (same as XqpowerCan in folder 66):
//   Priority 1: actuator_servos      (from Control Allocator, armed)
//   Priority 2: actuator_outputs_sim (from RocketGNC direct, armed)
//   Priority 3: actuator_test        (ground test from QGC, any time)
static void servo_output_loop() {
    LOGI_S("Servo output thread started (multi-source, priority-based)");

    // Read XQCAN_LIMIT to match the XqpowerCan driver's scaling semantics.
    // This is the ±limit (in degrees) that a full-scale normalized command
    // (|actuator_servos.control| = 1.0) maps to.  It MUST match
    // SOLVER_DELTA_MAX_RAD baked into the acados solver (20° by default),
    // otherwise USB servos and CAN servos will deflect by different amounts
    // for the same MPC command.
    float scaling_limit_deg = DEFAULT_SCALING_LIMIT_DEG;
    {
        param_t p = param_find("XQCAN_LIMIT");
        if (p != PARAM_INVALID) {
            float v = 0.0f;
            if (param_get(p, &v) == 0 && v > 0.1f) {
                scaling_limit_deg = v;
            }
        }
        LOGI_S("Scaling limit: %.2f deg (from %s)",
               (double)scaling_limit_deg,
               (scaling_limit_deg == DEFAULT_SCALING_LIMIT_DEG) ? "default" : "XQCAN_LIMIT");
    }

    // Subscriptions — same as XqpowerCan in folder 66
    uORB::Subscription armed_sub{ORB_ID(actuator_armed)};             // Armed state check
    uORB::Subscription servos_sub{ORB_ID(actuator_servos)};           // Priority 1: Control Allocator
    uORB::Subscription sim_out_sub{ORB_ID(actuator_outputs_sim)};     // Priority 2: RocketGNC direct
    uORB::Subscription test_sub{ORB_ID(actuator_test)};               // Priority 3: Ground test

    bool usb_connected = false;
    bool was_armed = false;
    int prev_fd = -1;
    int diag_counter = 0;  // diagnostic counter

    while (s_servo_running.load()) {
        int current_fd = s_usb_fd.load();

        // Handle USB connect/disconnect
        if (current_fd != prev_fd) {
            if (current_fd >= 0) {
                usb_connected = usb_open(current_fd);
                if (usb_connected) {
                    LOGI_S("Servo USB connected (fd=%d)", current_fd);
                } else {
                    LOGE_S("Failed to open servo USB (fd=%d)", current_fd);
                }
            } else {
                if (usb_connected) {
                    usb_close();
                    usb_connected = false;
                    LOGI_S("Servo USB disconnected");
                }
            }
            prev_fd = current_fd;
        }

        // Diagnostic: print status every 1 second (100 cycles)
        diag_counter++;
        if (diag_counter % 100 == 0) {
            int fd_now = s_usb_fd.load();
            LOGI_S("DIAG: usb_connected=%d  fd=%d  prev_fd=%d", usb_connected, fd_now, prev_fd);
        }

        // لا نقرأ uORB إذا USB غير متصل — لتجنب استهلاك الرسائل
        // التي يحتاجها xqpower_can (race condition)
        if (!usb_connected) {
            usleep(100000); // 100ms — ننتظر حتى يتصل USB
            continue;
        }

        // Read armed state (same as XqpowerCan)
        actuator_armed_s armed{};
        armed_sub.update(&armed);
        bool is_armed = armed.armed;

        // Detect disarm → zero fins (same as XqpowerCan at 10 Hz)
        if (was_armed && !is_armed && usb_connected) {
            uint8_t frame[20];
            for (int i = 0; i < 4; i++) {
                encode_servo(&frame[i * 5], SERVO_IDS[i], 0.0f);
            }
            usb_send(frame, 20);
            LOGI_S("Disarmed — fins zeroed");
        }
        was_armed = is_armed;

        // Read from multiple sources with priority
        if (usb_connected) {
            float fin_angles[4] = {0, 0, 0, 0};
            bool has_data = false;

            // Priority 3: Ground test (actuator_test from QGC) — works anytime
            actuator_test_s test{};
            if (test_sub.update(&test)) {
                if (test.action == actuator_test_s::ACTION_DO_CONTROL &&
                    test.function >= 201 && test.function <= 204) {
                    int idx = test.function - 201;
                    fin_angles[idx] = test.value * scaling_limit_deg;
                    has_data = true;
                }
            }

            // Priority 1 & 2: only when armed (same as XqpowerCan)
            if (is_armed) {
                // Priority 1: actuator_servos (normalized [-1,+1]) — the only
                // source in both HITL and real flight since RocketMPC now always
                // publishes normalized output.  actuator_outputs_sim (radians) is
                // drained below to prevent stale queue growth but the value is
                // discarded, matching the unified XqpowerCan path.
                actuator_servos_s servos{};
                if (servos_sub.update(&servos)) {
                    for (int i = 0; i < 4; i++) {
                        fin_angles[i] = servos.control[i] * scaling_limit_deg;
                    }
                    has_data = true;
                }

                // Drain actuator_outputs_sim queue (RocketMPC still publishes
                // it for SimulatorMavlink lockstep; we no longer use it here).
                { actuator_outputs_s _discard{}; sim_out_sub.update(&_discard); }
            }

            // Diagnostic: print status every 1 second
            if (diag_counter % 100 == 0) {
                LOGI_S("DIAG: armed=%d has_data=%d fins=[%.1f %.1f %.1f %.1f]",
                       is_armed, has_data,
                       fin_angles[0], fin_angles[1], fin_angles[2], fin_angles[3]);
            }

            if (has_data) {
                uint8_t frame[20]; // 4 servos × 5 bytes
                for (int i = 0; i < 4; i++) {
                    encode_servo(&frame[i * 5], SERVO_IDS[i], fin_angles[i]);
                }

                if (!usb_send(frame, 20)) {
                    usb_close();
                    usb_connected = false;
                    s_usb_fd.store(-1);
                    prev_fd = -1;
                }
            }
        }

        usleep(10000); // 100 Hz
    }

    if (usb_connected) {
        // Send center position before stopping
        uint8_t frame[20];
        for (int i = 0; i < 4; i++) {
            encode_servo(&frame[i * 5], SERVO_IDS[i], 0.0f);
        }
        usb_send(frame, 20);
        usb_close();
    }

    if (s_ctx) {
        libusb_exit(s_ctx);
        s_ctx = nullptr;
    }

    LOGI_S("Servo output thread stopped");
}

// ===== Public API =====

void servo_usb_output_start() {
    if (s_servo_running.load()) return;
    s_servo_running.store(true);
    s_servo_thread = std::thread(servo_output_loop);
}

void servo_usb_output_stop() {
    s_servo_running.store(false);
    if (s_servo_thread.joinable()) s_servo_thread.join();
}

void servo_usb_set_fd(int fd) {
    LOGI_S("servo_usb_set_fd(%d)", fd);
    s_usb_fd.store(fd);
}
