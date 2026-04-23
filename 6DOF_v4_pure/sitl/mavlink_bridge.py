#!/usr/bin/env python3
"""
MAVLink SITL Bridge — Connects 6DOF Python simulation to PX4 via MAVLink HIL protocol.
جسر SITL — يربط محاكاة بايثون 6DOF بـ PX4 عبر بروتوكول MAVLink HIL.

Architecture:
  Python 6DOF (physics) ←→ TCP:4560 (MAVLink) ←→ PX4 SimulatorMavlink (control)

The bridge:
  1. Runs the 6DOF physics engine with control_type: "none"
  2. Sends simulated sensor data (HIL_SENSOR, HIL_GPS, HIL_STATE_QUATERNION) to PX4
  3. Receives fin commands (HIL_ACTUATOR_CONTROLS) from PX4's rocket_mpc module
  4. Applies fin commands to the 6DOF simulation actuator model
  5. Supports lockstep and real-time timing modes
"""

import sys
import os
import time
import struct
import socket
import logging
import argparse
import threading
import queue
import csv
from pathlib import Path
from typing import Optional, Dict, Any, Tuple

import numpy as np
import yaml

# Add parent directory to path for imports
_SCRIPT_DIR = Path(__file__).resolve().parent
_SIM_DIR = _SCRIPT_DIR.parent
sys.path.insert(0, str(_SIM_DIR))

from rocket_6dof_sim import Rocket6DOFSimulation
from dynamics.quaternion_utils import quaternion_to_euler

logger = logging.getLogger(__name__)

# ============================================================================
# MAVLink message constants (minimal subset — no pymavlink dependency)
# ============================================================================

# MAVLink v2 header
MAVLINK_STX_V2 = 0xFD
MAVLINK_HEADER_LEN_V2 = 10
MAVLINK_CHECKSUM_LEN = 2

# Message IDs
MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_HIL_SENSOR = 107
MAVLINK_MSG_ID_HIL_GPS = 113
MAVLINK_MSG_ID_HIL_STATE_QUATERNION = 115
MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS = 93

# CRC extra bytes (from MAVLink XML definitions)
CRC_EXTRA_HEARTBEAT = 50
CRC_EXTRA_COMMAND_LONG = 152
CRC_EXTRA_HIL_SENSOR = 108
CRC_EXTRA_HIL_GPS = 124
CRC_EXTRA_HIL_STATE_QUATERNION = 4
CRC_EXTRA_HIL_ACTUATOR_CONTROLS = 47

# Payload sizes
HIL_SENSOR_LEN = 65          # Updated for MAVLink v2 with id field
HIL_GPS_LEN = 39             # lat,lon,alt + velocities + fix info
HIL_STATE_QUATERNION_LEN = 64
HIL_ACTUATOR_CONTROLS_LEN = 41  # 8 floats + mode + flags + time

# MAVLink system/component IDs
SYS_ID_SIM = 1
COMP_ID_SIM = 1
SYS_ID_GCS = 255
COMP_ID_GCS = 190

# MAVLink command IDs
MAV_CMD_COMPONENT_ARM_DISARM = 400


def _x25_crc(data: bytes, crc: int = 0xFFFF) -> int:
    """Compute MAVLink X.25 CRC."""
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF
    return crc


def _pack_mavlink_v2(msg_id: int, payload: bytes, crc_extra: int,
                     sys_id: int = SYS_ID_SIM, comp_id: int = COMP_ID_SIM,
                     seq: list = [0]) -> bytes:
    """Pack a MAVLink v2 message with header, payload, and CRC."""
    payload_len = len(payload)
    # Trim trailing zeros from payload (MAVLink v2 truncation)
    trimmed = payload.rstrip(b'\x00')
    trimmed_len = len(trimmed)
    
    seq_num = seq[0] & 0xFF
    seq[0] = (seq[0] + 1) & 0xFF

    # Header: STX, len, incompat_flags, compat_flags, seq, sys_id, comp_id, msg_id(3 bytes)
    header = struct.pack('<BBBBBBBHB',
                         MAVLINK_STX_V2,
                         trimmed_len,
                         0,  # incompat_flags
                         0,  # compat_flags
                         seq_num,
                         sys_id,
                         comp_id,
                         msg_id & 0xFFFF,
                         (msg_id >> 16) & 0xFF)

    # CRC over header[1:] + trimmed payload + crc_extra
    crc_data = header[1:] + trimmed
    crc = _x25_crc(crc_data)
    crc = _x25_crc(bytes([crc_extra]), crc)

    return header + trimmed + struct.pack('<H', crc)


# ============================================================================
# HIL Message Builders
# ============================================================================

def build_heartbeat() -> bytes:
    """Build HEARTBEAT message (ID 0) as GCS."""
    # type=6 (GCS), autopilot=8 (invalid), base_mode=0, custom_mode=0,
    # system_status=4 (active), mavlink_version=3
    payload = struct.pack('<IBBBBB',
                          0,   # custom_mode
                          6,   # type = MAV_TYPE_GCS
                          8,   # autopilot = MAV_AUTOPILOT_INVALID
                          0,   # base_mode
                          4,   # system_status = MAV_STATE_ACTIVE
                          3)   # mavlink_version
    return _pack_mavlink_v2(MAVLINK_MSG_ID_HEARTBEAT, payload,
                            CRC_EXTRA_HEARTBEAT,
                            sys_id=SYS_ID_GCS, comp_id=COMP_ID_GCS)


def build_command_long(command: int, target_sys: int = 1, target_comp: int = 1,
                       confirmation: int = 0,
                       p1=0.0, p2=0.0, p3=0.0, p4=0.0,
                       p5=0.0, p6=0.0, p7=0.0) -> bytes:
    """Build COMMAND_LONG message (ID 76)."""
    payload = struct.pack('<7fHBBB',
                          p1, p2, p3, p4, p5, p6, p7,
                          command,
                          target_sys,
                          target_comp,
                          confirmation)
    return _pack_mavlink_v2(MAVLINK_MSG_ID_COMMAND_LONG, payload,
                            CRC_EXTRA_COMMAND_LONG,
                            sys_id=SYS_ID_GCS, comp_id=COMP_ID_GCS)


def build_hil_sensor(time_usec: int,
                     accel_body: np.ndarray,    # [ax,ay,az] m/s² body frame
                     gyro_body: np.ndarray,     # [gx,gy,gz] rad/s body frame
                     mag_body: np.ndarray,      # [mx,my,mz] Gauss body frame
                     abs_pressure: float,       # mbar
                     diff_pressure: float,      # mbar
                     pressure_alt: float,       # m
                     temperature: float = 25.0, # °C
                     fields_updated: int = 0x1FFF,
                     sensor_id: int = 0) -> bytes:
    """Build HIL_SENSOR MAVLink message (ID 107)."""
    # fmt: time_usec(Q) + 3xaccel(fff) + 3xgyro(fff) + 3xmag(fff) +
    #      abs_pressure(f) + diff_pressure(f) + pressure_alt(f) + temperature(f) +
    #      fields_updated(I) + id(B)
    payload = struct.pack('<Q3f3f3fffffIB',
                          time_usec,
                          accel_body[0], accel_body[1], accel_body[2],
                          gyro_body[0], gyro_body[1], gyro_body[2],
                          mag_body[0], mag_body[1], mag_body[2],
                          abs_pressure, diff_pressure, pressure_alt,
                          temperature,
                          fields_updated,
                          sensor_id)
    return _pack_mavlink_v2(MAVLINK_MSG_ID_HIL_SENSOR, payload, CRC_EXTRA_HIL_SENSOR)


def build_hil_gps(time_usec: int,
                  lat_deg: float, lon_deg: float, alt_msl_m: float,
                  vn: float, ve: float, vd: float,
                  fix_type: int = 3,
                  eph_cm: int = 250, epv_cm: int = 400,
                  satellites: int = 12) -> bytes:
    """Build HIL_GPS MAVLink message (ID 113)."""
    lat_e7 = int(lat_deg * 1e7)
    lon_e7 = int(lon_deg * 1e7)
    alt_mm = int(alt_msl_m * 1000)
    vel_cm_s = int(np.sqrt(vn**2 + ve**2) * 100)  # ground speed
    vn_cm = int(vn * 100)
    ve_cm = int(ve * 100)
    vd_cm = int(vd * 100)
    # Course over ground in centidegrees
    cog = int(np.degrees(np.arctan2(ve, vn)) * 100) % 36000

    payload = struct.pack('<QBiiiHHHhhhHB',
                          time_usec,
                          fix_type,
                          lat_e7, lon_e7, alt_mm,
                          eph_cm, epv_cm,
                          vel_cm_s,
                          vn_cm, ve_cm, vd_cm,
                          cog,
                          satellites)
    return _pack_mavlink_v2(MAVLINK_MSG_ID_HIL_GPS, payload, CRC_EXTRA_HIL_GPS)


def build_hil_state_quaternion(time_usec: int,
                                quat: np.ndarray,      # [w,x,y,z]
                                omega: np.ndarray,      # [p,q,r] rad/s
                                lat_deg: float, lon_deg: float, alt_msl_m: float,
                                vn: float, ve: float, vd: float,
                                accel_body: np.ndarray,  # [ax,ay,az] m/s²
                                airspeed: float = 0.0) -> bytes:
    """Build HIL_STATE_QUATERNION MAVLink message (ID 115)."""
    lat_e7 = int(lat_deg * 1e7)
    lon_e7 = int(lon_deg * 1e7)
    alt_mm = int(alt_msl_m * 1000)
    vx_cm = int(vn * 100)
    vy_cm = int(ve * 100)
    vz_cm = int(vd * 100)
    # Accel in milli-g
    ax_mg = int(accel_body[0] / 9.80665 * 1000)
    ay_mg = int(accel_body[1] / 9.80665 * 1000)
    az_mg = int(accel_body[2] / 9.80665 * 1000)
    ias_cm = int(airspeed * 100)

    payload = struct.pack('<Q4f3fiiihhhHHhhh',
                          time_usec,
                          quat[0], quat[1], quat[2], quat[3],
                          omega[0], omega[1], omega[2],
                          lat_e7, lon_e7, alt_mm,
                          vx_cm, vy_cm, vz_cm,
                          ias_cm,             # ind_airspeed (uint16)
                          0,                  # true_airspeed (uint16)
                          ax_mg, ay_mg, az_mg)
    return _pack_mavlink_v2(MAVLINK_MSG_ID_HIL_STATE_QUATERNION, payload,
                            CRC_EXTRA_HIL_STATE_QUATERNION)


def parse_hil_actuator_controls(payload: bytes) -> Optional[Dict]:
    """Parse HIL_ACTUATOR_CONTROLS message payload.
    Returns dict with 'time_usec', 'controls' (16 floats), 'mode', 'flags'.

    MAVLink v2 wire order (fields sorted by size, largest first):
      time_usec  uint64  (8 bytes)
      flags      uint64  (8 bytes)
      controls   float[16] (64 bytes)
      mode       uint8   (1 byte)
    Total = 81 bytes
    """
    if len(payload) < 41:
        return None
    try:
        # Full parse: QQ16fB = 8 + 8 + 64 + 1 = 81
        if len(payload) >= 81:
            values = struct.unpack('<QQ16fB', payload[:81])
            return {
                'time_usec': values[0],
                'flags': values[1],
                'controls': list(values[2:18]),
                'mode': values[18],
            }
        # Trimmed version (MAVLink v2 truncates trailing zeros)
        # Minimum: time_usec + flags + at least 4 controls = 8 + 8 + 16 = 32
        elif len(payload) >= 32:
            n_floats = (len(payload) - 16) // 4  # after time_usec + flags
            fmt = f'<QQ{min(n_floats, 16)}f'
            sz = struct.calcsize(fmt)
            values = struct.unpack(fmt, payload[:sz])
            controls = list(values[2:]) + [0.0] * (16 - min(n_floats, 16))
            remaining = payload[sz:]
            mode = remaining[0] if len(remaining) > 0 else 0
            return {
                'time_usec': values[0],
                'flags': values[1],
                'controls': controls,
                'mode': mode,
            }
    except struct.error:
        pass
    return None


# ============================================================================
# MAVLink Parser (minimal v2 parser)
# ============================================================================

class MavlinkParser:
    """Minimal MAVLink v2 parser for receiving HIL_ACTUATOR_CONTROLS."""

    def __init__(self):
        self._buf = bytearray()

    def parse(self, data: bytes) -> list:
        """Feed data, return list of (msg_id, payload) tuples."""
        self._buf.extend(data)
        messages = []

        while True:
            # Find STX
            idx = self._buf.find(bytes([MAVLINK_STX_V2]))
            if idx < 0:
                self._buf.clear()
                break
            if idx > 0:
                del self._buf[:idx]

            # Need at least header
            if len(self._buf) < MAVLINK_HEADER_LEN_V2:
                break

            payload_len = self._buf[1]
            total_len = MAVLINK_HEADER_LEN_V2 + payload_len + MAVLINK_CHECKSUM_LEN

            if len(self._buf) < total_len:
                break

            # Extract message ID (3 bytes, little-endian at offsets 7,8,9)
            msg_id = self._buf[7] | (self._buf[8] << 8) | (self._buf[9] << 16)

            # Extract payload
            payload = bytes(self._buf[MAVLINK_HEADER_LEN_V2:
                                       MAVLINK_HEADER_LEN_V2 + payload_len])

            # We skip CRC validation for simplicity — PX4 is trusted on localhost
            messages.append((msg_id, payload))
            del self._buf[:total_len]

        return messages


# ============================================================================
# Noise Generator
# ============================================================================

class SensorNoise:
    """Add realistic sensor noise to simulation outputs."""

    def __init__(self, config: dict, rng_seed: int = 42):
        self.rng = np.random.default_rng(rng_seed)
        noise_cfg = config.get('noise', {})
        self.accel_std = noise_cfg.get('accel_std', 0.35)
        self.gyro_std = noise_cfg.get('gyro_std', 0.005)
        self.gps_pos_std = noise_cfg.get('gps_pos_std', 2.5)
        self.gps_vel_std = noise_cfg.get('gps_vel_std', 0.1)
        self.gps_delay_ms = noise_cfg.get('gps_delay_ms', 100)
        self.mag_std = noise_cfg.get('mag_std', 0.005)
        self.baro_std = noise_cfg.get('baro_std', 0.5)

        # Multipliers for high-noise robustness testing
        if noise_cfg.get('high_noise_enabled', False):
            self.accel_std *= noise_cfg.get('high_noise_accel_multiplier', 3.0)
            self.gyro_std *= noise_cfg.get('high_noise_gyro_multiplier', 3.0)

        # Fixed IMU biases (constant per run)
        accel_bias_std = noise_cfg.get('accel_bias_std', 0.05)
        gyro_bias_std = noise_cfg.get('gyro_bias_std', 0.001)
        self.accel_bias = self.rng.normal(0, accel_bias_std, 3)  # m/s²
        self.gyro_bias = self.rng.normal(0, gyro_bias_std, 3)   # rad/s

        # GPS delay buffer
        self._gps_buffer = []

    def add_imu_noise(self, accel: np.ndarray, gyro: np.ndarray
                      ) -> Tuple[np.ndarray, np.ndarray]:
        """Add noise + bias to IMU readings."""
        noisy_accel = accel + self.accel_bias + self.rng.normal(0, self.accel_std, 3)
        noisy_gyro = gyro + self.gyro_bias + self.rng.normal(0, self.gyro_std, 3)
        return noisy_accel, noisy_gyro

    def add_gps_noise(self, lat: float, lon: float, alt: float,
                      vn: float, ve: float, vd: float, t_usec: int
                      ) -> Optional[Tuple[float, float, float, float, float, float]]:
        """Add noise + delay to GPS. Returns None if delayed data not ready."""
        # Store with timestamp
        noisy_lat = lat + self.rng.normal(0, self.gps_pos_std / 111320.0)
        noisy_lon = lon + self.rng.normal(0, self.gps_pos_std / (111320.0 * max(np.cos(np.radians(lat)), 0.01)))
        noisy_alt = alt + self.rng.normal(0, self.gps_pos_std)
        noisy_vn = vn + self.rng.normal(0, self.gps_vel_std)
        noisy_ve = ve + self.rng.normal(0, self.gps_vel_std)
        noisy_vd = vd + self.rng.normal(0, self.gps_vel_std)

        self._gps_buffer.append((t_usec, noisy_lat, noisy_lon, noisy_alt,
                                  noisy_vn, noisy_ve, noisy_vd))

        delay_usec = self.gps_delay_ms * 1000
        # Return oldest entry past delay
        while len(self._gps_buffer) > 1:
            if t_usec - self._gps_buffer[0][0] >= delay_usec:
                _, lat_d, lon_d, alt_d, vn_d, ve_d, vd_d = self._gps_buffer.pop(0)
                return lat_d, lon_d, alt_d, vn_d, ve_d, vd_d
            break

        if len(self._gps_buffer) == 1 and delay_usec == 0:
            _, lat_d, lon_d, alt_d, vn_d, ve_d, vd_d = self._gps_buffer.pop(0)
            return lat_d, lon_d, alt_d, vn_d, ve_d, vd_d

        return None

    def add_mag_noise(self, mag: np.ndarray) -> np.ndarray:
        return mag + self.rng.normal(0, self.mag_std, 3)

    def add_baro_noise(self, alt: float) -> float:
        return alt + self.rng.normal(0, self.baro_std)


# ============================================================================
# Coordinate Transforms
# ============================================================================

def _ned_to_lla(pos_ned: np.ndarray, launch_lat_deg: float,
                launch_lon_deg: float, launch_alt_m: float
                ) -> Tuple[float, float, float]:
    """Convert NED position (relative to launch) to LLA.
    Simple flat-earth approximation (valid for <50 km range)."""
    lat = launch_lat_deg + np.degrees(pos_ned[0] / 6371000.0)
    lon = launch_lon_deg + np.degrees(pos_ned[1] / (6371000.0 * np.cos(np.radians(launch_lat_deg))))
    alt = launch_alt_m - pos_ned[2]  # NED: z down → alt up
    return lat, lon, alt


def _compute_body_accel(forces_body: np.ndarray, mass: float,
                        gravity_ned: np.ndarray, quat: np.ndarray) -> np.ndarray:
    """Compute specific force in body frame (what IMU accelerometer measures).
    accel_imu = (F_total / m) - g_body = specific force"""
    # Rotate gravity to body frame
    # quat = [q0, q1, q2, q3] body→NED
    q0, q1, q2, q3 = quat
    # DCM body→NED (transpose = NED→body)
    C = np.array([
        [1 - 2*(q2*q2 + q3*q3), 2*(q1*q2 + q0*q3), 2*(q1*q3 - q0*q2)],
        [2*(q1*q2 - q0*q3), 1 - 2*(q1*q1 + q3*q3), 2*(q2*q3 + q0*q1)],
        [2*(q1*q3 + q0*q2), 2*(q2*q3 - q0*q1), 1 - 2*(q1*q1 + q2*q2)],
    ])
    g_body = C @ gravity_ned  # gravity in body frame
    # Specific force = total_accel - gravity (what accelerometer reads)
    return forces_body / max(mass, 0.1) - g_body


def _compute_mag_field(lat_deg: float, _lon_deg: float, quat: np.ndarray) -> np.ndarray:
    """Approximate Earth magnetic field in body frame (Gauss).
    Simplified dipole model for Yemen region (~16°N)."""
    # Magnetic field in NED (approximate for 16°N latitude)
    # Total field ~40000 nT = 0.4 Gauss, inclination ~25° (pointing down)
    inclination = np.radians(25.0 + 0.5 * (lat_deg - 16.0))
    B_h = 0.32 * np.cos(inclination)  # horizontal component
    B_ned = np.array([B_h, 0.0, 0.32 * np.sin(inclination)])

    # Rotate to body frame
    q0, q1, q2, q3 = quat
    C = np.array([
        [1 - 2*(q2*q2 + q3*q3), 2*(q1*q2 + q0*q3), 2*(q1*q3 - q0*q2)],
        [2*(q1*q2 - q0*q3), 1 - 2*(q1*q1 + q3*q3), 2*(q2*q3 + q0*q1)],
        [2*(q1*q3 + q0*q2), 2*(q2*q3 - q0*q1), 1 - 2*(q1*q1 + q2*q2)],
    ])
    return C @ B_ned


# ============================================================================
# SITL Bridge
# ============================================================================

class SITLBridge:
    """MAVLink HIL bridge between 6DOF sim and PX4."""

    def __init__(self, sitl_config_path: str, sim_config_path: Optional[str] = None):
        # Load simulation config — SINGLE SOURCE OF TRUTH
        sim_cfg_path = sim_config_path or str(_SIM_DIR / 'config' / '6dof_config_advanced.yaml')
        with open(sim_cfg_path, 'r', encoding='utf-8') as f:
            sim_config = yaml.safe_load(f)

        # Bridge settings from sim config (bridge section)
        bridge_cfg = sim_config.get('bridge', {})

        # Legacy fallback: if sim config has no bridge section, try sitl_config.yaml
        if not bridge_cfg and os.path.isfile(sitl_config_path):
            with open(sitl_config_path, 'r', encoding='utf-8') as f:
                legacy_cfg = yaml.safe_load(f)
            bridge_cfg = legacy_cfg.get('bridge', {})
            noise_cfg = legacy_cfg.get('noise', {})
            logger.warning("Bridge settings not in sim config — using legacy sitl_config.yaml")
        else:
            noise_cfg = bridge_cfg.get('noise', {})

        self.host = bridge_cfg.get('host', '127.0.0.1')
        self.port = bridge_cfg.get('port', 4560)
        self.protocol = bridge_cfg.get('protocol', 'tcp')
        self.timing = bridge_cfg.get('timing', 'lockstep')
        self.sensor_rate = bridge_cfg.get('sensor_rate_hz', 250)
        self.gps_rate = bridge_cfg.get('gps_rate_hz', 10)
        self.state_rate = bridge_cfg.get('state_rate_hz', 50)
        self.lockstep_timeout_ms = bridge_cfg.get('lockstep_timeout_ms', 5000)

        self.launch_lat = sim_config.get('launch', {}).get('latitude', 16.457472)
        self.launch_lon = sim_config.get('launch', {}).get('longitude', 44.115361)
        self.launch_alt = sim_config.get('launch', {}).get('altitude', 1200.0)
        self.sim_dt = sim_config.get('simulation', {}).get('dt', 0.01)
        self.sim_duration = sim_config.get('simulation', {}).get('duration', 500.0)

        # Write temp config with control_type: "none" and estimation off
        import tempfile
        sim_config['simulation']['control_type'] = 'none'
        sim_config.setdefault('estimation', {})['mode'] = 'off'
        # Zero initial angular velocity to match baseline behaviour
        # (baseline MPC mode resets spin rate on init; bridge must do same)
        sim_config.setdefault('initial_conditions', {})['angular_velocity'] = [0.0, 0.0, 0.0]
        self._tmp_cfg = tempfile.NamedTemporaryFile(
            mode='w', suffix='.yaml', delete=False, encoding='utf-8')
        yaml.dump(sim_config, self._tmp_cfg, allow_unicode=True,
                  default_flow_style=False)
        self._tmp_cfg.close()

        long_range = sim_config.get('long_range', {}).get('enabled', False)
        self.sim = Rocket6DOFSimulation(
            config_file=self._tmp_cfg.name, long_range_mode=long_range)

        # Noise generator — reads from bridge.noise in sim config
        self.noise = SensorNoise({'noise': noise_cfg})

        # Network
        self._sock: Optional[socket.socket] = None
        self._parser = MavlinkParser()

        # State
        self._sim_time_usec = 0
        self._last_controls = np.zeros(16)
        self._running = False
        self._fin_commands_rad = np.zeros(4)
        # PX4 status tracking (from HIL_ACTUATOR_CONTROLS)
        self._last_flags = 0   # bit-0 = ARMED
        self._last_mode = 0
        self._last_step_dt_ms = 0.0  # wall-clock time per sim step

        # History for CSV export
        self.history = {
            'time': [], 'position': [], 'velocity': [],
            'attitude': [], 'angular_velocity': [], 'mass': [],
            'fin_commands': [], 'fin_actual': [],
            'forces': [], 'altitude': [], 'ground_range': [],
            'alpha': [], 'beta': [], 'mach': [],
            'accel_body': [], 'gyro_body': [],
            'lat': [], 'lon': [], 'alt_msl': [],
            # PX4 status per step
            'px4_armed': [], 'px4_mode': [], 'step_dt_ms': [],
        }

    def __del__(self):
        try:
            os.unlink(self._tmp_cfg.name)
        except (OSError, AttributeError):
            pass

    # ------------------------------------------------------------------
    # Network
    # ------------------------------------------------------------------

    def _connect(self):
        """Listen for PX4 connection (bridge is SERVER, PX4 is CLIENT)."""
        if self.protocol == 'tcp':
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            server.bind((self.host, self.port))
            server.listen(1)
            server.settimeout(60.0)
            logger.info("Waiting for PX4 on TCP %s:%d ...", self.host, self.port)
            print(f"[BRIDGE] Listening on TCP {self.host}:{self.port} — start PX4 now...")
            self._sock, addr = server.accept()
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._sock.setblocking(True)
            server.close()
            logger.info("PX4 connected from %s", addr)
            print(f"[BRIDGE] PX4 connected from {addr}")
        else:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.bind((self.host, self.port))
            self._sock.settimeout(60.0)
            logger.info("Waiting for PX4 on UDP %s:%d ...", self.host, self.port)
            print(f"[BRIDGE] Listening on UDP {self.host}:{self.port}")

    def _send(self, data: bytes):
        """Send data to PX4."""
        if self._sock:
            try:
                self._sock.sendall(data)
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                logger.error("PX4 disconnected on send: %s (step t_sim=%.3f)",
                             e, self._sim_time_usec / 1e6)
                print(f"[BRIDGE] PX4 disconnected on send: {e} "
                      f"(t_sim={self._sim_time_usec/1e6:.3f}s)")
                self._running = False

    def _recv(self, timeout_sec: float = 5.0) -> list:
        """Receive and parse MAVLink messages from PX4."""
        self._sock.settimeout(timeout_sec)
        try:
            data = self._sock.recv(4096)
            if not data:
                # Empty bytes = TCP peer closed connection (EOF)
                logger.error("PX4 closed TCP connection (recv returned empty)")
                self._running = False
                return []
            return self._parser.parse(data)
        except socket.timeout:
            return []
        except (ConnectionResetError, BrokenPipeError, OSError):
            self._running = False
            return []

    # ------------------------------------------------------------------
    # Sensor data generation
    # ------------------------------------------------------------------

    def _build_sensors(self, snapshot: dict, state: np.ndarray,
                       t: float, t_usec: int) -> dict:
        """Build all HIL messages from simulation state."""
        pos = state[0:3]
        vel = state[3:6]
        quat = state[6:10]
        omega = state[10:13]
        mass = state[13] if len(state) > 13 else 30.0

        # Get forces from snapshot
        forces_body = np.array(snapshot.get('forces', [0, 0, 0]))
        g_ned = np.array([0, 0, 9.80665])

        # Body accelerations (what IMU measures = specific force)
        accel_body = _compute_body_accel(forces_body, mass, g_ned, quat)

        # Position as LLA
        if 'position_lla' in snapshot and snapshot['position_lla'] is not None:
            lla = np.asarray(snapshot['position_lla'])
            lat, lon, alt = np.degrees(lla[0]), np.degrees(lla[1]), lla[2]
        else:
            lat, lon, alt = _ned_to_lla(pos, self.launch_lat, self.launch_lon,
                                        self.launch_alt)

        # Velocity NED
        vel_ned = np.array(snapshot.get('vel_ned', vel))

        # Magnetic field in body frame
        mag_body = _compute_mag_field(lat, lon, quat)

        # Barometric altitude
        altitude_m = alt

        # Atmospheric pressure (simple ISA model)
        baro_pressure = 1013.25 * (1.0 - 2.25577e-5 * altitude_m) ** 5.25588

        # Add noise
        noisy_accel, noisy_gyro = self.noise.add_imu_noise(accel_body, omega)
        noisy_mag = self.noise.add_mag_noise(mag_body)
        noisy_alt = self.noise.add_baro_noise(altitude_m)
        noisy_baro = 1013.25 * (1.0 - 2.25577e-5 * noisy_alt) ** 5.25588

        # Airspeed differential pressure: 0.5 * rho * V²  → mbar
        airspeed = np.linalg.norm(vel)
        rho = 1.225 * (1.0 - 2.25577e-5 * altitude_m) ** 4.25588
        diff_press = 0.5 * rho * airspeed**2 / 100.0  # Pa → mbar

        # GPS (with delay)
        gps_data = self.noise.add_gps_noise(lat, lon, alt,
                                             vel_ned[0], vel_ned[1], vel_ned[2],
                                             t_usec)

        return {
            'accel_body': noisy_accel,
            'gyro_body': noisy_gyro,
            'mag_body': noisy_mag,
            'baro_pressure': noisy_baro,
            'diff_pressure': diff_press,
            'pressure_alt': noisy_alt,
            'lat': lat, 'lon': lon, 'alt': alt,
            'vel_ned': vel_ned,
            'quat': quat,
            'omega': omega,
            'airspeed': airspeed,
            'gps': gps_data,  # None if delayed
            'accel_body_true': accel_body,
        }

    # ------------------------------------------------------------------
    # Main simulation loop
    # ------------------------------------------------------------------

    def run(self, csv_output: Optional[str] = None) -> dict:
        """Run the SITL bridge simulation.

        Args:
            csv_output: Path to save flight data CSV

        Returns:
            dict: Flight history
        """
        self._connect()
        self._running = True

        sim = self.sim
        dt = self.sim_dt

        # Compute send intervals in simulation steps
        sensor_interval = max(1, int(round(1.0 / (self.sensor_rate * dt))))
        gps_interval = max(1, int(round(1.0 / (self.gps_rate * dt))))
        state_interval = max(1, int(round(1.0 / (self.state_rate * dt))))

        # Initialize simulation state
        state = sim._initialize_state()
        sim._update_last_valid_quaternion(state[6:10])

        logger.info("Starting SITL bridge: dt=%.4f, timing=%s, duration=%.1f",
                     dt, self.timing, self.sim_duration)
        print(f"[BRIDGE] Starting simulation: dt={dt}, timing={self.timing}, "
              f"duration={self.sim_duration}s")

        n_steps = int(self.sim_duration / dt)
        t = 0.0
        step = 0

        # Control function: apply fin commands from PX4
        def bridge_control(state_dict, t_sim):
            return self._fin_commands_rad.copy()

        # PX4 lockstep initialization: send HIL_SENSOR at wall-clock pace
        # so PX4's rcS script, EKF2, commander, rocket_mpc can all boot.
        # In lockstep, PX4 time ONLY advances via HIL_SENSOR timestamps.
        # We must add small wall-clock delays between messages to let PX4
        # process each step (its internal poll loop needs time to run).
        logger.info("PX4 lockstep warm-up — advancing clock for boot...")
        print("[BRIDGE] PX4 lockstep warm-up — paced clock for boot...")

        init_dt_us = int(dt * 1e6)           # 10 000 µs per tick
        init_gps_every = max(1, int(round(1.0 / (self.gps_rate * dt))))
        got_controls = False
        px4_bytes_received = 0

        initial_snapshot = {'forces': [0, 0, 0], 'vel_ned': [0, 0, 0]}
        sensors = self._build_sensors(initial_snapshot, state, 0.0, 0)

        # Send HIL_SENSOR at ~200Hz wall-clock (5ms sleep between messages)
        # 30s sim-time = 3000 steps at dt=0.01
        boot_steps = int(30.0 / dt)
        self._sock.setblocking(False)

        # Send initial HEARTBEAT so PX4 commander recognizes a GCS
        self._send(build_heartbeat())

        heartbeat_interval = 100  # send HEARTBEAT every 100 steps (~1s sim-time)
        init_state_every = max(1, int(round(1.0 / (self.state_rate * dt))))

        # Initial groundtruth state: rocket on pad, identity quaternion, zero velocity
        init_quat = np.array([1.0, 0.0, 0.0, 0.0])
        init_omega = np.zeros(3)
        init_accel = np.array([0.0, 0.0, -9.80665])

        for init_i in range(boot_steps):
            self._sim_time_usec = (init_i + 1) * init_dt_us

            # Periodic HEARTBEAT (every ~1s sim-time)
            if init_i % heartbeat_interval == 0:
                self._send(build_heartbeat())

            # Send HIL_STATE_QUATERNION BEFORE HIL_SENSOR so groundtruth
            # topics are published before lockstep time advances and
            # other modules (rocket_mpc) run.
            if init_i % init_state_every == 0:
                self._send(build_hil_state_quaternion(
                    self._sim_time_usec,
                    init_quat, init_omega,
                    self.launch_lat, self.launch_lon, self.launch_alt,
                    0.0, 0.0, 0.0,
                    init_accel, 0.0))

            if init_i % init_gps_every == 0:
                # Send GPS directly (bypass delay buffer during warm-up)
                self._send(build_hil_gps(
                    self._sim_time_usec,
                    self.launch_lat, self.launch_lon, self.launch_alt,
                    0.0, 0.0, 0.0))

            # HIL_SENSOR LAST — triggers lockstep time advance in PX4
            self._send(build_hil_sensor(
                self._sim_time_usec,
                sensors['accel_body'], sensors['gyro_body'],
                sensors['mag_body'], sensors['baro_pressure'],
                sensors['diff_pressure'], sensors['pressure_alt']))

            # Wall-clock delay to let PX4 process
            time.sleep(0.005)

            # Non-blocking check for actuator controls
            try:
                raw = self._sock.recv(4096)
                if raw:
                    px4_bytes_received += len(raw)
                    msgs = self._parser.parse(raw)
                    for msg_id, payload in msgs:
                        if msg_id == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
                            parsed = parse_hil_actuator_controls(payload)
                            if parsed:
                                self._last_controls = np.array(parsed['controls'])
                                if not got_controls:
                                    got_controls = True
                                    print(f"[BRIDGE] PX4 controls at init step {init_i} "
                                          f"(t_sim={self._sim_time_usec/1e6:.2f}s)")
            except (BlockingIOError, socket.timeout):
                pass

            if init_i % 500 == 0:
                print(f"[BRIDGE] Boot: {init_i}/{boot_steps} "
                      f"(t_sim={self._sim_time_usec/1e6:.1f}s) "
                      f"rx={px4_bytes_received}B "
                      f"{'CONTROLS OK' if got_controls else 'booting...'}")

            # Once we have controls and enough boot time (≥10s sim), stop early
            if got_controls and init_i > 1000:
                print(f"[BRIDGE] Boot complete at step {init_i} "
                      f"(t_sim={self._sim_time_usec/1e6:.1f}s)")
                break

        # Restore blocking mode for main simulation
        self._sock.setblocking(True)

        if not got_controls:
            logger.warning("No PX4 controls after warm-up — proceeding")
            print(f"[BRIDGE] WARNING: No PX4 controls after warm-up "
                  f"(rx={px4_bytes_received}B)")
        else:
            print("[BRIDGE] PX4 fully initialized with actuator controls")

        # ---- ARM the vehicle ----
        # Send ARM command via PX4's GCS MAVLink UDP port (18570).
        # The GCS port has no preconfigured -o remote, so PX4 responds to
        # the sender's address.  We also try the offboard port (14580).
        # simulator_mavlink (TCP 4560) only handles HIL_* messages.
        print("[BRIDGE] Sending ARM command via MAVLink UDP...")
        arm_ack_received = False
        arm_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        arm_sock.settimeout(0.2)
        # Try GCS port first, then offboard port
        arm_ports = [18570, 14580]
        for arm_attempt in range(50):
            port = arm_ports[arm_attempt % len(arm_ports)]
            arm_addr = ('127.0.0.1', port)
            # Send HEARTBEAT + ARM on UDP
            arm_sock.sendto(build_heartbeat(), arm_addr)
            arm_sock.sendto(build_command_long(
                MAV_CMD_COMPONENT_ARM_DISARM,
                p1=1.0,       # 1 = arm
                p2=21196.0,   # force-arm magic value
            ), arm_addr)
            # Advance sim clock on TCP so PX4 processes
            self._sim_time_usec += init_dt_us
            # Groundtruth + GPS BEFORE HIL_SENSOR (lockstep ordering)
            if arm_attempt % max(1, init_state_every // 10) == 0:
                self._send(build_hil_state_quaternion(
                    self._sim_time_usec,
                    init_quat, init_omega,
                    self.launch_lat, self.launch_lon, self.launch_alt,
                    0.0, 0.0, 0.0,
                    init_accel, 0.0))
            if arm_attempt % init_gps_every == 0:
                self._send(build_hil_gps(
                    self._sim_time_usec,
                    self.launch_lat, self.launch_lon, self.launch_alt,
                    0.0, 0.0, 0.0))
            # HIL_SENSOR LAST — triggers lockstep time advance
            self._send(build_hil_sensor(
                self._sim_time_usec,
                sensors['accel_body'], sensors['gyro_body'],
                sensors['mag_body'], sensors['baro_pressure'],
                sensors['diff_pressure'], sensors['pressure_alt']))
            time.sleep(0.05)
            # Check for COMMAND_ACK on UDP
            try:
                raw, _ = arm_sock.recvfrom(4096)
                if raw:
                    msgs = self._parser.parse(raw)
                    for msg_id, payload in msgs:
                        if msg_id == 77:  # COMMAND_ACK
                            if len(payload) >= 3:
                                cmd = struct.unpack('<H', payload[0:2])[0]
                                result = payload[2]
                                print(f"[BRIDGE]   COMMAND_ACK (port {port}): cmd={cmd} "
                                      f"result={result} "
                                      f"({'ACCEPTED' if result == 0 else f'REJECTED({result})'})")
                                if cmd == MAV_CMD_COMPONENT_ARM_DISARM and result == 0:
                                    arm_ack_received = True
                        elif arm_attempt < 3:
                            print(f"[BRIDGE]   UDP RX port={port} msg_id={msg_id} "
                                  f"len={len(payload)}")
            except socket.timeout:
                pass
            # Drain TCP actuator controls
            try:
                self._sock.setblocking(False)
                raw = self._sock.recv(4096)
                if raw:
                    for msg_id, payload in self._parser.parse(raw):
                        if msg_id == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
                            parsed = parse_hil_actuator_controls(payload)
                            if parsed:
                                self._last_controls = np.array(parsed['controls'])
            except (BlockingIOError, socket.timeout):
                pass
            finally:
                self._sock.setblocking(True)
            if arm_ack_received:
                print(f"[BRIDGE] ARM ACCEPTED at attempt {arm_attempt + 1} "
                      f"(port {port})")
                break
        arm_sock.close()
        if not arm_ack_received:
            print("[BRIDGE] WARNING: No ARM ACK — trying to proceed anyway")
        print(f"[BRIDGE] ARM phase complete. t_sim={self._sim_time_usec/1e6:.2f}s")

        print("[BRIDGE] PX4 initialized & armed — starting flight simulation")
        t_wall_start = time.monotonic()

        # Time offset: main loop timestamps must continue from init warm-up.
        # Add one dt tick so the first main-loop timestamp is strictly greater
        # than the last ARM-phase timestamp (PX4 vehicle_imu rejects duplicates).
        _t_offset_us = self._sim_time_usec + init_dt_us
        _first_nonzero_logged = False

        while step < n_steps and self._running:
            t = step * dt
            self._sim_time_usec = _t_offset_us + int(t * 1e6)

            # Update fin commands from last received controls
            # PX4 rocket_mpc publishes fin deflections in radians directly
            self._fin_commands_rad = np.array([
                self._last_controls[0],
                self._last_controls[1],
                self._last_controls[2],
                self._last_controls[3],
            ])

            # Integrate one step (track wall-clock time per step)
            _t_step_wall = time.monotonic()
            try:
                next_state, snapshot, t_end, _ = sim._integrate_one_step(
                    state, t, dt, bridge_control)
            except Exception as e:
                logger.error("Simulation error at t=%.4f: %s", t, e)
                print(f"[BRIDGE] Simulation error at t={t:.4f}: {e}")
                break

            self._last_step_dt_ms = (time.monotonic() - _t_step_wall) * 1000.0
            sim._normalize_state(next_state, t_end)

            # Build sensor data
            sensors = self._build_sensors(snapshot, next_state, t_end,
                                           self._sim_time_usec)

            # Send periodic HEARTBEAT (~1Hz sim-time)
            if step % 100 == 0:
                self._send(build_heartbeat())

            # Send HIL_STATE_QUATERNION BEFORE HIL_SENSOR so groundtruth
            # is published before lockstep time advances.
            if step % state_interval == 0:
                self._send(build_hil_state_quaternion(
                    self._sim_time_usec,
                    sensors['quat'], sensors['omega'],
                    sensors['lat'], sensors['lon'], sensors['alt'],
                    sensors['vel_ned'][0], sensors['vel_ned'][1],
                    sensors['vel_ned'][2],
                    sensors['accel_body_true'],
                    sensors['airspeed']))

            # Send HIL_GPS
            if step % gps_interval == 0 and sensors['gps'] is not None:
                gps = sensors['gps']
                self._send(build_hil_gps(
                    self._sim_time_usec,
                    gps[0], gps[1], gps[2],
                    gps[3], gps[4], gps[5]))

            # Send HIL_SENSOR LAST — triggers lockstep time advance
            if step % sensor_interval == 0:
                self._send(build_hil_sensor(
                    self._sim_time_usec,
                    sensors['accel_body'], sensors['gyro_body'],
                    sensors['mag_body'], sensors['baro_pressure'],
                    sensors['diff_pressure'], sensors['pressure_alt'],
                    sensor_id=0))

            # Receive actuator controls from PX4
            if self.timing == 'lockstep':
                # In lockstep: MUST wait for PX4 to respond
                timeout = self.lockstep_timeout_ms / 1000.0
                deadline = time.monotonic() + timeout
                got_response = False
                while time.monotonic() < deadline:
                    msgs = self._recv(timeout_sec=min(0.1, timeout))
                    for msg_id, payload in msgs:
                        if msg_id == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
                            parsed = parse_hil_actuator_controls(payload)
                            if parsed:
                                self._last_controls = np.array(parsed['controls'])
                                self._last_flags = parsed.get('flags', 0)
                                self._last_mode = parsed.get('mode', 0)
                                got_response = True
                                if not _first_nonzero_logged and any(abs(c) > 1e-6 for c in parsed['controls'][:4]):
                                    _first_nonzero_logged = True
                                    print(f"[BRIDGE] First non-zero controls at step {step} t={t:.3f}s: "
                                          f"{parsed['controls'][:4]}")
                    if got_response:
                        break
                if not got_response and step > 10:
                    logger.warning("Lockstep timeout at step %d (t=%.3f)", step, t)
            else:
                # Real-time: non-blocking receive + pace simulation to wall-clock
                msgs = self._recv(timeout_sec=0.0001)
                for msg_id, payload in msgs:
                    if msg_id == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
                        parsed = parse_hil_actuator_controls(payload)
                        if parsed:
                            self._last_controls = np.array(parsed['controls'])
                            self._last_flags = parsed.get('flags', 0)
                            self._last_mode = parsed.get('mode', 0)
                            if not _first_nonzero_logged and any(abs(c) > 1e-6 for c in parsed['controls'][:4]):
                                _first_nonzero_logged = True
                                print(f"[BRIDGE] First non-zero controls at step {step} t={t:.3f}s: "
                                      f"{parsed['controls'][:4]}")
                # Pace: wait until wall-clock catches up with sim time
                target_wall = t_wall_start + t
                now_wall = time.monotonic()
                if now_wall < target_wall:
                    time.sleep(target_wall - now_wall)

            # Log history
            self._log_step(snapshot, next_state, t_end, sensors)

            # Check ground impact
            if sim._detect_ground_impact(next_state, t_end):
                print(f"\n[BRIDGE] Ground impact detected at t={t_end:.3f}s")
                break

            state = next_state
            step += 1

            # Progress
            if step % 1000 == 0:
                pct = 100.0 * step / n_steps
                alt = -next_state[2] if not sim.long_range_mode else sensors['alt'] - self.launch_alt
                spd = np.linalg.norm(next_state[3:6])
                elapsed = time.monotonic() - t_wall_start
                ratio = t / elapsed if elapsed > 0 else 0
                print(f"\r[BRIDGE] {pct:5.1f}% | t={t:.1f}s | "
                      f"alt={alt:.0f}m | v={spd:.0f}m/s | "
                      f"fins=[{self._fin_commands_rad[0]:.3f},{self._fin_commands_rad[1]:.3f},"
                      f"{self._fin_commands_rad[2]:.3f},{self._fin_commands_rad[3]:.3f}] | "
                      f"ratio={ratio:.1f}x", end='', flush=True)

        print(f"\n[BRIDGE] Simulation complete: {step} steps, t={t:.3f}s")
        t_wall = time.monotonic() - t_wall_start
        print(f"[BRIDGE] Wall time: {t_wall:.1f}s, ratio: {t / t_wall:.1f}x")

        # Close connection
        if self._sock:
            self._sock.close()
            self._sock = None

        # Export CSV
        if csv_output:
            self._export_csv(csv_output)
            print(f"[BRIDGE] Results saved to {csv_output}")

        return self.history

    def _log_step(self, snapshot: dict, state: np.ndarray,
                  t: float, sensors: dict):
        """Log one timestep to history."""
        pos = state[0:3]
        vel = state[3:6]
        quat = state[6:10]
        omega = state[10:13]
        mass = state[13] if len(state) > 13 else 0.0

        self.history['time'].append(t)
        self.history['position'].append(pos.copy())
        self.history['velocity'].append(vel.copy())
        self.history['attitude'].append(quat.copy())
        self.history['angular_velocity'].append(omega.copy())
        self.history['mass'].append(mass)
        self.history['fin_commands'].append(self._fin_commands_rad.copy())
        self.history['fin_actual'].append(
            np.array(snapshot.get('control_fins_rad', np.zeros(4))))
        self.history['forces'].append(
            np.array(snapshot.get('forces', [0, 0, 0])))
        self.history['altitude'].append(sensors['alt'] - self.launch_alt)
        self.history['ground_range'].append(
            np.sqrt(pos[0]**2 + pos[1]**2) if not self.sim.long_range_mode
            else snapshot.get('ground_range_km', 0.0) * 1000.0)
        self.history['alpha'].append(snapshot.get('alpha', 0.0))
        self.history['beta'].append(snapshot.get('beta', 0.0))
        self.history['mach'].append(snapshot.get('mach', 0.0))
        self.history['accel_body'].append(sensors['accel_body_true'].copy())
        self.history['gyro_body'].append(sensors['omega'].copy())
        self.history['lat'].append(sensors['lat'])
        self.history['lon'].append(sensors['lon'])
        self.history['alt_msl'].append(sensors['alt'])
        # PX4 status per step
        self.history['px4_armed'].append(1 if (self._last_flags & 0x1) else 0)
        self.history['px4_mode'].append(self._last_mode)
        self.history['step_dt_ms'].append(self._last_step_dt_ms)

    def _export_csv(self, path: str):
        """Export flight history to CSV."""
        n = len(self.history['time'])
        if n == 0:
            return

        pos = np.array(self.history['position'])
        vel = np.array(self.history['velocity'])
        att = np.array(self.history['attitude'])
        omega = np.array(self.history['angular_velocity'])
        fins_cmd = np.array(self.history['fin_commands'])
        fins_act = np.array(self.history['fin_actual'])
        forces = np.array(self.history['forces'])
        accel = np.array(self.history['accel_body'])

        with open(path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            header = [
                'time', 'pos_x', 'pos_y', 'pos_z',
                'vel_x', 'vel_y', 'vel_z',
                'q0', 'q1', 'q2', 'q3',
                'omega_x', 'omega_y', 'omega_z',
                'mass', 'altitude', 'ground_range',
                'alpha', 'beta', 'mach',
                'fin_cmd_1', 'fin_cmd_2', 'fin_cmd_3', 'fin_cmd_4',
                'fin_act_1', 'fin_act_2', 'fin_act_3', 'fin_act_4',
                'force_x', 'force_y', 'force_z',
                'accel_x', 'accel_y', 'accel_z',
                'lat', 'lon', 'alt_msl',
                'px4_armed', 'px4_mode', 'step_dt_ms',
            ]
            writer.writerow(header)

            for i in range(n):
                row = [
                    f"{self.history['time'][i]:.6f}",
                    f"{pos[i,0]:.6f}", f"{pos[i,1]:.6f}", f"{pos[i,2]:.6f}",
                    f"{vel[i,0]:.6f}", f"{vel[i,1]:.6f}", f"{vel[i,2]:.6f}",
                    f"{att[i,0]:.8f}", f"{att[i,1]:.8f}",
                    f"{att[i,2]:.8f}", f"{att[i,3]:.8f}",
                    f"{omega[i,0]:.8f}", f"{omega[i,1]:.8f}", f"{omega[i,2]:.8f}",
                    f"{self.history['mass'][i]:.4f}",
                    f"{self.history['altitude'][i]:.4f}",
                    f"{self.history['ground_range'][i]:.4f}",
                    f"{self.history['alpha'][i]:.8f}",
                    f"{self.history['beta'][i]:.8f}",
                    f"{self.history['mach'][i]:.6f}",
                    f"{fins_cmd[i,0]:.8f}", f"{fins_cmd[i,1]:.8f}",
                    f"{fins_cmd[i,2]:.8f}", f"{fins_cmd[i,3]:.8f}",
                    f"{fins_act[i,0]:.8f}", f"{fins_act[i,1]:.8f}",
                    f"{fins_act[i,2]:.8f}", f"{fins_act[i,3]:.8f}",
                    f"{forces[i,0]:.4f}", f"{forces[i,1]:.4f}", f"{forces[i,2]:.4f}",
                    f"{accel[i,0]:.6f}", f"{accel[i,1]:.6f}", f"{accel[i,2]:.6f}",
                    f"{self.history['lat'][i]:.8f}",
                    f"{self.history['lon'][i]:.8f}",
                    f"{self.history['alt_msl'][i]:.4f}",
                    f"{self.history['px4_armed'][i]}",
                    f"{self.history['px4_mode'][i]}",
                    f"{self.history['step_dt_ms'][i]:.3f}",
                ]
                writer.writerow(row)


# ============================================================================
# CLI Entry Point
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description='M130 SITL MAVLink Bridge')
    parser.add_argument('--sitl-config', type=str,
                        default=str(_SCRIPT_DIR / 'sitl_config.yaml'),
                        help='Path to SITL bridge config YAML')
    parser.add_argument('--sim-config', type=str, default=None,
                        help='Path to simulation config YAML (default: 6dof_config_advanced.yaml)')
    parser.add_argument('--csv', type=str, default=None,
                        help='Output CSV file path')
    parser.add_argument('--timing', type=str, choices=['lockstep', 'realtime'],
                        default=None, help='Override timing mode')
    parser.add_argument('--port', type=int, default=None,
                        help='Override TCP/UDP port')
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s [%(name)s] %(levelname)s: %(message)s')

    bridge = SITLBridge(args.sitl_config, args.sim_config)

    if args.timing:
        bridge.timing = args.timing
    if args.port:
        bridge.port = args.port

    csv_path = args.csv or str(_SCRIPT_DIR / 'sitl_flight.csv')
    bridge.run(csv_output=csv_path)


if __name__ == '__main__':
    main()
