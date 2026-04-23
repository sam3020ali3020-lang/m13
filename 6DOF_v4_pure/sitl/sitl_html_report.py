#!/usr/bin/env python3
"""
sitl_html_report.py — SITL Analysis Report (نفس تنسيق advanced_analysis.py بالضبط)

Maps SITL CSV + PX4 ULG → advanced_analysis.py schema → identical HTML report + MHE/MPC tab

Usage:
    python3 sitl_html_report.py <csv> --html <out.html> [--ulg <path>] [--no-open]
    python3 sitl_html_report.py <csv>  # auto-named output
"""

import sys, os, argparse, importlib.util, webbrowser
from pathlib import Path
from datetime import datetime
from collections import OrderedDict

import numpy as np
import pandas as pd

# ── Load advanced_analysis module from reference path ─────────────────────────
_AA_PATH = Path(__file__).resolve().parent.parent.parent / 'results' / 'advanced_analysis.py'
if not _AA_PATH.exists():
    _AA_PATH = Path(__file__).resolve().parent.parent / 'results' / 'advanced_analysis.py'

_spec = importlib.util.spec_from_file_location("advanced_analysis", str(_AA_PATH))
aa = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(aa)

import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly.io as pio

G              = 9.80665
LAUNCH_ALT_M   = aa.LAUNCH_ALT_M    # 1200.0 m
TARGET_RANGE_M = aa.TARGET_RANGE_M  # e.g. 2900 m

# ── ECEF → local NED ──────────────────────────────────────────────────────────
def _ecef_vel_to_ned(vx_ecef, vy_ecef, vz_ecef, lat0_deg, lon0_deg):
    """Convert ECEF velocity components to local NED (North, East, Down)."""
    lat0 = np.radians(lat0_deg)
    lon0 = np.radians(lon0_deg)
    slat, clat = np.sin(lat0), np.cos(lat0)
    slon, clon = np.sin(lon0), np.cos(lon0)
    vn = -slat * clon * vx_ecef - slat * slon * vy_ecef + clat * vz_ecef
    ve = -slon * vx_ecef + clon * vy_ecef
    vd = -clat * clon * vx_ecef - clat * slon * vy_ecef - slat * vz_ecef
    return vn, ve, vd


def _ecef_pos_to_ned(px_ecef, py_ecef, pz_ecef, lat0_deg, lon0_deg, alt0_msl):
    """Convert ECEF position to local NED from launch point (WGS84)."""
    lat0 = np.radians(lat0_deg)
    lon0 = np.radians(lon0_deg)
    slat, clat = np.sin(lat0), np.cos(lat0)
    slon, clon = np.sin(lon0), np.cos(lon0)
    R_N = 6378137.0 / np.sqrt(1.0 - 0.00669437999014 * slat**2)
    x0  = (R_N + alt0_msl) * clat * clon
    y0  = (R_N + alt0_msl) * clat * slon
    z0  = (R_N * (1.0 - 0.00669437999014) + alt0_msl) * slat
    dx, dy, dz = px_ecef - x0, py_ecef - y0, pz_ecef - z0
    pos_n = -slat * clon * dx - slat * slon * dy + clat * dz
    pos_e = -slon * dx + clon * dy
    pos_d = -clat * clon * dx - clat * slon * dy - slat * dz
    return pos_n, pos_e, pos_d


# ── Quaternion → Euler (ZYX) ──────────────────────────────────────────────────
def _quat_to_euler(q0, q1, q2, q3):
    """Quaternion (w, x, y, z) → (roll_deg, pitch_deg, yaw_deg) ZYX."""
    sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1.0 - 2.0 * (q1**2 + q2**2)
    roll      = np.arctan2(sinr_cosp, cosr_cosp)
    sinp      = np.clip(2.0 * (q0 * q2 - q3 * q1), -1.0, 1.0)
    pitch     = np.arcsin(sinp)
    siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1.0 - 2.0 * (q2**2 + q3**2)
    yaw       = np.arctan2(siny_cosp, cosy_cosp)
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


# ── Flight phase ──────────────────────────────────────────────────────────────
def _compute_flight_phase(df):
    t        = df['time_s'].values
    alt      = df['alt_agl_m'].values
    vel      = df['velocity_total_m_s'].values
    mass     = df['mass_kg'].values
    n        = len(df)
    phase    = np.full(n, 'TERMINAL', dtype=object)
    peak_idx = int(np.argmax(alt))
    mass_rate = np.gradient(mass, t)
    burning   = mass_rate < -0.05           # threshold: losing ≥ 0.05 kg/s
    for i in range(n):
        if alt[i] < 1.0 and vel[i] < 8.0:
            phase[i] = 'ARMED'
        elif burning[i]:
            phase[i] = 'LAUNCH' if alt[i] < 5.0 else 'BOOST'
        elif i <= peak_idx:
            phase[i] = 'COAST'
        else:
            phase[i] = 'TERMINAL'
    return phase


# ── ISA atmosphere ─────────────────────────────────────────────────────────────
def _isa_density(alt_msl):
    T = np.maximum(288.15 - 0.0065 * alt_msl, 216.65)
    p = 101325.0 * (T / 288.15) ** 5.2561
    return p / (287.05 * T)


def _isa_sos(alt_msl):
    T = np.maximum(288.15 - 0.0065 * alt_msl, 216.65)
    return np.sqrt(1.4 * 287.05 * T)


# ── SITL CSV loader → advanced_analysis schema ────────────────────────────────
def _load_sitl_csv(path: Path) -> pd.DataFrame:
    raw = pd.read_csv(str(path))
    df  = pd.DataFrame()

    # ── Reference point (launch) ─────────────────────────────────────────────
    lat0    = float(raw['lat'].iloc[0])
    lon0    = float(raw['lon'].iloc[0])
    alt0_msl = float(raw['alt_msl'].iloc[0])

    # ── Time ─────────────────────────────────────────────────────────────────
    t_arr           = raw['time'].astype(float).values
    df['time_s']    = t_arr

    # ── Altitude ─────────────────────────────────────────────────────────────
    alt_msl_arr     = raw['alt_msl'].astype(float).values
    alt_agl_arr     = raw['altitude'].astype(float).values   # AGL in metres
    df['altitude_m']    = alt_msl_arr                        # MSL (matches reference)
    df['alt_agl_m']     = alt_agl_arr
    df['ground_range_m'] = raw['ground_range'].astype(float)

    # ── Position: ECEF → NED ─────────────────────────────────────────────────
    pos_n, pos_e, pos_d = _ecef_pos_to_ned(
        raw['pos_x'].astype(float).values,
        raw['pos_y'].astype(float).values,
        raw['pos_z'].astype(float).values,
        lat0, lon0, alt0_msl)
    df['position_x_m'] = pos_n
    df['position_y_m'] = pos_e
    df['position_z_m'] = pos_d

    # ── Velocity: ECEF → NED ─────────────────────────────────────────────────
    vn, ve, vd = _ecef_vel_to_ned(
        raw['vel_x'].astype(float).values,
        raw['vel_y'].astype(float).values,
        raw['vel_z'].astype(float).values,
        lat0, lon0)
    vtot = np.sqrt(vn**2 + ve**2 + vd**2)
    df['velocity_x_m_s']        = vn
    df['velocity_y_m_s']        = ve
    df['velocity_z_m_s']        = vd
    df['velocity_total_m_s']    = vtot
    df['speed_horizontal_m_s']  = np.sqrt(vn**2 + ve**2)
    df['speed_vertical_m_s']    = -vd            # positive = climbing
    df['airspeed_m_s']          = vtot           # no wind
    df['mach_aero']             = raw['mach'].astype(float)

    # ── Euler angles ─────────────────────────────────────────────────────────
    roll_d, pitch_d, yaw_d = _quat_to_euler(
        raw['q0'].astype(float).values,
        raw['q1'].astype(float).values,
        raw['q2'].astype(float).values,
        raw['q3'].astype(float).values)
    df['roll_deg']  = roll_d
    df['pitch_deg'] = pitch_d
    df['yaw_deg']   = yaw_d
    df['roll_rad']  = np.radians(roll_d)
    df['pitch_rad'] = np.radians(pitch_d)
    df['yaw_rad']   = np.radians(yaw_d)

    # ── Angular rates ─────────────────────────────────────────────────────────
    df['omega_x_deg_s']     = np.degrees(raw['omega_x'].astype(float))
    df['omega_y_deg_s']     = np.degrees(raw['omega_y'].astype(float))
    df['omega_z_deg_s']     = np.degrees(raw['omega_z'].astype(float))
    df['omega_total_deg_s'] = np.sqrt(df['omega_x_deg_s']**2 +
                                       df['omega_y_deg_s']**2 +
                                       df['omega_z_deg_s']**2)

    # ── AoA / Sideslip ────────────────────────────────────────────────────────
    alpha_rad = raw['alpha'].astype(float).values
    beta_rad  = raw['beta'].astype(float).values
    df['alpha_rad']   = alpha_rad
    df['beta_rad']    = beta_rad
    df['alpha_deg']   = np.degrees(alpha_rad)
    df['beta_deg']    = np.degrees(beta_rad)

    # ── Mass & propulsion ─────────────────────────────────────────────────────
    mass_arr       = raw['mass'].astype(float).values
    df['mass_kg']  = mass_arr
    df['mach']     = raw['mach'].astype(float)
    m0, m_end      = mass_arr[0], mass_arr[-1]
    prop_used      = max(m0 - m_end, 1e-3)
    prop_frac      = np.clip((mass_arr - m_end) / prop_used, 0.0, 1.0)
    df['propellant_fraction'] = prop_frac
    df['xbc_m']               = 0.55 - 0.05 * (1.0 - prop_frac)

    # ── Atmosphere ────────────────────────────────────────────────────────────
    rho                       = _isa_density(alt_msl_arr)
    df['q_dynamic_Pa']        = 0.5 * rho * vtot**2
    df['speed_of_sound_m_s']  = _isa_sos(alt_msl_arr)

    # ── Forces ────────────────────────────────────────────────────────────────
    fx = raw['force_x'].astype(float).values
    fy = raw['force_y'].astype(float).values
    fz = raw['force_z'].astype(float).values
    df['force_x_N'] = fx
    df['force_y_N'] = fy
    df['force_z_N'] = fz
    ax = raw['accel_x'].astype(float).values
    ay = raw['accel_y'].astype(float).values
    az = raw['accel_z'].astype(float).values
    df['acceleration_body_x_g']  = ax / G
    df['acceleration_body_y_g']  = ay / G
    df['acceleration_body_z_g']  = az / G
    df['g_total']                = np.sqrt((ax/G)**2 + (ay/G)**2 + (az/G)**2)
    df['acceleration_fur_x_m_s2'] = ax
    df['acceleration_fur_y_m_s2'] = -az
    df['acceleration_fur_z_m_s2'] = ay
    # Thrust: net axial force (proxy)
    df['thrust_x_N']    = np.maximum(0.0, mass_arr * ax)
    df['thrust_y_N']    = np.zeros(len(df))
    df['thrust_z_N']    = np.zeros(len(df))
    df['thrust_total_N'] = df['thrust_x_N']

    # ── Aero coefficients ─────────────────────────────────────────────────────
    S_ref = 0.0127
    L_ref = 0.127
    q_Pa  = df['q_dynamic_Pa'].values
    qS    = np.where(q_Pa * S_ref > 1.0, q_Pa * S_ref, np.nan)
    df['CN_total']          = -fz / qS
    df['CY_total']          = -fy / qS
    df['CN_control']        = np.zeros(len(df))
    df['CY_control']        = np.zeros(len(df))
    df['CN_delta']          = np.zeros(len(df))
    df['CM_total']          = np.zeros(len(df))
    df['CM_control']        = np.zeros(len(df))
    df['CM_delta']          = np.zeros(len(df))
    df['Cn_total']          = np.zeros(len(df))
    df['Cn_control']        = np.zeros(len(df))
    df['CA']                = np.zeros(len(df))
    df['moment_x_Nm']       = np.zeros(len(df))
    df['moment_y_Nm']       = np.zeros(len(df))
    df['moment_z_Nm']       = np.zeros(len(df))
    df['moment_total_Nm']   = np.zeros(len(df))
    df['M_pitch_aero']      = np.zeros(len(df))
    df['M_yaw_aero']        = np.zeros(len(df))
    df['M_roll_aero']       = np.zeros(len(df))
    df['static_margin_cal'] = np.full(len(df), np.nan)

    # ── Energy ────────────────────────────────────────────────────────────────
    df['KE_kJ']           = 0.5 * mass_arr * vtot**2 / 1000.0
    df['PE_kJ']           = mass_arr * G * alt_agl_arr / 1000.0
    df['total_energy_kJ'] = df['KE_kJ'] + df['PE_kJ']
    dt_arr                = np.where(np.gradient(t_arr) > 0, np.gradient(t_arr), 1e-6)
    df['dE_dt_kW']        = np.gradient(df['total_energy_kJ'].values) / dt_arr

    # ── Flight path angle & range error ───────────────────────────────────────
    df['gamma_deg']     = aa._flight_path_angle(vn, ve, vd, vtot)
    df['range_error_m'] = df['ground_range_m'] - TARGET_RANGE_M

    # ── Fin deflections ────────────────────────────────────────────────────────
    for j in range(1, 5):
        df[f'fin_{j}_rad']              = raw[f'fin_act_{j}'].astype(float)
        df[f'actuator_cmd_fin{j}_rad']  = raw[f'fin_cmd_{j}'].astype(float)
        df[f'actuator_cmd_fin{j}_deg']  = np.degrees(raw[f'fin_cmd_{j}'].astype(float))
        df[f'actuator_lag_fin{j}_deg']  = np.degrees(
            raw[f'fin_cmd_{j}'].astype(float) - raw[f'fin_act_{j}'].astype(float))
    f1 = raw['fin_act_1'].astype(float).values
    f2 = raw['fin_act_2'].astype(float).values
    f3 = raw['fin_act_3'].astype(float).values
    f4 = raw['fin_act_4'].astype(float).values
    df['delta_pitch_rad'] = (-f1 + f2 + f3 - f4) / 4.0
    df['delta_yaw_rad']   = (-f1 - f2 + f3 + f4) / 4.0
    df['delta_roll_rad']  = ( f1 - f2 + f3 - f4) / 4.0
    df['delta_pitch_deg'] = np.degrees(df['delta_pitch_rad'])
    df['delta_yaw_deg']   = np.degrees(df['delta_yaw_rad'])
    df['delta_roll_deg']  = np.degrees(df['delta_roll_rad'])

    # ── Velocity frames ────────────────────────────────────────────────────────
    df['velocity_fur_x_m_s']  = vtot * np.cos(alpha_rad) * np.cos(beta_rad)
    df['velocity_fur_y_m_s']  = vtot * np.sin(np.radians(df['gamma_deg']))
    df['velocity_fur_z_m_s']  = np.zeros(len(df))
    df['position_fur_x_m']    = np.cumsum(df['velocity_fur_x_m_s'].values * dt_arr)
    df['position_fur_y_m']    = alt_agl_arr.copy()
    df['position_fur_z_m']    = np.zeros(len(df))
    df['vel_ned_north_m_s']   = vn
    df['vel_ned_east_m_s']    = ve
    df['vel_ned_down_m_s']    = vd
    df['vel_aero_north_m_s']  = vn
    df['vel_aero_east_m_s']   = ve
    df['vel_aero_down_m_s']   = vd

    # ── Geographic ────────────────────────────────────────────────────────────
    df['latitude_deg']   = raw['lat'].astype(float)
    df['longitude_deg']  = raw['lon'].astype(float)
    df['altitude_lla_m'] = alt_msl_arr

    # ── Fin authority & safety ────────────────────────────────────────────────
    df['fin_authority']      = np.minimum(1.0, df['q_dynamic_Pa'] / 500.0)
    df['safety_violations']  = np.zeros(len(df))

    # ── attrs flags ───────────────────────────────────────────────────────────
    df.attrs['has_mhe']           = False
    df.attrs['has_sensor']        = False
    df.attrs['has_lla']           = True
    df.attrs['has_actuator_cmd']  = True
    df.attrs['has_fins']          = True
    df.attrs['has_mpc_diag']      = False
    df.attrs['has_angular_accel'] = False
    df.attrs['has_mhe_states']    = False

    # ── Flight phase ──────────────────────────────────────────────────────────
    df['flight_phase'] = _compute_flight_phase(df)

    return df


# ── ULG loader ────────────────────────────────────────────────────────────────
def _load_ulg(ulg_path: str) -> dict:
    """Load PX4 ULG and return rocket_gnc_status arrays."""
    try:
        from pyulog import ULog
    except ImportError:
        return {}
    try:
        ulog = ULog(ulg_path)
    except Exception:
        return {}
    try:
        msgs = [m for m in ulog.data_list if m.name == 'rocket_gnc_status']
        if not msgs:
            return {}
        data = msgs[0].data
        t0   = data['timestamp'][0]
        out  = {'t': (data['timestamp'] - t0) / 1e6}
        for k in data.keys():
            if k != 'timestamp':
                try:
                    out[k] = data[k].astype(float)
                except Exception:
                    pass
        return out
    except Exception:
        return {}


def _find_ulg(px4_bin: str = '') -> str:
    """Auto-detect the most recent ULG log."""
    candidates = []
    roots = [
        Path('/home/wd/Desktop/gab_2/q_2/2/m13/AndroidApp/app/src/main/cpp/'
             'PX4-Autopilot/build/px4_sitl_default/log'),
    ]
    if px4_bin:
        p = Path(px4_bin).parent
        while p != p.parent:
            c = p / 'log'
            if c.is_dir():
                roots.append(c)
            p = p.parent
    for r in roots:
        if r.is_dir():
            candidates.extend(r.rglob('*.ulg'))
    if not candidates:
        return ''
    candidates.sort(key=lambda f: f.stat().st_mtime, reverse=True)
    return str(candidates[0])


# ── MHE/MPC tab ───────────────────────────────────────────────────────────────
def _build_mhe_tab_html(gnc: dict, df_sim: pd.DataFrame) -> str:
    has_gnc = bool(gnc) and 'mhe_quality' in gnc

    config = {"responsive": True, "displayModeBar": True,
              "modeBarButtonsToRemove": ["lasso2d", "select2d"]}

    if not has_gnc:
        return (
            '<div id="tab-mhe" class="tab-panel">'
            '<h2>🎯 MHE / MPC — Real PX4 Data</h2>'
            '<div class="card"><div class="diag warning">'
            '<div class="dtitle">⚠️ ULG not loaded</div>'
            '<div class="ddetail">No MHE/MPC data. Run with --ulg to enable this tab.</div>'
            '</div></div></div>')

    gnc_t     = gnc['t']
    sim_t     = df_sim['time_s'].values
    sim_alpha = df_sim['alpha_deg'].values

    def _g(key):
        arr = gnc.get(key)
        return arr.astype(float) if arr is not None else np.zeros_like(gnc_t)

    mhv       = _g('mhe_valid')
    mhq       = _g('mhe_quality')
    mhe_st    = _g('mhe_status')
    mpc_st    = _g('mpc_solver_status')
    blend     = _g('blend_alpha')
    alpha_est = _g('alpha_est')
    pc        = _g('pitch_accel_cmd')
    yc        = _g('yaw_accel_cmd')
    airspeed  = _g('airspeed')
    sc        = _g('mpc_solve_count')
    fin1      = _g('fin1')
    fin2      = _g('fin2')
    fin3      = _g('fin3')
    fin4      = _g('fin4')
    gamma_err = _g('xval_gamma_err')
    chi_err   = _g('xval_chi_err')
    xmpc0     = _g('x_mpc[0]')

    # Key event times
    def _first(arr, thr=0.5):
        idx = np.argmax(arr > thr)
        return float(gnc_t[idx]) if arr[idx] > thr else None

    t_mhe_v  = _first(mhv, 0.5)
    t_mpc_a  = _first(sc,  0.5)
    t_blend  = _first(blend, 0.001)
    avg_mhq  = float(np.mean(mhq[mhv > 0.5])) if (mhv > 0.5).any() else 0.0
    max_sc   = int(sc.max()) if len(sc) > 0 else 0

    fig = make_subplots(rows=4, cols=2,
        subplot_titles=(
            '① MHE Valid×100  +  Quality %',
            '① Solver Status  (1=OK)',
            '② α: MHE estimate vs truth',
            '② Blend factor  (0=MPC only → 1=LOS only)',
            '③ MPC accel commands',
            '③ x_mpc[0] vs airspeed',
            '④ Tracking errors (γ, χ)',
            '④ Fin avg: PX4 vs sim',
        ),
        vertical_spacing=0.10, horizontal_spacing=0.10)

    # R1 L – MHE valid + quality
    fig.add_trace(go.Scatter(x=gnc_t, y=mhv * 100,  name='Valid ×100',
                             fill='tozeroy', fillcolor='rgba(76,175,80,0.25)',
                             line=dict(color='#4caf50', width=2.5)), row=1, col=1)
    fig.add_trace(go.Scatter(x=gnc_t, y=mhq * 100,  name='Quality %',
                             fill='tozeroy', fillcolor='rgba(25,118,210,0.2)',
                             line=dict(color='#1976d2', width=2.5)), row=1, col=1)
    if t_mhe_v:
        fig.add_vline(x=t_mhe_v, line_color='#1976d2', line_dash='dot', line_width=2,
                      annotation_text=f'Valid @{t_mhe_v:.2f}s',
                      annotation_font_size=10, row=1, col=1)
    fig.add_hline(y=80, line_dash='dash', line_color='orange',
                  annotation_text='80%', row=1, col=1)

    # R1 R – solver status
    fig.add_trace(go.Scatter(x=gnc_t, y=np.where(mhe_st == 0, 1.0, 0.0),
                             name='MHE Solver (1=OK)', fill='tozeroy',
                             fillcolor='rgba(25,118,210,0.3)',
                             line=dict(color='#1976d2', width=2.5)), row=1, col=2)
    fig.add_trace(go.Scatter(x=gnc_t, y=np.where(mpc_st == 0, 1.0, 0.0) - 0.05,
                             name='MPC Solver (1=OK)', fill='tozeroy',
                             fillcolor='rgba(123,31,162,0.2)',
                             line=dict(color='#7b1fa2', width=2.5, dash='dot')), row=1, col=2)
    if t_mpc_a:
        fig.add_vline(x=t_mpc_a, line_color='#7b1fa2', line_dash='dot', line_width=2,
                      annotation_text=f'MPC @{t_mpc_a:.2f}s',
                      annotation_font_size=10, row=1, col=2)

    # R2 L – alpha_est vs truth
    fig.add_trace(go.Scatter(x=gnc_t, y=np.degrees(alpha_est), name='α MHE (alpha_est)',
                             line=dict(color='#1976d2', width=2.5)), row=2, col=1)
    fig.add_trace(go.Scatter(x=sim_t,  y=sim_alpha,             name='α truth (sim)',
                             line=dict(color='#d62728', width=2, dash='dash')), row=2, col=1)

    # R2 R – blend_alpha
    fig.add_trace(go.Scatter(x=gnc_t, y=blend, name='blend_alpha',
                             fill='tozeroy', fillcolor='rgba(245,124,0,0.35)',
                             line=dict(color='#f57c00', width=3)), row=2, col=2)
    fig.add_hline(y=0.5, line_dash='dash', line_color='#666',
                  annotation_text='50/50', row=2, col=2)

    # R3 L – pitch + yaw cmd
    fig.add_trace(go.Scatter(x=gnc_t, y=pc, name='Pitch α-dot cmd',
                             line=dict(color='#1f77b4', width=2.5)), row=3, col=1)
    fig.add_trace(go.Scatter(x=gnc_t, y=yc, name='Yaw  α-dot cmd',
                             line=dict(color='#d62728', width=2.5)), row=3, col=1)
    fig.add_hline(y=0, line_dash='dot', line_color='#ccc', row=3, col=1)

    # R3 R – x_mpc[0] vs airspeed
    fig.add_trace(go.Scatter(x=gnc_t, y=xmpc0,    name='x_mpc[0]=V (MHE→MPC)',
                             line=dict(color='#1f77b4', width=2.5)), row=3, col=2)
    fig.add_trace(go.Scatter(x=gnc_t, y=airspeed, name='Airspeed (measured)',
                             line=dict(color='#d62728', width=2, dash='dash')), row=3, col=2)

    # R4 L – tracking errors
    fig.add_trace(go.Scatter(x=gnc_t, y=np.degrees(gamma_err), name='γ error (deg)',
                             fill='tozeroy', fillcolor='rgba(25,118,210,0.15)',
                             line=dict(color='#1976d2', width=2.5)), row=4, col=1)
    fig.add_trace(go.Scatter(x=gnc_t, y=np.degrees(chi_err),   name='χ error (deg)',
                             fill='tozeroy', fillcolor='rgba(211,47,47,0.15)',
                             line=dict(color='#d32f2f', width=2.5)), row=4, col=1)
    fig.add_hline(y=0, line_dash='dot', line_color='#ccc', row=4, col=1)

    # R4 R – fin avg
    fin_avg_px4 = (fin1 + fin2 + fin3 + fin4) / 4.0
    fin_avg_sim_rad = (df_sim['fin_1_rad'].values + df_sim['fin_2_rad'].values +
                       df_sim['fin_3_rad'].values + df_sim['fin_4_rad'].values) / 4.0
    fig.add_trace(go.Scatter(x=gnc_t, y=np.degrees(fin_avg_px4),
                             name='Fin avg PX4 (deg)',
                             line=dict(color='#1976d2', width=2.5)), row=4, col=2)
    fig.add_trace(go.Scatter(x=sim_t,  y=np.degrees(fin_avg_sim_rad),
                             name='Fin avg sim (deg)',
                             line=dict(color='#d62728', width=2, dash='dash')), row=4, col=2)
    fig.add_hline(y=0, line_dash='dot', line_color='#ccc', row=4, col=2)

    fig.update_layout(
        height=1200, template='plotly_white',
        title_text='🎯 MHE / MPC — Real PX4 Data (rocket_gnc_status from ULG)',
        legend=dict(font=dict(size=10), bgcolor='rgba(255,255,255,0.9)',
                    bordercolor='#e0e0e0', borderwidth=1))
    for r in range(1, 5):
        fig.update_xaxes(title_text='Time (s)', row=r, col=1, showgrid=True)
        fig.update_xaxes(title_text='Time (s)', row=r, col=2, showgrid=True)
    fig.update_yaxes(title_text='% (0–100)',              row=1, col=1)
    fig.update_yaxes(title_text='0=FAIL  1=OK',           row=1, col=2)
    fig.update_yaxes(title_text='AoA (deg)',               row=2, col=1)
    fig.update_yaxes(title_text='0=MPC … 1=LOS',          row=2, col=2)
    fig.update_yaxes(title_text='rad/s²',                  row=3, col=1)
    fig.update_yaxes(title_text='m/s',                     row=3, col=2)
    fig.update_yaxes(title_text='Error (deg)',             row=4, col=1)
    fig.update_yaxes(title_text='Avg deflection (deg)',    row=4, col=2)

    plot_div = pio.to_html(fig, full_html=False, include_plotlyjs=False, config=config)

    def _mc(label, val, sub=''):
        return (f'<div class="metric-box"><div class="value">{val}</div>'
                f'<div class="label">{label}</div><div class="sub">{sub}</div></div>')

    t_mhe_s = f'{t_mhe_v:.3f}s'  if t_mhe_v  else 'N/A'
    t_mpc_s = f'{t_mpc_a:.3f}s'  if t_mpc_a  else 'N/A'
    t_bld_s = f'{t_blend:.3f}s'  if (t_blend and t_blend > 0.01) else 'None (pure MPC)'

    cards = (
        f'<div class="grid grid-4" style="margin-bottom:16px">'
        f'<div class="card">{_mc("MHE Valid first", t_mhe_s, "mhe_valid=1")}</div>'
        f'<div class="card">{_mc("MPC Active first", t_mpc_s, "mpc_solve_count>0")}</div>'
        f'<div class="card">{_mc("Blend started", t_bld_s, "blend_alpha>0.001")}</div>'
        f'<div class="card">{_mc("MHE Quality avg", f"{avg_mhq*100:.1f}%", f"when valid | {max_sc} MPC cycles")}</div>'
        f'</div>')

    note = (
        '<div class="card" style="margin-bottom:16px">'
        '<p style="font-size:.85rem;color:#555;line-height:1.9">'
        '<b>①</b> MHE Valid×100 + Quality % — exact PX4 moment when MHE starts estimating. '
        'Right: MHE + MPC solver status.<br>'
        '<b>②</b> α_est (blue) vs truth α from sim CSV (red dashed) — MHE accuracy. '
        'Right: blend_alpha — 0=pure MPC guidance, 1=pure LOS guidance.<br>'
        '<b>③</b> MPC pitch/yaw accel commands. '
        'Right: x_mpc[0] vs airspeed — proves MHE state reaches MPC.<br>'
        '<b>④</b> γ & χ tracking errors (from xval). '
        'Right: avg fin PX4 vs sim — validates fin command pipeline.'
        '</p></div>')

    return (
        f'<div id="tab-mhe" class="tab-panel">'
        f'<h2>🎯 MHE / MPC — Real PX4 Data</h2>'
        f'<p style="color:#666;font-size:.85rem;margin-bottom:12px">'
        f'ULG: <code>rocket_gnc_status</code>  ({len(gnc_t)} samples, '
        f't = {gnc_t[0]:.2f}–{gnc_t[-1]:.2f} s)</p>'
        f'{cards}{note}'
        f'<div class="chart-container">{plot_div}</div>'
        f'</div>')


# ── Main report generator ─────────────────────────────────────────────────────
def generate_sitl_html_report(csv_path, html_path=None, ulg_path=None,
                               px4_log_path=None, metadata=None, auto_open=False):
    csv_path = Path(csv_path)
    if html_path is None:
        html_path = csv_path.parent / (csv_path.stem + '_report.html')

    print(f'  Loading SITL CSV: {csv_path.name}')
    df = _load_sitl_csv(csv_path)

    metrics = aa._extract_run_metrics(df, csv_path)
    scores  = aa._score_run(metrics)
    diags   = aa._diagnose(df, metrics)
    recs    = aa._recommend(metrics, scores, diags)
    phases  = aa._analyze_phases(df)
    metrics['file']      = csv_path.name
    metrics['timestamp'] = csv_path.stem

    print('  Generating HTML (reference format)...')
    html = aa.generate_html_report(df, metrics, scores, diags, recs, phases,
                                    html_path=None)

    # Fix title
    html = html.replace('M130 6-DOF Simulation Analysis', 'M130 SITL Analysis')

    # ── Inject MHE/MPC tab ────────────────────────────────────────────────────
    gnc = {}
    if ulg_path and Path(ulg_path).exists():
        print(f'  Loading ULG: {Path(ulg_path).name}')
        gnc = _load_ulg(str(ulg_path))

    mhe_html = _build_mhe_tab_html(gnc, df)

    # Button injection: tabs bar ends with ...>[last button]</div><div id="tab-overview"
    if '</div><div id="tab-overview"' in html:
        html = html.replace(
            '</div><div id="tab-overview"',
            '<button class="tab-btn" onclick="openTab(event,\'tab-mhe\')">'
            '🎯 MHE/MPC</button>'
            '</div><div id="tab-overview"',
            1)

    # Panel injection: before the final </div></div> that closes the tab container
    inject_marker = '</div></div>\n<script>'
    if inject_marker in html:
        html = html.replace(inject_marker,
                            f'\n{mhe_html}\n</div></div>\n<script>', 1)
    else:
        html = html.replace('</div></div><script>',
                            f'\n{mhe_html}\n</div></div><script>', 1)

    Path(html_path).write_text(html, encoding='utf-8')
    print(f'  ✓ Report: {html_path}')

    if auto_open:
        webbrowser.open(f'file://{Path(html_path).resolve()}')

    return str(html_path)


# ── CLI ───────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='SITL HTML Report — exact reference format')
    parser.add_argument('csv',        help='SITL CSV file path')
    parser.add_argument('--html',     default=None, help='Output HTML path')
    parser.add_argument('--ulg',      default=None, help='PX4 ULG log path')
    parser.add_argument('--px4-log',  default=None, help='PX4 stdout log')
    parser.add_argument('--no-open',  action='store_true',
                        help="Don't open browser after generation")
    args = parser.parse_args()
    ulg = args.ulg or _find_ulg()
    generate_sitl_html_report(
        args.csv,
        html_path=args.html,
        ulg_path=ulg,
        px4_log_path=args.px4_log,
        auto_open=not args.no_open)


if __name__ == '__main__':
    main()
