#!/usr/bin/env python3
"""
sitl_html_report.py  — M130 SITL Comprehensive Flight Report
=============================================================
مصادر البيانات:
  1. CSV  — بيانات المحاكاة (6DOF): مسار، سرعة، اتجاه، أوامر الزعانف، etc.
  2. ULG  — لوق PX4: rocket_gnc_status (MHE/MPC الحقيقي), vehicle_attitude,
             sensor_combined, actuator_outputs_sim, vehicle_local_position_groundtruth
  3. PX4 stdout log — أحداث الإقلاع، التسليح، الكشف عن الإطلاق

التابات:
  Overview · Trajectory · 3D View · Attitude · Aero & Forces ·
  Forces Detail · Control · G-Load & Energy · Velocity · Phases ·
  Stability · FFT Spectrum · Tracking · [NEW] MHE/MPC · SITL Diagnostics
"""
from __future__ import annotations
import os, csv, re, json, math, webbrowser
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, List, Any
import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# Legacy CSV header (files without header row)
# ─────────────────────────────────────────────────────────────────────────────
_LEGACY_HDR = [
    'time','pos_x','pos_y','pos_z','vel_x','vel_y','vel_z',
    'q0','q1','q2','q3','omega_x','omega_y','omega_z',
    'mass','altitude','ground_range','alpha','beta','mach',
    'fin_cmd_1','fin_cmd_2','fin_cmd_3','fin_cmd_4',
    'fin_act_1','fin_act_2','fin_act_3','fin_act_4',
    'force_x','force_y','force_z','accel_x','accel_y','accel_z',
    'lat','lon','alt_msl',
]

# ─────────────────────────────────────────────────────────────────────────────
# CSV loader
# ─────────────────────────────────────────────────────────────────────────────
def _load_csv(path: str) -> Dict[str, np.ndarray]:
    with open(path, encoding='utf-8') as f:
        first = f.readline().strip()
    has_hdr = first.startswith('time')
    raw: Dict[str, List[float]] = {}
    with open(path, encoding='utf-8') as f:
        if has_hdr:
            rdr = csv.DictReader(f)
            for row in rdr:
                for k, v in row.items():
                    raw.setdefault(k, [])
                    try: raw[k].append(float(v))
                    except: raw[k].append(0.0)
        else:
            rdr = csv.reader(f)
            for row in rdr:
                for i, v in enumerate(row):
                    col = _LEGACY_HDR[i] if i < len(_LEGACY_HDR) else f'col_{i}'
                    raw.setdefault(col, [])
                    try: raw[col].append(float(v))
                    except: raw[col].append(0.0)
    return {k: np.array(v) for k, v in raw.items()}

# ─────────────────────────────────────────────────────────────────────────────
# ULG loader — returns dict of topic → dict of field → np.ndarray
# ─────────────────────────────────────────────────────────────────────────────
def _load_ulg(ulg_path: str) -> Dict[str, Any]:
    """Load ULG and return {topic: {field: np.ndarray}}. Returns {} if unavailable."""
    try:
        from pyulog import ULog
        ulg = ULog(ulg_path)
        out: Dict[str, Any] = {}
        for d in ulg.data_list:
            name = d.name
            # Handle duplicate topics (e.g. sensor_baro appears twice)
            if name in out:
                name = f"{name}_1"
            out[name] = {k: np.array(v) for k, v in d.data.items()}
        return out
    except Exception as e:
        print(f"  [WARN] Could not load ULG: {e}")
        return {}

def _find_ulg(px4_bin: str) -> Optional[str]:
    """Find most recent ULG near PX4 build directory."""
    # Strategy 1: build/px4_sitl_default/log/**/*.ulg
    if px4_bin:
        build_dir = os.path.dirname(os.path.dirname(px4_bin))
        log_dir = os.path.join(build_dir, 'log')
        if os.path.isdir(log_dir):
            ulgs = sorted(Path(log_dir).rglob('*.ulg'), key=lambda p: p.stat().st_mtime)
            if ulgs:
                return str(ulgs[-1])
    # Strategy 2: any ULG in PX4-Autopilot tree
    candidates = []
    for root, dirs, files in os.walk('/home'):
        for f in files:
            if f.endswith('.ulg'):
                p = os.path.join(root, f)
                candidates.append((os.path.getmtime(p), p))
    if candidates:
        return sorted(candidates)[-1][1]
    return None

# ─────────────────────────────────────────────────────────────────────────────
# PX4 stdout log parser
# ─────────────────────────────────────────────────────────────────────────────
def _parse_px4_log(log_path: str) -> Dict:
    out = {
        'boot_ok': False, 'mhe_init': False, 'mpc_init': False,
        'launch_detected': False, 'arm_time': None, 'first_mpc_t': None,
        'dt_lines': [], 'warnings': [], 'errors': [], 'raw': '',
        'mhe_N': None, 'mpc_N': None, 'mpc_tf': None,
        'sensor_rate': None, 'origin_lat': None, 'origin_lon': None,
        'target_range': None, 'thrust': None,
    }
    if not log_path or not os.path.isfile(log_path):
        return out
    try:
        with open(log_path, encoding='utf-8', errors='replace') as f:
            txt = f.read()
        out['raw'] = txt
        lines = txt.splitlines()
        for ln in lines:
            if 'Startup script returned successfully' in ln: out['boot_ok'] = True
            if 'MHE solver init' in ln:
                out['mhe_init'] = True
                m = re.search(r'N=(\d+)', ln)
                if m: out['mhe_N'] = int(m.group(1))
                m = re.search(r'dt=([\d.]+)', ln)
                if m: out['sensor_rate'] = round(1/float(m.group(1)))
            if 'MPC solver init' in ln:
                out['mpc_init'] = True
                m = re.search(r'N=(\d+)', ln); 
                if m: out['mpc_N'] = int(m.group(1))
                m = re.search(r'tf=([\d.]+)', ln)
                if m: out['mpc_tf'] = float(m.group(1))
            if 'LAUNCH DETECTED' in ln: out['launch_detected'] = True
            if 'Armed by external command' in ln: out['arm_time'] = ln
            if 'First MPC cycle' in ln:
                m = re.search(r't=([\d.]+)', ln)
                if m: out['first_mpc_t'] = float(m.group(1))
            if re.search(r'dt: avg=', ln): out['dt_lines'].append(ln.strip())
            if ln.strip().startswith('WARN'): out['warnings'].append(ln.strip())
            if ln.strip().startswith('ERROR'): out['errors'].append(ln.strip())
            if 'GPS origin' in ln:
                m = re.search(r'lat=([\d.]+) lon=([\d.]+)', ln)
                if m: out['origin_lat'], out['origin_lon'] = float(m.group(1)), float(m.group(2))
            if 'range=' in ln and 'origin' in ln:
                m = re.search(r'range=(\d+)', ln)
                if m: out['target_range'] = int(m.group(1))
            if 'Thrust plateau' in ln:
                m = re.search(r'([\d.]+) N', ln)
                if m: out['thrust'] = float(m.group(1))
    except Exception as e:
        print(f"  [WARN] PX4 log parse error: {e}")
    return out

# ─────────────────────────────────────────────────────────────────────────────
# Quaternion → Euler
# ─────────────────────────────────────────────────────────────────────────────
def _q2euler(q0, q1, q2, q3):
    sinr = 2*(q0*q1 + q2*q3); cosr = 1 - 2*(q1*q1 + q2*q2)
    roll = np.degrees(np.arctan2(sinr, cosr))
    sinp = 2*(q0*q2 - q3*q1)
    sinp = np.clip(sinp, -1, 1)
    pitch = np.degrees(np.arcsin(sinp))
    siny = 2*(q0*q3 + q1*q2); cosy = 1 - 2*(q2*q2 + q3*q3)
    yaw = np.degrees(np.arctan2(siny, cosy))
    return roll, pitch, yaw

# ─────────────────────────────────────────────────────────────────────────────
# Plotly to HTML div
# ─────────────────────────────────────────────────────────────────────────────
def _div(fig, first=False):
    import plotly.io as pio
    return pio.to_html(fig, full_html=False,
                       include_plotlyjs='cdn' if first else False,
                       config={'responsive': True, 'displayModeBar': True,
                               'modeBarButtonsToRemove': ['lasso2d','select2d']})

# ─────────────────────────────────────────────────────────────────────────────
# Color helpers
# ─────────────────────────────────────────────────────────────────────────────
C = ['#1976d2','#d32f2f','#388e3c','#f57c00','#7b1fa2','#0097a7','#e64a19','#5d4037']

# ─────────────────────────────────────────────────────────────────────────────
# ═══════════════════════ MAIN GENERATOR ═══════════════════════════════════
# ─────────────────────────────────────────────────────────────────────────────
def generate_sitl_html_report(csv_path: str,
                               html_path: str,
                               metadata: Optional[Dict] = None,
                               px4_log_path: Optional[str] = None,
                               ulg_path: Optional[str] = None,
                               auto_open: bool = True) -> str:
    import plotly.graph_objects as go
    import plotly.subplots as sp

    metadata = metadata or {}
    # ── Load data ──────────────────────────────────────────────────────────
    d = _load_csv(csv_path)
    t = d['time']
    n = len(t)

    # Find ULG if not given
    if ulg_path is None:
        ulg_path = _find_ulg(metadata.get('px4_bin',''))
    ulg = _load_ulg(ulg_path) if ulg_path else {}
    px4 = _parse_px4_log(px4_log_path or '')

    def col(name, default=None):
        if name in d: return d[name]
        v = default if default is not None else np.zeros(n)
        return np.full(n, v) if np.isscalar(v) else v

    # ── Derived quantities ─────────────────────────────────────────────────
    roll, pitch, yaw = _q2euler(col('q0'), col('q1'), col('q2'), col('q3'))
    speed = np.sqrt(col('vel_x')**2 + col('vel_y')**2 + col('vel_z')**2)
    mach  = col('mach')
    alt   = col('altitude')
    rng   = col('ground_range')
    alpha = np.degrees(col('alpha'))
    beta  = np.degrees(col('beta'))
    ox    = np.degrees(col('omega_x')); oy = np.degrees(col('omega_y')); oz = np.degrees(col('omega_z'))
    accel_g = np.sqrt(col('accel_x')**2 + col('accel_y')**2 + col('accel_z')**2) / 9.81
    fx = col('force_x'); fy = col('force_y'); fz = col('force_z')

    fc = [np.degrees(col(f'fin_cmd_{i}')) for i in range(1,5)]
    fa = [np.degrees(col(f'fin_act_{i}')) for i in range(1,5)]
    fe = [fc[i]-fa[i] for i in range(4)]

    armed = col('px4_armed', 0)
    step_dt = col('step_dt_ms', 0)
    # MPC active steps
    mpc_norm = np.sqrt(sum(col(f'fin_cmd_{i}')**2 for i in range(1,5)))

    # ── ULG → rocket_gnc_status ────────────────────────────────────────────
    gnc = ulg.get('rocket_gnc_status', {})
    has_gnc = bool(gnc)
    def g(k, default=None):
        if k in gnc: return gnc[k]
        return np.array([]) if default is None else np.full(len(gnc.get('t_flight', np.array([1]))), default)

    gnc_t      = g('t_flight')
    gnc_mhq    = g('mhe_quality', 0)
    gnc_mhv    = g('mhe_valid', 0)
    gnc_alpha  = np.degrees(g('alpha_est', 0))
    gnc_fin1   = np.degrees(g('fin1', 0))
    gnc_fin2   = np.degrees(g('fin2', 0))
    gnc_fin3   = np.degrees(g('fin3', 0))
    gnc_fin4   = np.degrees(g('fin4', 0))
    gnc_phi    = np.degrees(g('phi', 0))
    gnc_theta  = np.degrees(g('theta', 0))
    gnc_psi    = np.degrees(g('psi', 0))
    gnc_alt    = g('altitude', 0)
    gnc_v      = g('airspeed', 0)
    gnc_qdyn   = g('q_dyn', 0)
    gnc_gamma  = np.degrees(g('gamma_rad', 0))
    gnc_xr     = g('pos_downrange', 0)
    gnc_xc     = g('pos_crossrange', 0)
    gnc_vd     = g('vel_downrange', 0)
    gnc_vz     = g('vel_down', 0)
    gnc_blend  = g('blend_alpha', 0)
    gnc_stage  = g('stage', 0)
    gnc_trr    = g('target_range_remaining', 0)
    gnc_ge     = g('xval_gamma_err', 0)
    gnc_ce     = g('xval_chi_err', 0)
    gnc_ae     = g('xval_alt_err', 0)
    gnc_pc     = g('pitch_accel_cmd', 0)
    gnc_yc     = g('yaw_accel_cmd', 0)
    gnc_dp     = g('delta_pitch', 0)
    gnc_dy     = g('delta_yaw', 0)
    gnc_dr     = g('delta_roll', 0)
    gnc_launched = g('launched', 0)
    gnc_solve_cnt = g('mpc_solve_count', 0)
    gnc_sqp   = g('mpc_sqp_iter', 0)
    gnc_mhus  = g('mhe_solve_us', 0)
    gnc_mpus  = g('mpc_solve_us', 0)
    gnc_cus   = gnc.get('cycle_us', np.zeros_like(gnc_t))
    gnc_fails = g('mpc_fail_count', 0)
    gnc_mhfail = g('mhe_fail_count', 0)

    # ── Summary stats ──────────────────────────────────────────────────────
    t_dur      = float(t[-1]) if len(t) else 0
    max_alt    = float(np.max(alt))
    max_spd    = float(np.max(speed))
    max_mach   = float(np.max(mach))
    max_alpha  = float(np.max(np.abs(alpha)))
    max_beta   = float(np.max(np.abs(beta)))
    max_G      = float(np.max(accel_g))
    max_om     = float(np.max(np.abs(np.concatenate([ox, oy, oz]))))
    final_rng  = float(rng[-1]) if len(rng) else 0
    impact_gamma = float(np.degrees(np.arctan2(-col('vel_z')[-1], speed[-1]))) if len(t) else 0
    target_rng = metadata.get('target_range') or (px4.get('target_range') or 0)
    rng_err    = abs(final_rng - target_rng) if target_rng else None
    pitch_std  = float(np.std(pitch))
    mpc_first_t = float(t[np.argmax(mpc_norm>0.001)]) if np.any(mpc_norm>0.001) else None
    mhe_first_valid = float(gnc_t[np.argmax(gnc_mhv>0)]) if has_gnc and np.any(gnc_mhv>0) else None
    avg_mhq    = float(np.mean(gnc_mhq[gnc_mhv>0])) if has_gnc and np.any(gnc_mhv>0) else 0
    avg_step   = float(np.mean(step_dt[step_dt>0])) if np.any(step_dt>0) else 0
    p99_step   = float(np.percentile(step_dt[step_dt>0], 99)) if np.any(step_dt>0) else 0
    max_fin_err = float(np.max([np.max(np.abs(fe[i])) for i in range(4)])) if any(len(fe[i]) for i in range(4)) else 0

    # Score
    score = 0; cats = []
    if target_rng and rng_err is not None:
        s = max(0, 40*(1 - rng_err/max(target_rng*0.1, 1))); score += min(40, s)
        cats.append(('Range Accuracy', 'PASS' if rng_err < target_rng*0.05 else 'WARN' if rng_err < target_rng*0.1 else 'FAIL',
                      min(40, s), 40, f'|err|={rng_err:.0f}m of {target_rng}m'))
    else:
        cats.append(('Range', 'INFO', 0, 40, f'final={final_rng:.0f}m'))
    if impact_gamma != 0:
        s = 15 if abs(impact_gamma) > 30 else 7; score += s
        cats.append(('Impact Angle', 'PASS' if abs(impact_gamma)>30 else 'WARN', s, 15, f'γ={impact_gamma:.1f}°'))
    cats.append(('Stability', 'PASS' if pitch_std<5 else 'WARN' if pitch_std<10 else 'FAIL',
                  max(0, 15-int(pitch_std)), 15, f'pitch σ={pitch_std:.1f}°'))
    score += max(0, 15-int(pitch_std))
    cats.append(('AoA Margin','PASS' if max_alpha<15 else 'WARN' if max_alpha<25 else 'FAIL',
                  10 if max_alpha<15 else 5 if max_alpha<25 else 0, 10, f'max|α|={max_alpha:.1f}°'))
    score += 10 if max_alpha<15 else 5 if max_alpha<25 else 0
    cats.append(('Sideslip','PASS' if max_beta<5 else 'WARN' if max_beta<10 else 'FAIL',
                  10 if max_beta<5 else 5 if max_beta<10 else 0, 10, f'max|β|={max_beta:.1f}°'))
    score += 10 if max_beta<5 else 5 if max_beta<10 else 0
    cats.append(('G-Load','PASS' if max_G<20 else 'WARN' if max_G<30 else 'FAIL',
                  10 if max_G<20 else 5, 10, f'max={max_G:.1f}g'))
    score += 10 if max_G<20 else 5
    score = min(100, score)
    score_cls = 'pass' if score>=80 else 'warn' if score>=60 else 'fail'

    # ── FIGURES ───────────────────────────────────────────────────────────

    # ── Tab: Trajectory ────────────────────────────────────────────────────
    fig_traj = sp.make_subplots(rows=2, cols=2,
        subplot_titles=('Altitude vs Time (m)', 'Ground Range vs Time (m)',
                        'Speed (m/s) & Mach', 'Ground Track N-E (m)'),
        vertical_spacing=0.13)
    fig_traj.add_trace(go.Scatter(x=t, y=alt, name='Altitude', line=dict(color=C[0],width=2)), row=1, col=1)
    fig_traj.add_trace(go.Scatter(x=t, y=rng, name='Range', line=dict(color=C[2],width=2)), row=1, col=2)
    fig_traj.add_trace(go.Scatter(x=t, y=speed, name='Speed m/s', line=dict(color=C[1],width=2)), row=2, col=1)
    fig_traj.add_trace(go.Scatter(x=t, y=mach, name='Mach', line=dict(color=C[3],dash='dash',width=1.5),
                                   yaxis='y'), row=2, col=1)
    fig_traj.add_trace(go.Scatter(x=col('pos_y'), y=col('pos_x'), name='Track',
                                   mode='lines', line=dict(color=C[4],width=2)), row=2, col=2)
    fig_traj.add_trace(go.Scatter(x=[col('pos_y')[0]], y=[col('pos_x')[0]],
                                   mode='markers', name='Launch',
                                   marker=dict(color='green', size=12, symbol='triangle-up')), row=2, col=2)
    fig_traj.add_trace(go.Scatter(x=[col('pos_y')[-1]], y=[col('pos_x')[-1]],
                                   mode='markers', name='Impact',
                                   marker=dict(color='red', size=12, symbol='x')), row=2, col=2)
    fig_traj.update_layout(height=640, title_text='Trajectory Overview')
    for r,c_ in [(1,1),(1,2),(2,1)]:
        fig_traj.update_xaxes(title_text='Time (s)', row=r, col=c_)
    fig_traj.update_xaxes(title_text='East (m)', row=2, col=2)
    fig_traj.update_yaxes(title_text='North (m)', row=2, col=2)

    # ── Tab: 3D View ──────────────────────────────────────────────────────
    # Compute NED relative (pos_x=N, pos_y=E, pos_z=D → alt=-pos_z approx)
    alt_agl = alt - float(alt[0])
    fig_3d = go.Figure()
    fig_3d.add_trace(go.Scatter3d(
        x=col('pos_y') - col('pos_y')[0],
        y=col('pos_x') - col('pos_x')[0],
        z=alt_agl,
        mode='lines', name='Trajectory',
        line=dict(color=speed, colorscale='Jet', width=4,
                  colorbar=dict(title='Speed m/s', x=1.02)),
    ))
    fig_3d.add_trace(go.Scatter3d(
        x=[0], y=[0], z=[0], mode='markers', name='Launch',
        marker=dict(color='green', size=8, symbol='diamond')))
    fig_3d.add_trace(go.Scatter3d(
        x=[col('pos_y')[-1]-col('pos_y')[0]],
        y=[col('pos_x')[-1]-col('pos_x')[0]],
        z=[alt_agl[-1]], mode='markers', name='Impact',
        marker=dict(color='red', size=10, symbol='x')))
    fig_3d.update_layout(height=600, title='3D Flight Path (color = speed)',
                          scene=dict(xaxis_title='East (m)', yaxis_title='North (m)',
                                     zaxis_title='AGL Altitude (m)',
                                     camera=dict(eye=dict(x=1.5,y=-1.5,z=0.8))))

    # ── Tab: Attitude ──────────────────────────────────────────────────────
    fig_att = sp.make_subplots(rows=2, cols=2,
        subplot_titles=('Euler Angles (deg)', 'Angular Rates (deg/s)',
                        'Angle of Attack & Sideslip (deg)', 'Quaternion Components'),
        vertical_spacing=0.13)
    for nm, arr, col_ in [('Roll', roll, C[0]),('Pitch', pitch, C[1]),('Yaw', yaw, C[2])]:
        fig_att.add_trace(go.Scatter(x=t, y=arr, name=nm, line=dict(color=col_,width=1.5)), row=1, col=1)
    for nm, arr, col_ in [('ωx', ox, C[0]),('ωy', oy, C[1]),('ωz', oz, C[2])]:
        fig_att.add_trace(go.Scatter(x=t, y=arr, name=nm, line=dict(color=col_,width=1.5)), row=1, col=2)
    fig_att.add_trace(go.Scatter(x=t, y=alpha, name='α AoA', line=dict(color=C[3],width=2)), row=2, col=1)
    fig_att.add_trace(go.Scatter(x=t, y=beta,  name='β Sideslip', line=dict(color=C[4],width=2)), row=2, col=1)
    for i, (qn, col_) in enumerate(zip(['q0','q1','q2','q3'], C[:4])):
        fig_att.add_trace(go.Scatter(x=t, y=col(qn), name=qn,
                                      line=dict(color=col_, width=1)), row=2, col=2)
    # Also overlay PX4 attitude from ULG if available
    va = ulg.get('vehicle_attitude_groundtruth', {})
    if va:
        ts_va = va['timestamp']/1e6
        r2,p2,y2 = _q2euler(va['q[0]'], va['q[1]'], va['q[2]'], va['q[3]'])
        fig_att.add_trace(go.Scatter(x=ts_va-ts_va[0], y=p2, name='Pitch(PX4)', opacity=0.6,
                                      line=dict(color=C[1],dash='dot',width=1)), row=1, col=1)
    fig_att.update_layout(height=640, title_text='Attitude & Angular Motion')
    for r,c_ in [(1,1),(1,2),(2,1),(2,2)]:
        fig_att.update_xaxes(title_text='Time (s)', row=r, col=c_)

    # ── Tab: Aero & Forces ─────────────────────────────────────────────────
    # Dynamic pressure from ULG if available
    fig_aero = sp.make_subplots(rows=2, cols=2,
        subplot_titles=('Dynamic Pressure q_dyn (Pa)', 'Aerodynamic Angles (deg)',
                        'Total Body Forces (N)', 'Mach Number'),
        vertical_spacing=0.13)
    if has_gnc and len(gnc_qdyn):
        fig_aero.add_trace(go.Scatter(x=gnc_t, y=gnc_qdyn, name='q_dyn [PX4]',
                                       line=dict(color=C[0],width=2)), row=1, col=1)
    # Estimate q_dyn from sim: 0.5 * rho * v^2 (rho≈1.2)
    qdyn_sim = 0.5 * 1.2 * speed**2
    fig_aero.add_trace(go.Scatter(x=t, y=qdyn_sim, name='q_dyn [sim]',
                                   line=dict(color=C[1],dash='dash',width=1)), row=1, col=1)
    fig_aero.add_trace(go.Scatter(x=t, y=alpha, name='α AoA', line=dict(color=C[2],width=2)), row=1, col=2)
    fig_aero.add_trace(go.Scatter(x=t, y=beta,  name='β Sideslip', line=dict(color=C[3],width=2)), row=1, col=2)
    for ax_, cc in zip(['x','y','z'], C[:3]):
        fig_aero.add_trace(go.Scatter(x=t, y=col(f'force_{ax_}'), name=f'F_{ax_}',
                                       line=dict(color=cc,width=1.5)), row=2, col=1)
    fig_aero.add_trace(go.Scatter(x=t, y=mach, name='Mach',
                                   line=dict(color=C[4],width=2)), row=2, col=2)
    fig_aero.update_layout(height=640, title_text='Aerodynamics & Forces')
    for r,c_ in [(1,1),(1,2),(2,1),(2,2)]:
        fig_aero.update_xaxes(title_text='Time (s)', row=r, col=c_)

    # ── Tab: Forces Detail ─────────────────────────────────────────────────
    fig_fd = sp.make_subplots(rows=2, cols=2,
        subplot_titles=('Axial & Normal Force (N)', 'Body Specific Force (m/s²)',
                        'Force Magnitude (N)', 'Aero vs Body Frame Angles'),
        vertical_spacing=0.13)
    fig_fd.add_trace(go.Scatter(x=t, y=fx, name='Fx (axial)', line=dict(color=C[0],width=2)), row=1, col=1)
    fig_fd.add_trace(go.Scatter(x=t, y=fy, name='Fy', line=dict(color=C[1],width=1.5)), row=1, col=1)
    fig_fd.add_trace(go.Scatter(x=t, y=fz, name='Fz (normal)', line=dict(color=C[2],width=1.5)), row=1, col=1)
    for ax_, cc in zip(['x','y','z'], C[:3]):
        fig_fd.add_trace(go.Scatter(x=t, y=col(f'accel_{ax_}'), name=f'a_{ax_}',
                                     line=dict(color=cc,width=1.5)), row=1, col=2)
    f_mag = np.sqrt(fx**2 + fy**2 + fz**2)
    fig_fd.add_trace(go.Scatter(x=t, y=f_mag, name='|F|', fill='tozeroy',
                                 line=dict(color=C[3],width=2)), row=2, col=1)
    fig_fd.add_trace(go.Scatter(x=t, y=alpha, name='α', line=dict(color=C[0],width=2)), row=2, col=2)
    fig_fd.add_trace(go.Scatter(x=t, y=beta, name='β', line=dict(color=C[1],width=2)), row=2, col=2)
    fig_fd.update_layout(height=640, title_text='Forces — Detailed View')
    for r,c_ in [(1,1),(1,2),(2,1),(2,2)]:
        fig_fd.update_xaxes(title_text='Time (s)', row=r, col=c_)

    # ── Tab: Control (Fins) ────────────────────────────────────────────────
    fig_ctrl = sp.make_subplots(rows=2, cols=2,
        subplot_titles=('Fin 1 CMD vs ACT (deg)', 'Fin 2 CMD vs ACT (deg)',
                        'Fin 3 CMD vs ACT (deg)', 'Fin 4 CMD vs ACT (deg)'),
        vertical_spacing=0.13)
    for i, (r_, c_) in enumerate([(1,1),(1,2),(2,1),(2,2)]):
        fig_ctrl.add_trace(go.Scatter(x=t, y=fc[i], name=f'Fin{i+1} CMD',
                                       line=dict(color=C[i],width=2)), row=r_, col=c_)
        fig_ctrl.add_trace(go.Scatter(x=t, y=fa[i], name=f'Fin{i+1} ACT',
                                       line=dict(color=C[i],dash='dot',width=1.5),
                                       opacity=0.8), row=r_, col=c_)
        # shaded error
        fig_ctrl.add_trace(go.Scatter(
            x=np.concatenate([t, t[::-1]]),
            y=np.concatenate([fc[i], fa[i][::-1]]),
            fill='toself', fillcolor=f'rgba(50,50,200,0.1)',
            line=dict(color='rgba(0,0,0,0)'), name=f'Err{i+1}', showlegend=False
        ), row=r_, col=c_)
    fig_ctrl.update_layout(height=640, title_text='Fin Control — CMD vs ACT')
    for r,c_ in [(1,1),(1,2),(2,1),(2,2)]:
        fig_ctrl.update_xaxes(title_text='Time (s)', row=r, col=c_)
        fig_ctrl.update_yaxes(title_text='Deflection (deg)', row=r, col=c_)

    # ── Tab: G-Load & Energy ───────────────────────────────────────────────
    ke = 0.5 * col('mass') * speed**2
    pe = col('mass') * 9.81 * alt
    fig_ge = sp.make_subplots(rows=2, cols=2,
        subplot_titles=('G-Load (g)', 'G Components (g)',
                        'Kinetic Energy (J)', 'Potential Energy (J)'),
        vertical_spacing=0.13)
    fig_ge.add_trace(go.Scatter(x=t, y=accel_g, name='G-Load', fill='tozeroy',
                                 line=dict(color=C[0],width=2)), row=1, col=1)
    fig_ge.add_hline(y=20, line_dash='dash', line_color='red', row=1, col=1,
                     annotation_text='20g limit')
    for ax_, cc in zip(['x','y','z'], C[:3]):
        fig_ge.add_trace(go.Scatter(x=t, y=col(f'accel_{ax_}')/9.81, name=f'G_{ax_}',
                                     line=dict(color=cc,width=1.5)), row=1, col=2)
    fig_ge.add_trace(go.Scatter(x=t, y=ke, name='KE', fill='tozeroy',
                                 line=dict(color=C[1],width=2)), row=2, col=1)
    fig_ge.add_trace(go.Scatter(x=t, y=pe, name='PE', fill='tozeroy',
                                 line=dict(color=C[2],width=2)), row=2, col=2)
    fig_ge.update_layout(height=640, title_text='G-Load & Energy')
    for r,c_ in [(1,1),(1,2),(2,1),(2,2)]:
        fig_ge.update_xaxes(title_text='Time (s)', row=r, col=c_)

    # ── Tab: Velocity ──────────────────────────────────────────────────────
    vel_ned = np.sqrt(col('vel_x')**2 + col('vel_y')**2)
    fig_vel = sp.make_subplots(rows=2, cols=2,
        subplot_titles=('Total Speed (m/s)', 'Velocity Components NED (m/s)',
                        'Mach Number', 'Flight Path Angle γ (deg)'),
        vertical_spacing=0.13)
    fig_vel.add_trace(go.Scatter(x=t, y=speed, name='V_total',
                                  line=dict(color=C[0],width=2)), row=1, col=1)
    for nm, arr, cc in [('Vx',col('vel_x'),C[1]),('Vy',col('vel_y'),C[2]),('Vz',col('vel_z'),C[3])]:
        fig_vel.add_trace(go.Scatter(x=t, y=arr, name=nm, line=dict(color=cc,width=1.5)), row=1, col=2)
    fig_vel.add_trace(go.Scatter(x=t, y=mach, name='Mach', fill='tozeroy',
                                  line=dict(color=C[4],width=2)), row=2, col=1)
    gamma = np.degrees(np.arctan2(-col('vel_z'), np.maximum(vel_ned, 0.01)))
    fig_vel.add_trace(go.Scatter(x=t, y=gamma, name='γ fpa',
                                  line=dict(color=C[0],width=2)), row=2, col=2)
    if has_gnc:
        fig_vel.add_trace(go.Scatter(x=gnc_t, y=np.degrees(gnc_gamma), name='γ[PX4]', opacity=0.7,
                                      line=dict(color=C[1],dash='dot',width=1)), row=2, col=2)
    fig_vel.update_layout(height=640, title_text='Velocity Analysis')
    for r,c_ in [(1,1),(1,2),(2,1),(2,2)]:
        fig_vel.update_xaxes(title_text='Time (s)', row=r, col=c_)

    # ── Tab: Phases ────────────────────────────────────────────────────────
    phase = np.zeros(n)  # 0=pre, 1=boost, 2=coast
    # boost: mpc_norm>0 and speed increasing
    dv = np.gradient(speed, t)
    for i in range(n):
        if float(armed[i]) < 0.5: phase[i] = 0
        elif dv[i] > 1: phase[i] = 1
        else: phase[i] = 2
    fig_ph = sp.make_subplots(rows=2, cols=2,
        subplot_titles=('Phase Timeline', 'Altitude by Phase',
                        'Speed by Phase', 'Stage (from PX4 GNC)'),
        vertical_spacing=0.13)
    phase_colors = {0:'#bdbdbd', 1:'#ff7043', 2:'#42a5f5'}
    for ph, nm in [(0,'Pre-launch'),(1,'Boost'),(2,'Coast')]:
        mask = phase == ph
        if not np.any(mask): continue
        fig_ph.add_trace(go.Scatter(x=t[mask], y=alt[mask], name=nm, mode='lines',
                                     line=dict(color=phase_colors[ph], width=2)), row=1, col=2)
        fig_ph.add_trace(go.Scatter(x=t[mask], y=speed[mask], name=nm+'_v', mode='lines',
                                     line=dict(color=phase_colors[ph], width=2),
                                     showlegend=False), row=2, col=1)
    # Phase timeline bar (correct horizontal bar per phase)
    dur_pre   = float(np.sum(phase==0)) / n * t_dur
    dur_boost = float(np.sum(phase==1)) / n * t_dur
    dur_coast = float(np.sum(phase==2)) / n * t_dur
    for dur, nm, clr in [(dur_pre,'Pre-launch','#bdbdbd'),
                          (dur_boost,'Boost','#ff7043'),
                          (dur_coast,'Coast','#42a5f5')]:
        if dur > 0:
            fig_ph.add_trace(go.Bar(x=[dur], y=[nm], orientation='h',
                                     marker_color=clr, showlegend=False,
                                     text=[f'{dur:.1f}s'], textposition='inside'),
                              row=1, col=1)
    if has_gnc:
        stage_colors = {1:'#ff7043', 2:'#42a5f5', 3:'#66bb6a'}
        for st in [1,2,3]:
            mask = gnc_stage == st
            if np.any(mask):
                fig_ph.add_trace(go.Scatter(x=gnc_t[mask], y=np.full(mask.sum(), st),
                                             name=f'Stage {st}', mode='markers',
                                             marker=dict(color=stage_colors.get(st,'gray'), size=4)),
                                  row=2, col=2)
    fig_ph.update_layout(height=640, title_text='Flight Phases')
    for r,c_ in [(1,2),(2,1),(2,2)]:
        fig_ph.update_xaxes(title_text='Time (s)', row=r, col=c_)

    # ── Tab: Stability ─────────────────────────────────────────────────────
    fig_stab = sp.make_subplots(rows=2, cols=2,
        subplot_titles=('AoA vs Time', 'Sideslip vs Time',
                        'Pitch Rate vs Time', 'Pitch-AoA Phase Portrait'),
        vertical_spacing=0.13)
    fig_stab.add_trace(go.Scatter(x=t, y=alpha, name='α AoA',
                                   line=dict(color=C[0],width=2)), row=1, col=1)
    fig_stab.add_hline(y=15, line_dash='dash', line_color='orange', row=1, col=1)
    fig_stab.add_hline(y=-15, line_dash='dash', line_color='orange', row=1, col=1)
    fig_stab.add_trace(go.Scatter(x=t, y=beta, name='β',
                                   line=dict(color=C[1],width=2)), row=1, col=2)
    fig_stab.add_trace(go.Scatter(x=t, y=oy, name='q (pitch rate)',
                                   line=dict(color=C[2],width=2)), row=2, col=1)
    fig_stab.add_trace(go.Scatter(x=alpha, y=oy, name='Phase', mode='lines',
                                   line=dict(color=C[3],width=1)), row=2, col=2)
    fig_stab.update_layout(height=640, title_text='Stability Analysis')
    for r,c_ in [(1,1),(1,2),(2,1)]:
        fig_stab.update_xaxes(title_text='Time (s)', row=r, col=c_)
    fig_stab.update_xaxes(title_text='AoA (deg)', row=2, col=2)
    fig_stab.update_yaxes(title_text='Pitch Rate (deg/s)', row=2, col=2)

    # ── Tab: Phase Portrait ────────────────────────────────────────────────
    fig_port = sp.make_subplots(rows=2, cols=2,
        subplot_titles=('Pitch: θ vs q', 'Yaw: ψ vs r',
                        'Roll: φ vs p', 'AoA vs Pitch Rate'),
        vertical_spacing=0.13)
    fig_port.add_trace(go.Scatter(x=pitch, y=oy, mode='lines', name='pitch-q',
                                   line=dict(color=C[0],width=1.2)), row=1, col=1)
    fig_port.add_trace(go.Scatter(x=yaw, y=oz, mode='lines', name='yaw-r',
                                   line=dict(color=C[1],width=1.2)), row=1, col=2)
    fig_port.add_trace(go.Scatter(x=roll, y=ox, mode='lines', name='roll-p',
                                   line=dict(color=C[2],width=1.2)), row=2, col=1)
    fig_port.add_trace(go.Scatter(x=alpha, y=oy, mode='lines', name='α-q',
                                   line=dict(color=C[3],width=1.2)), row=2, col=2)
    fig_port.update_layout(height=640, title_text='Phase Portraits')
    for r,c_,xt,yt in [(1,1,'Pitch (deg)','q (deg/s)'),(1,2,'Yaw (deg)','r (deg/s)'),
                        (2,1,'Roll (deg)','p (deg/s)'),(2,2,'AoA (deg)','q (deg/s)')]:
        fig_port.update_xaxes(title_text=xt, row=r, col=c_)
        fig_port.update_yaxes(title_text=yt, row=r, col=c_)

    # ── Tab: FFT Spectrum ──────────────────────────────────────────────────
    dt_fft = float(np.mean(np.diff(t))) if len(t)>1 else 0.01
    def _fft(sig):
        f = np.fft.rfftfreq(len(sig), d=dt_fft)
        a = np.abs(np.fft.rfft(sig - np.mean(sig))) / len(sig) * 2
        return f, a
    fig_fft = sp.make_subplots(rows=2, cols=2,
        subplot_titles=('Pitch Rate FFT', 'Yaw Rate FFT',
                        'AoA FFT', 'Fin CMD 1 FFT'),
        vertical_spacing=0.13)
    for arr, nm, r_, c_ in [(oy,'Pitch Rate',1,1),(oz,'Yaw Rate',1,2),(alpha,'AoA',2,1),(fc[0],'Fin1 CMD',2,2)]:
        f_, a_ = _fft(arr)
        fig_fft.add_trace(go.Scatter(x=f_, y=a_, name=nm, fill='tozeroy',
                                      line=dict(width=1.5)), row=r_, col=c_)
        fig_fft.update_xaxes(title_text='Frequency (Hz)', range=[0,20], row=r_, col=c_)
    fig_fft.update_layout(height=640, title_text='FFT Frequency Spectrum')

    # ── Tab: Tracking ──────────────────────────────────────────────────────
    fig_track = sp.make_subplots(rows=2, cols=2,
        subplot_titles=('MPC xval — Gamma Error (rad)', 'MPC xval — Chi (Yaw) Error (rad)',
                        'MPC xval — Alt Error (m)', 'Target Range Remaining (m)'),
        vertical_spacing=0.13)
    if has_gnc:
        fig_track.add_trace(go.Scatter(x=gnc_t, y=gnc_ge, name='γ err',
                                        fill='tozeroy', line=dict(color=C[0],width=2)), row=1, col=1)
        fig_track.add_trace(go.Scatter(x=gnc_t, y=gnc_ce, name='χ err',
                                        fill='tozeroy', line=dict(color=C[1],width=2)), row=1, col=2)
        fig_track.add_trace(go.Scatter(x=gnc_t, y=gnc_ae, name='alt err',
                                        fill='tozeroy', line=dict(color=C[2],width=2)), row=2, col=1)
        fig_track.add_trace(go.Scatter(x=gnc_t, y=gnc_trr, name='range remaining',
                                        line=dict(color=C[3],width=2)), row=2, col=2)
    else:
        # Fallback: fin cmd norm as proxy
        fig_track.add_trace(go.Scatter(x=t, y=np.degrees(mpc_norm), name='‖fin_cmd‖',
                                        fill='tozeroy', line=dict(color=C[0],width=2)), row=1, col=1)
        fig_track.add_trace(go.Scatter(x=t, y=rng, name='Range (m)',
                                        line=dict(color=C[2],width=2)), row=2, col=2)
    fig_track.update_layout(height=640, title_text='Guidance Tracking Performance')
    for r,c_ in [(1,1),(1,2),(2,1),(2,2)]:
        fig_track.update_xaxes(title_text='Time (s)', row=r, col=c_)

    # ── [NEW] Tab: MHE / MPC ──────────────────────────────────────────────
    # ══ KEY EVENT TIMESTAMPS (from ULG) ═══════════════════════════════════
    def _first_t(arr, threshold=0.5):
        if not has_gnc or len(arr) == 0: return None
        idx = np.argmax(arr > threshold)
        return float(gnc_t[idx]) if arr[idx] > threshold else None

    t_launch      = _first_t(gnc_launched, 0.5)
    t_mhe_valid   = _first_t(gnc_mhv, 0.5)
    t_mpc_active  = None
    if has_gnc and 'mpc_solve_count' in gnc:
        _sc = gnc['mpc_solve_count']
        _idx = np.argmax(_sc > 0)
        if _sc[_idx] > 0: t_mpc_active = float(gnc_t[_idx])
    t_blend_start = _first_t(gnc_blend, 0.001)

    gnc_mhe_status    = gnc.get('mhe_status',        np.zeros_like(gnc_t)) if has_gnc else np.array([])
    gnc_mpc_status    = gnc.get('mpc_solver_status', np.zeros_like(gnc_t)) if has_gnc else np.array([])
    gnc_solve_cnt_arr = gnc.get('mpc_solve_count',   np.zeros_like(gnc_t)) if has_gnc else np.array([])

    # ══ 4 ROWS × 2 COLS — max 2 traces per chart, no clutter ══════════════
    fig_mhe = sp.make_subplots(rows=4, cols=2,
        subplot_titles=(
            '① متى بدأ MHE؟  —  mhe_valid & mhe_quality',
            '① حالة المحلّ  —  MHE solver OK / FAIL',
            '② دقة تقدير α  —  alpha_est vs truth',
            '② نسبة الدمج blend_alpha  (0=MPC | 1=LOS)',
            '③ أوامر MPC  —  pitch & yaw accel cmd',
            '③ سرعة MPC تأتي من MHE  —  x_mpc[0] vs V_measured',
            '④ أخطاء التتبع  —  γ & χ errors',
            '④ أوامر الزعانف  —  fin avg PX4 vs sim',
        ),
        vertical_spacing=0.1, horizontal_spacing=0.12)

    if has_gnc:
        # ── ROW 1 L: MHE Valid (step) + Quality (%) — 2 traces, clear ────
        # valid flag scaled to 100 so both fit same axis
        fig_mhe.add_trace(go.Scatter(
            x=gnc_t, y=gnc_mhv.astype(float) * 100,
            name='MHE Valid ×100',
            fill='tozeroy', fillcolor='rgba(76,175,80,0.25)',
            line=dict(color='#4caf50', width=2.5),
            hovertemplate='t=%{x:.2f}s  valid=%{customdata:.0f}',
            customdata=gnc_mhv.astype(float)), row=1, col=1)
        fig_mhe.add_trace(go.Scatter(
            x=gnc_t, y=gnc_mhq * 100,
            name='MHE Quality %',
            line=dict(color='#1976d2', width=2.5),
            fill='tozeroy', fillcolor='rgba(25,118,210,0.2)'), row=1, col=1)
        # mark the moment MHE became valid
        if t_mhe_valid:
            fig_mhe.add_vline(x=t_mhe_valid, line_color='#1976d2', line_dash='dot',
                               line_width=2, row=1, col=1,
                               annotation_text=f'Valid @ {t_mhe_valid:.2f}s',
                               annotation_font_size=10, annotation_font_color='#1976d2')
        fig_mhe.add_hline(y=80, line_dash='dash', line_color='orange',
                           annotation_text='Quality 80%', row=1, col=1)

        # ── ROW 1 R: Solver OK/FAIL — 2 clean step traces ─────────────────
        mhe_ok = np.where(gnc_mhe_status == 0, 1.0, 0.0)
        mpc_ok = np.where(gnc_mpc_status == 0, 1.0, 0.0)
        fig_mhe.add_trace(go.Scatter(
            x=gnc_t, y=mhe_ok,
            name='MHE Solver (1=OK)',
            fill='tozeroy', fillcolor='rgba(25,118,210,0.3)',
            line=dict(color='#1976d2', width=2.5)), row=1, col=2)
        fig_mhe.add_trace(go.Scatter(
            x=gnc_t, y=mpc_ok - 0.05,   # slight offset so both visible
            name='MPC Solver (1=OK)',
            fill='tozeroy', fillcolor='rgba(123,31,162,0.2)',
            line=dict(color='#7b1fa2', width=2.5, dash='dot')), row=1, col=2)
        if t_mpc_active:
            fig_mhe.add_vline(x=t_mpc_active, line_color='#7b1fa2', line_dash='dot',
                               line_width=2, row=1, col=2,
                               annotation_text=f'MPC @ {t_mpc_active:.2f}s',
                               annotation_font_size=10, annotation_font_color='#7b1fa2')

        # ── ROW 2 L: alpha_est vs truth — 2 lines ─────────────────────────
        fig_mhe.add_trace(go.Scatter(
            x=gnc_t, y=gnc_alpha,
            name='α MHE (alpha_est)', line=dict(color='#1976d2', width=2.5)), row=2, col=1)
        fig_mhe.add_trace(go.Scatter(
            x=t, y=alpha,
            name='α Truth (sim)', line=dict(color='#d32f2f', width=2, dash='dash')), row=2, col=1)

        # ── ROW 2 R: blend_alpha — 1 thick line, colored zones ───────────
        # Green zone = MPC active (blend near 0), orange zone = blend > 0
        fig_mhe.add_vrect(x0=float(gnc_t[0]), x1=float(gnc_t[-1]),
                           fillcolor='rgba(123,31,162,0.07)',
                           line_width=0, row=2, col=2,
                           annotation_text='MPC zone', annotation_position='top left',
                           annotation_font_color='#7b1fa2', annotation_font_size=9)
        fig_mhe.add_trace(go.Scatter(
            x=gnc_t, y=gnc_blend,
            name='blend_alpha',
            fill='tozeroy', fillcolor='rgba(245,124,0,0.35)',
            line=dict(color='#f57c00', width=3)), row=2, col=2)
        fig_mhe.add_hline(y=0.5, line_dash='dash', line_color='#666',
                           annotation_text='50/50', row=2, col=2)
        if t_blend_start and t_blend_start > 0.1:
            fig_mhe.add_vline(x=t_blend_start, line_color='#f57c00', line_dash='dot',
                               line_width=2, row=2, col=2,
                               annotation_text=f'Blend @ {t_blend_start:.2f}s',
                               annotation_font_size=10, annotation_font_color='#f57c00')

        # ── ROW 3 L: MPC accel commands — 2 lines ─────────────────────────
        fig_mhe.add_trace(go.Scatter(
            x=gnc_t, y=gnc_pc,
            name='Pitch accel cmd (rad/s²)', line=dict(color=C[0], width=2.5)), row=3, col=1)
        fig_mhe.add_trace(go.Scatter(
            x=gnc_t, y=gnc_yc,
            name='Yaw accel cmd (rad/s²)', line=dict(color=C[1], width=2.5)), row=3, col=1)
        fig_mhe.add_hline(y=0, line_dash='dot', line_color='#ccc', row=3, col=1)

        # ── ROW 3 R: x_mpc[0]=V vs measured V — 2 lines (proves MHE→MPC) ─
        _xv = gnc.get('x_mpc[0]', np.zeros_like(gnc_t))
        fig_mhe.add_trace(go.Scatter(
            x=gnc_t, y=_xv,
            name='x_mpc[0] = V (from MHE)', line=dict(color=C[0], width=2.5)), row=3, col=2)
        fig_mhe.add_trace(go.Scatter(
            x=gnc_t, y=gnc_v,
            name='V measured (airspeed)', line=dict(color=C[1], width=2, dash='dash')), row=3, col=2)

        # ── ROW 4 L: tracking errors (gamma + chi only, cleaner) ──────────
        fig_mhe.add_trace(go.Scatter(
            x=gnc_t, y=np.degrees(gnc_ge),
            name='γ error (deg)',
            fill='tozeroy', fillcolor='rgba(25,118,210,0.15)',
            line=dict(color=C[0], width=2.5)), row=4, col=1)
        fig_mhe.add_trace(go.Scatter(
            x=gnc_t, y=np.degrees(gnc_ce),
            name='χ error (deg)',
            fill='tozeroy', fillcolor='rgba(211,47,47,0.15)',
            line=dict(color=C[1], width=2.5)), row=4, col=1)
        fig_mhe.add_hline(y=0, line_dash='dot', line_color='#ccc', row=4, col=1)

        # ── ROW 4 R: average fin PX4 vs average fin sim — 2 lines ─────────
        fin_avg_px4 = (gnc_fin1 + gnc_fin2 + gnc_fin3 + gnc_fin4) / 4.0
        fin_avg_sim = np.interp(gnc_t, t, (fc[0]+fc[1]+fc[2]+fc[3])/4.0)
        fig_mhe.add_trace(go.Scatter(
            x=gnc_t, y=fin_avg_px4,
            name='Fin avg [PX4]', line=dict(color=C[0], width=2.5)), row=4, col=2)
        fig_mhe.add_trace(go.Scatter(
            x=gnc_t, y=fin_avg_sim,
            name='Fin avg [sim]', line=dict(color=C[1], width=2, dash='dash')), row=4, col=2)
        fig_mhe.add_hline(y=0, line_dash='dot', line_color='#ccc', row=4, col=2)

    else:
        # Fallback CSV-only
        fig_mhe.add_trace(go.Scatter(x=t, y=armed, name='Armed', fill='tozeroy',
                                      line=dict(color='green',width=2)), row=1, col=1)
        fig_mhe.add_trace(go.Scatter(x=t, y=alpha, name='α truth',
                                      line=dict(color=C[0],width=2)), row=2, col=1)
        fin_avg_sim2 = (fc[0]+fc[1]+fc[2]+fc[3])/4.0
        fig_mhe.add_trace(go.Scatter(x=t, y=fin_avg_sim2, name='Fin avg CMD',
                                      line=dict(color=C[1],width=2)), row=4, col=2)

    fig_mhe.update_layout(
        height=1300,
        title_text='🎯 MHE / MPC  —  بيانات حقيقية من ULG: rocket_gnc_status',
        legend=dict(font=dict(size=10), bgcolor='rgba(255,255,255,0.9)',
                    bordercolor='#e0e0e0', borderwidth=1))

    for r in range(1, 5):
        fig_mhe.update_xaxes(title_text='Time (s)', row=r, col=1, showgrid=True)
        fig_mhe.update_xaxes(title_text='Time (s)', row=r, col=2, showgrid=True)

    fig_mhe.update_yaxes(title_text='%  (0–100)', row=1, col=1)
    fig_mhe.update_yaxes(title_text='0=FAIL  1=OK', row=1, col=2)
    fig_mhe.update_yaxes(title_text='AoA (deg)', row=2, col=1)
    fig_mhe.update_yaxes(title_text='0=pure MPC  1=pure LOS', row=2, col=2)
    fig_mhe.update_yaxes(title_text='rad/s²', row=3, col=1)
    fig_mhe.update_yaxes(title_text='m/s', row=3, col=2)
    fig_mhe.update_yaxes(title_text='Error (deg)', row=4, col=1)
    fig_mhe.update_yaxes(title_text='Fin avg (deg)', row=4, col=2)

    # ── Tab: SITL Diagnostics ──────────────────────────────────────────────
    fig_diag = sp.make_subplots(rows=2, cols=2,
        subplot_titles=('Step Wall-Clock Time (ms)', 'Fin CMD Distribution (deg)',
                        'Body Forces (N)', 'Speed + Altitude Profile'),
        vertical_spacing=0.13)
    step_dt_nz = step_dt[step_dt > 0]
    if len(step_dt_nz):
        fig_diag.add_trace(go.Scatter(x=t[step_dt>0], y=step_dt_nz, name='step_dt',
                                       line=dict(color=C[0],width=0.8)), row=1, col=1)
        p99 = float(np.percentile(step_dt_nz, 99))
        fig_diag.add_hline(y=p99, line_dash='dash', line_color='red',
                            annotation_text=f'P99={p99:.1f}ms', row=1, col=1)
    all_cmds = np.concatenate([fc[i][np.abs(fc[i])>0.05] for i in range(4)])
    if len(all_cmds):
        fig_diag.add_trace(go.Histogram(x=all_cmds, nbinsx=60, name='Fin CMD',
                                         marker_color=C[1], opacity=0.8), row=1, col=2)
    for ax_, cc in zip(['x','y','z'], C[:3]):
        fig_diag.add_trace(go.Scatter(x=t, y=col(f'force_{ax_}'), name=f'F_{ax_}',
                                       line=dict(color=cc,width=1)), row=2, col=1)
    fig_diag.add_trace(go.Scatter(x=t, y=alt, name='Alt(m)',
                                   line=dict(color=C[0],width=2)), row=2, col=2)
    fig_diag.add_trace(go.Scatter(x=t, y=speed, name='V(m/s)',
                                   line=dict(color=C[1],dash='dash',width=1.5)), row=2, col=2)
    fig_diag.update_layout(height=640, title_text='SITL Diagnostics — Timing & Forces')
    for r,c_ in [(1,1),(2,1),(2,2)]:
        fig_diag.update_xaxes(title_text='Time (s)', row=r, col=c_)
    fig_diag.update_xaxes(title_text='Deflection (deg)', row=1, col=2)

    # ── Serialize to divs ──────────────────────────────────────────────────
    # First figure gets CDN plotly.js
    divs = {}
    figs = [('traj', fig_traj), ('3d', fig_3d), ('att', fig_att),
            ('aero', fig_aero), ('forces', fig_fd), ('control', fig_ctrl),
            ('struct', fig_ge), ('vel', fig_vel), ('phases', fig_ph),
            ('stab', fig_stab), ('portrait', fig_port), ('fft', fig_fft),
            ('tracking', fig_track), ('mhe', fig_mhe), ('diag', fig_diag)]
    for i, (k, fig) in enumerate(figs):
        divs[k] = _div(fig, first=(i == 0))

    # ── Summary cards ──────────────────────────────────────────────────────
    def mb(v, l, s=None):
        sub = f'<div class="sub">{s}</div>' if s else ''
        return f'<div class="card metric-box"><div class="value">{v}</div><div class="label">{l}</div>{sub}</div>'

    cards = f"""<div class="grid grid-4" style="margin-bottom:16px">
      {mb(f'{t_dur:.1f}s','Duration')}
      {mb(f'{max_alt:.0f}m','Max Altitude')}
      {mb(f'{max_spd:.0f} m/s','Max Speed')}
      {mb(f'{max_mach:.2f}','Max Mach')}
      {mb(f'{max_alpha:.1f}°','Max |AoA|')}
      {mb(f'{max_beta:.1f}°','Max |Sideslip|')}
      {mb(f'{max_G:.1f}g','Max G-Load')}
      {mb(f'{max_om:.0f}°/s','Max ω')}
      {mb(f'{final_rng:.0f}m','Final Range',f'target={target_rng}m' if target_rng else '')}
      {mb(f'{impact_gamma:.1f}°','Impact Angle')}
      {mb(f'{avg_mhq*100:.0f}%' if has_gnc else 'N/A','MHE Quality',f'valid from t={mhe_first_valid:.1f}s' if mhe_first_valid else '')}
      {mb(f'{mpc_first_t:.2f}s' if mpc_first_t else 'N/A','MPC Start')}
      {mb(f'{avg_step:.1f}ms','Avg Step dt',f'P99={p99_step:.1f}ms')}
      {mb(px4.get('timing_mode', metadata.get('timing_mode','lockstep')),'Timing Mode')}
      {mb(f'{px4.get("mpc_N","?")}','MPC Horizon N',f'tf={px4.get("mpc_tf","?")}s')}
      {mb(f'{px4.get("mhe_N","?")}','MHE Horizon N',f'{px4.get("sensor_rate","?")}Hz')}
    </div>"""

    # ── Score table ────────────────────────────────────────────────────────
    cat_rows = ''
    for cat, status, s, tot, detail in cats:
        pct = int(100*s/tot) if tot else 0
        bcls = 'pass' if status == 'PASS' else ('warn' if status == 'WARN' else ('fail' if status == 'FAIL' else 'info'))
        bar_color = 'var(--pass)' if bcls in ('pass','info') else ('var(--warn)' if bcls == 'warn' else 'var(--fail)')
        cat_rows += (
            "<tr>"
            f"<td>{cat}</td>"
            f"<td><span class='badge {bcls}'>{status}</span></td>"
            "<td><div style='background:#eee;border-radius:4px;height:14px'>"
            f"<div style='background:{bar_color};height:100%;width:{pct}%;border-radius:4px'></div></div></td>"
            f"<td style='text-align:right;font-weight:600'>{s}/{tot}</td>"
            f"<td style='font-size:.8rem;color:#666'>{detail}</td>"
            "</tr>"
        )

    # Diagnostics from PX4 log
    diag_html = ''
    for w in px4['warnings'][:8]:
        diag_html += f"<div class='diag warning'><div class='dtitle'>{w[:120]}</div></div>"
    for e in px4['errors'][:5]:
        if 'UXRCE' not in e and 'MAV_1' not in e and 'SER_TEL' not in e and 'XQCAN' not in e:
            diag_html += f"<div class='diag error'><div class='dtitle'>{e[:120]}</div></div>"
    if not diag_html:
        diag_html = "<div class='diag info'><div class='dtitle'>No critical warnings</div></div>"

    ulg_info = (f"<b>ULG:</b> {os.path.basename(ulg_path)}" if ulg_path else
                "<b>ULG:</b> not found (MHE/MPC tab uses CSV fallback)")
    px4_dt = ''
    if px4['dt_lines']:
        px4_dt = px4['dt_lines'][-1].replace('INFO  [rocket_mpc]','').strip()

    mhe_tab_info = (
        '<b>مصادر البيانات (ULG: rocket_gnc_status — حقيقية من PX4):</b><br>'
        + (f'• <b>t_launch</b> = {t_launch:.3f}s — أول لحظة يُكتشف الإطلاق (launched=1)<br>'
           if t_launch is not None else '• <b>t_launch</b> = N/A<br>')
        + (f'• <b>MHE صالح أول مرة</b> = {t_mhe_valid:.3f}s — بعد '
           f'{(t_mhe_valid - (t_launch or 0)):.3f}s من الإطلاق<br>'
           if t_mhe_valid is not None else '• <b>t_mhe_valid</b> = N/A<br>')
        + (f'• <b>MPC نشط أول مرة</b> = {t_mpc_active:.3f}s<br>'
           if t_mpc_active is not None else '• <b>t_mpc_active</b> = N/A<br>')
        + (f'• <b>Blend بدأ</b> = {t_blend_start:.3f}s<br>'
           if t_blend_start is not None and t_blend_start > 0.01 else '• <b>Blend</b> = لم يبدأ (0 طوال الرحلة = MPC بالكامل)<br>')
        + (f'• <b>mhe_quality avg</b> = {avg_mhq*100:.1f}% '
           f'| <b>mpc_solve_count max</b> = {int(gnc_solve_cnt_arr.max())} دورة<br>'
           if has_gnc and len(gnc_solve_cnt_arr) else '')
        + '<hr style="border:0;border-top:1px solid #90caf9;margin:6px 0">'
        '<b>قراءة الرسمات:</b><br>'
        '• ① MHE Valid(×100) = أخضر ← يوضح اللحظة التي أصبح فيها MHE يعطي تقديرات صحيحة | MHE Quality = أزرق<br>'
        '• ① (يمين) محل MHE ومحل MPC: 1=يعمل بنجاح، 0=فشل/تهيؤ<br>'
        '• ② alpha_est (أزرق) vs alpha الحقيقي (أحمر متقطع) — مقارنة مباشرة<br>'
        '• ② blend_alpha — الخط البرتقالي السميك يوضح نسبة LOS في كل لحظة<br>'
        '• ③ أوامر التسارع الزاوية من MPC | x_mpc[0] vs V_measured — يُثبت أن MPC يستقبل بيانات MHE<br>'
        '• ④ أخطاء التتبع γ و χ | متوسط أوامر الزعانف PX4 vs المحاكاة<br>'
        '<b>⚠️ ملاحظة:</b> <em>كل الرسمات مولَّدة من بيانات هذه المحاكاة تحديداً (CSV + ULG مختلفان في كل تشغيل).</em>'
    ) if has_gnc else '<b>⚠️ ULG غير محمّل</b> — يتم عرض بيانات CSV فقط كبديل'

    # ── HTML ───────────────────────────────────────────────────────────────
    css = """
:root{--pass:#4caf50;--warn:#ff9800;--fail:#f44336;--bg:#fafafa;--card:#fff;--border:#e0e0e0;--text:#212121}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,-apple-system,sans-serif;background:var(--bg);color:var(--text);line-height:1.6;padding:20px}
.container{max-width:1400px;margin:0 auto}
h1{font-size:1.8rem;border-bottom:3px solid #1976d2;padding-bottom:8px;margin-bottom:16px}
h2{font-size:1.3rem;color:#1976d2;margin:24px 0 12px;border-left:4px solid #1976d2;padding-left:10px}
.grid{display:grid;gap:12px}.grid-2{grid-template-columns:1fr 1fr}.grid-3{grid-template-columns:1fr 1fr 1fr}.grid-4{grid-template-columns:repeat(4,1fr)}
.card{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:16px;box-shadow:0 1px 3px rgba(0,0,0,.08)}
.score-ring{width:110px;height:110px;border-radius:50%;display:flex;align-items:center;justify-content:center;font-size:2rem;font-weight:700;margin:0 auto 8px;border:6px solid}
.score-ring.pass{border-color:var(--pass);color:var(--pass)}.score-ring.warn{border-color:var(--warn);color:var(--warn)}.score-ring.fail{border-color:var(--fail);color:var(--fail)}
.badge{display:inline-block;padding:2px 10px;border-radius:12px;font-size:.75rem;font-weight:700;color:#fff;text-transform:uppercase}
.badge.pass{background:var(--pass)}.badge.warn{background:var(--warn)}.badge.fail{background:var(--fail)}.badge.info{background:#2196f3}
.metric-box{text-align:center;padding:10px}.metric-box .value{font-size:1.5rem;font-weight:700;color:#1976d2}.metric-box .label{font-size:.72rem;color:#666;text-transform:uppercase;letter-spacing:.4px}.metric-box .sub{font-size:.68rem;color:#999}
table{width:100%;border-collapse:collapse;font-size:.85rem}th{background:#f5f5f5;padding:8px 12px;text-align:left;border-bottom:2px solid var(--border);font-weight:600}td{padding:6px 12px;border-bottom:1px solid var(--border)}tr:hover{background:#f0f7ff}
.diag{padding:10px 14px;border-radius:6px;margin-bottom:8px;border-left:4px solid}
.diag.error{background:#ffebee;border-color:var(--fail)}.diag.warning{background:#fff3e0;border-color:var(--warn)}.diag.info{background:#e3f2fd;border-color:#2196f3}
.diag .dtitle{font-weight:600;font-size:.85rem}
.tabs{display:flex;flex-wrap:wrap;gap:3px;border-bottom:2px solid var(--border);margin-bottom:4px}
.tab-btn{padding:7px 16px;border:none;background:none;cursor:pointer;font-size:.85rem;font-weight:600;border-bottom:3px solid transparent;color:#666;transition:.2s}
.tab-btn:hover{color:#1976d2}.tab-btn.active{color:#1976d2;border-bottom-color:#1976d2}
.tab-panel{display:none;padding:12px 0}.tab-panel.active{display:block}
.info-box{background:#e3f2fd;border:1px solid #90caf9;border-radius:6px;padding:12px;margin-bottom:12px;font-size:.82rem;line-height:1.8}
@media(max-width:900px){.grid-2,.grid-3,.grid-4{grid-template-columns:1fr}}
"""

    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SITL Flight Report — {now}</title>
<style>{css}</style>
</head>
<body>
<div class="container">

<h1>🚀 M130 SITL Flight Report</h1>
<p style="color:#666;margin-bottom:16px;font-size:.9rem">
  Generated {now} &nbsp;·&nbsp;
  Timing: <b>{metadata.get('timing_mode','lockstep')}</b> &nbsp;·&nbsp;
  {n} steps &nbsp;·&nbsp;
  Duration: {t_dur:.1f}s &nbsp;·&nbsp;
  {ulg_info} &nbsp;·&nbsp;
  PX4: {os.path.basename(metadata.get('px4_bin','') or '(external)')} &nbsp;·&nbsp;
  Source: {os.path.basename(csv_path)}
</p>

{cards}

<div class="tabs">
  <button class="tab-btn active"   onclick="openTab(event,'tab-overview')">Overview</button>
  <button class="tab-btn"          onclick="openTab(event,'tab-traj')">Trajectory</button>
  <button class="tab-btn"          onclick="openTab(event,'tab-3d')">3D View</button>
  <button class="tab-btn"          onclick="openTab(event,'tab-att')">Attitude</button>
  <button class="tab-btn"          onclick="openTab(event,'tab-aero')">Aero &amp; Forces</button>
  <button class="tab-btn"          onclick="openTab(event,'tab-forces')">Forces Detail</button>
  <button class="tab-btn"          onclick="openTab(event,'tab-control')">Control</button>
  <button class="tab-btn"          onclick="openTab(event,'tab-struct')">G-Load &amp; Energy</button>
  <button class="tab-btn"          onclick="openTab(event,'tab-vel')">Velocity</button>
  <button class="tab-btn"          onclick="openTab(event,'tab-phases')">Phases</button>
  <button class="tab-btn"          onclick="openTab(event,'tab-stab')">Stability</button>
  <button class="tab-btn"          onclick="openTab(event,'tab-portrait')">Phase Portrait</button>
  <button class="tab-btn"          onclick="openTab(event,'tab-fft')">FFT Spectrum</button>
  <button class="tab-btn"          onclick="openTab(event,'tab-tracking')">Tracking</button>
  <button class="tab-btn" style="color:#7b1fa2;font-weight:700;border-bottom:3px solid #7b1fa2"
                           onclick="openTab(event,'tab-mhe')">🎯 MHE/MPC</button>
  <button class="tab-btn"          onclick="openTab(event,'tab-diag')">SITL Diagnostics</button>
</div>

<!-- ── TAB: Overview ─────────────────────────────────────────────────── -->
<div id="tab-overview" class="tab-panel active">
  <h2>Performance Score</h2>
  <div class="grid grid-2">
    <div class="card" style="text-align:center">
      <div class="score-ring {score_cls}">{score}</div>
      <div style="font-size:1.1rem;font-weight:700;margin-bottom:8px">
        <span class="badge {score_cls}">{'PASS' if score>=80 else 'WARN' if score>=60 else 'FAIL'}</span>
        Overall Score
      </div>
      <table>
        <tr><th>Category</th><th>Status</th><th style="width:110px">Score</th><th></th><th>Detail</th></tr>
        {cat_rows}
      </table>
    </div>
    <div class="card">
      <h3 style="margin-bottom:8px">PX4 Diagnostics</h3>
      {diag_html}
      <div style="margin-top:12px;font-size:.8rem;color:#666">
        <b>PX4 Loop:</b> {px4_dt or 'N/A'}<br>
        <b>Launch detected:</b> {'✅' if px4['launch_detected'] else '❌'}<br>
        <b>MHE init:</b> {'✅' if px4['mhe_init'] else '❌'} &nbsp;
        <b>MPC init:</b> {'✅' if px4['mpc_init'] else '❌'}<br>
        <b>First MPC cycle:</b> t={px4.get('first_mpc_t','?')}s &nbsp;
        <b>Thrust:</b> {px4.get('thrust','?')} N<br>
        <b>Origin:</b> lat={px4.get('origin_lat','?')} lon={px4.get('origin_lon','?')}
      </div>
    </div>
  </div>
</div>

<!-- ── TAB: Trajectory ─────────────────────────────────────────────────── -->
<div id="tab-traj" class="tab-panel">
  <h2>Trajectory</h2>
  <div>{divs['traj']}</div>
</div>

<!-- ── TAB: 3D View ────────────────────────────────────────────────────── -->
<div id="tab-3d" class="tab-panel">
  <h2>3D Flight Path</h2>
  <div>{divs['3d']}</div>
</div>

<!-- ── TAB: Attitude ───────────────────────────────────────────────────── -->
<div id="tab-att" class="tab-panel">
  <h2>Attitude &amp; Angular Motion</h2>
  <div>{divs['att']}</div>
</div>

<!-- ── TAB: Aero & Forces ──────────────────────────────────────────────── -->
<div id="tab-aero" class="tab-panel">
  <h2>Aerodynamics &amp; Forces</h2>
  <div>{divs['aero']}</div>
</div>

<!-- ── TAB: Forces Detail ──────────────────────────────────────────────── -->
<div id="tab-forces" class="tab-panel">
  <h2>Forces — Detailed View</h2>
  <div>{divs['forces']}</div>
</div>

<!-- ── TAB: Control ────────────────────────────────────────────────────── -->
<div id="tab-control" class="tab-panel">
  <h2>Fin Control — CMD vs ACT</h2>
  <p style="font-size:.85rem;color:#555;margin-bottom:8px">
    CMD = received from PX4 HIL_ACTUATOR_CONTROLS · ACT = applied by 6DOF physics engine
  </p>
  <div>{divs['control']}</div>
</div>

<!-- ── TAB: G-Load & Energy ────────────────────────────────────────────── -->
<div id="tab-struct" class="tab-panel">
  <h2>G-Load &amp; Energy</h2>
  <div>{divs['struct']}</div>
</div>

<!-- ── TAB: Velocity ───────────────────────────────────────────────────── -->
<div id="tab-vel" class="tab-panel">
  <h2>Velocity Analysis</h2>
  <div>{divs['vel']}</div>
</div>

<!-- ── TAB: Phases ─────────────────────────────────────────────────────── -->
<div id="tab-phases" class="tab-panel">
  <h2>Flight Phases</h2>
  <div>{divs['phases']}</div>
</div>

<!-- ── TAB: Stability ──────────────────────────────────────────────────── -->
<div id="tab-stab" class="tab-panel">
  <h2>Stability Analysis</h2>
  <div>{divs['stab']}</div>
</div>

<!-- ── TAB: Phase Portrait ─────────────────────────────────────────────── -->
<div id="tab-portrait" class="tab-panel">
  <h2>Phase Portraits</h2>
  <div>{divs['portrait']}</div>
</div>

<!-- ── TAB: FFT ────────────────────────────────────────────────────────── -->
<div id="tab-fft" class="tab-panel">
  <h2>FFT Frequency Spectrum</h2>
  <div>{divs['fft']}</div>
</div>

<!-- ── TAB: Tracking ───────────────────────────────────────────────────── -->
<div id="tab-tracking" class="tab-panel">
  <h2>Guidance Tracking Performance</h2>
  <div>{divs['tracking']}</div>
</div>

<!-- ── TAB: MHE/MPC  [NEW] ─────────────────────────────────────────────── -->
<div id="tab-mhe" class="tab-panel">
  <h2 style="color:#7b1fa2;border-left-color:#7b1fa2">🎯 MHE / MPC Analysis</h2>
  <div class="info-box">{mhe_tab_info}</div>
  <div>{divs['mhe']}</div>
</div>

<!-- ── TAB: SITL Diagnostics ───────────────────────────────────────────── -->
<div id="tab-diag" class="tab-panel">
  <h2>SITL Diagnostics</h2>
  <p style="font-size:.85rem;color:#555;margin-bottom:8px">
    Step wall-clock = time each lockstep took in real time (ms).
    High P99 = PX4 ACADOS solver was slow at that step.
  </p>
  <div>{divs['diag']}</div>
  <div class="card" style="margin-top:16px;font-family:monospace;font-size:.78rem;color:#444;white-space:pre-wrap;overflow-x:auto">
{json.dumps({k:v for k,v in {
    'generated_at': now,
    'csv': os.path.basename(csv_path),
    'ulg': os.path.basename(ulg_path) if ulg_path else None,
    'duration_s': t_dur, 'n_steps': n,
    'max_alt_m': max_alt, 'max_speed_ms': max_spd, 'max_mach': max_mach,
    'max_alpha_deg': max_alpha, 'max_beta_deg': max_beta, 'max_G': max_G,
    'final_range_m': final_rng, 'target_range_m': target_rng,
    'impact_gamma_deg': impact_gamma,
    'mpc_first_t': mpc_first_t, 'mhe_first_valid_t': mhe_first_valid,
    'avg_mhe_quality': round(avg_mhq, 3),
    'avg_step_dt_ms': round(avg_step, 2), 'p99_step_dt_ms': round(p99_step, 2),
    'max_fin_err_deg': round(max_fin_err, 3),
    'score': score, 'timing_mode': metadata.get('timing_mode','lockstep'),
    'mpc_N': px4.get('mpc_N'), 'mhe_N': px4.get('mhe_N'),
    'px4_launch_detected': px4['launch_detected'],
}.items()}, indent=2, default=str)}
  </div>
</div>

</div><!-- /container -->
<script>
function openTab(evt, tabId) {{
  document.querySelectorAll('.tab-panel').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
  document.getElementById(tabId).classList.add('active');
  if (evt) evt.currentTarget.classList.add('active');
  var plots = document.getElementById(tabId).querySelectorAll('.js-plotly-plot');
  plots.forEach(function(p) {{ Plotly.Plots.resize(p); }});
}}
</script>
</body>
</html>"""

    os.makedirs(os.path.dirname(os.path.abspath(html_path)) or '.', exist_ok=True)
    with open(html_path, 'w', encoding='utf-8') as f:
        f.write(html)

    if auto_open:
        try:
            webbrowser.open(f'file://{os.path.abspath(html_path)}')
        except Exception:
            pass

    return html_path


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    import sys, argparse
    ap = argparse.ArgumentParser(description='Generate SITL HTML report from CSV (+ULG)')
    ap.add_argument('csv', help='CSV file from SITL run')
    ap.add_argument('--html', help='Output HTML path (default: csv.replace .csv with _report.html)')
    ap.add_argument('--ulg',  help='PX4 ULG log file (auto-detected if omitted)')
    ap.add_argument('--px4-log', help='PX4 stdout log (px4_stdout.log)')
    ap.add_argument('--no-open', action='store_true', help='Do not open browser')
    args = ap.parse_args()
    html_p = args.html or args.csv.replace('.csv','_report.html')
    out = generate_sitl_html_report(
        args.csv, html_p,
        ulg_path=args.ulg,
        px4_log_path=args.px4_log,
        auto_open=not args.no_open,
    )
    print(f"✓ Report: {out}")
