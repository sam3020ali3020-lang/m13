#!/usr/bin/env python3
"""
SITL Runner — launches PX4 SITL and drives it through the 6DOF MAVLink bridge.
سكربت تشغيل SITL — يشغّل PX4 ويقوده عبر جسر MAVLink + محاكاة 6DOF.

Produces a single flight log: results/sitl_flight.csv (or user-chosen path).

Usage:
    python run_sitl_test.py --px4-bin /path/to/px4        # launch PX4 ourselves
    python run_sitl_test.py                               # PX4 already running
    python run_sitl_test.py --sim-config my_cfg.yaml
"""

import sys
import os
import time
import argparse
import subprocess
import signal
from datetime import datetime
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
_SIM_DIR = _SCRIPT_DIR.parent
_RESULTS_DIR = _SCRIPT_DIR / 'results'

sys.path.insert(0, str(_SIM_DIR))


def run_sitl(px4_bin: str = None, sitl_config: str = None,
             sim_config: str = None, output_csv: str = None,
             port: int = 4560) -> str:
    """Launch PX4 (optional) and drive it through the 6DOF MAVLink bridge.

    Returns path to the CSV flight log.
    """
    print("=" * 70)
    print("  SITL: PX4 rocket_mpc + 6DOF MAVLink bridge")
    print("=" * 70)

    csv_path = output_csv or str(_RESULTS_DIR / 'sitl_flight.csv')
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)

    sitl_cfg = sitl_config or str(_SCRIPT_DIR / 'sitl_config.yaml')

    px4_proc = None
    px4_log_file = None
    px4_log_path = None

    if px4_bin and os.path.isfile(px4_bin):
        # Launch PX4 binary in background
        print(f"  Starting PX4: {px4_bin}")
        px4_env = os.environ.copy()
        px4_env['PX4_SYS_AUTOSTART'] = '22003'  # SITL airframe
        px4_env['PX4_SIM_MODEL'] = 'none'

        # PX4 SITL needs LD_LIBRARY_PATH for acados shared libs.
        # Resolution order:
        #   1. ACADOS_LIB env var (explicit override)
        #   2. <repo_root>/acados-main/lib (standard layout: _SIM_DIR.parent)
        #   3. existing LD_LIBRARY_PATH (unmodified)
        _repo_root = _SIM_DIR.parent
        acados_lib_candidates = [
            os.environ.get('ACADOS_LIB'),
            str(_repo_root / 'acados-main' / 'lib'),
        ]
        acados_lib = next((p for p in acados_lib_candidates
                           if p and os.path.isdir(p)), None)
        if acados_lib:
            px4_env['LD_LIBRARY_PATH'] = acados_lib + ':' + px4_env.get('LD_LIBRARY_PATH', '')

        # PX4 SITL working directory must be build/px4_sitl_default
        px4_build_dir = os.path.dirname(os.path.dirname(px4_bin))
        px4_cwd = px4_build_dir if os.path.isdir(os.path.join(px4_build_dir, 'etc')) else None

        # Clean stale parameters so PX4 re-applies airframe 22003 config
        if px4_cwd:
            for pfile in ('parameters.bson', 'parameters_backup.bson'):
                p = os.path.join(px4_cwd, pfile)
                if os.path.exists(p):
                    os.remove(p)

        # PX4 stdout log file — CRITICAL: do NOT use subprocess.PIPE without
        # reading it.  The 64KB pipe buffer fills up, PX4 blocks on write,
        # and the sim_rcv/sim_send threads deadlock → TCP dies → BrokenPipe.
        px4_log_path = os.path.join(str(_RESULTS_DIR), 'px4_stdout.log')
        os.makedirs(os.path.dirname(px4_log_path), exist_ok=True)
        px4_log_file = open(px4_log_path, 'w', encoding='utf-8')

        px4_proc = subprocess.Popen(
            [px4_bin, '-s', 'etc/init.d-posix/rcS', '-d'],
            env=px4_env,
            cwd=px4_cwd,
            stdout=px4_log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )
        print(f"  PX4 started (PID={px4_proc.pid}), cwd={px4_cwd}")
        print(f"  PX4 stdout → {px4_log_path}")
        print(f"  Waiting 5s for PX4 TCP init...")
        time.sleep(5)
    else:
        print("  No PX4 binary specified — assuming PX4 is already running")
        print(f"  (If not, start PX4 SITL manually and connect to port {port})")

    try:
        # Run bridge
        from mavlink_bridge import SITLBridge
        bridge = SITLBridge(sitl_cfg, sim_config)
        bridge.port = port
        bridge.run(csv_output=csv_path)
    finally:
        if px4_proc:
            retcode = px4_proc.poll()
            if retcode is not None:
                print(f"  PX4 already exited with code {retcode} "
                      f"(signal {-retcode if retcode < 0 else 'N/A'})")
            else:
                print("  Stopping PX4...")
                os.killpg(os.getpgid(px4_proc.pid), signal.SIGTERM)
                px4_proc.wait(timeout=5)
            if px4_log_file:
                px4_log_file.close()
            # Print last 80 lines of PX4 output for diagnostics
            if px4_log_path:
                print(f"\n  --- PX4 stdout (last 80 lines from {px4_log_path}) ---")
                try:
                    with open(px4_log_path, 'r', encoding='utf-8', errors='replace') as f:
                        lines = f.readlines()
                    for line in lines[-80:]:
                        print(f"  PX4| {line.rstrip()}")
                except OSError as e:
                    print(f"  (could not read PX4 log: {e})")
                print(f"  --- end PX4 stdout ---\n")

    print(f"  Saved: {csv_path}")
    return csv_path


def main():
    parser = argparse.ArgumentParser(
        description='M130 SITL runner — PX4 rocket_mpc + 6DOF MAVLink bridge')
    parser.add_argument('--sim-config', type=str, default=None,
                        help='Simulation config YAML (default: config/6dof_config_advanced.yaml)')
    parser.add_argument('--sitl-config', type=str, default=None,
                        help='SITL bridge config YAML (deprecated — kept for back-compat)')
    parser.add_argument('--px4-bin', type=str, default=None,
                        help='Path to PX4 SITL binary (if omitted, PX4 must be started externally)')
    parser.add_argument('--port', type=int, default=4560,
                        help='MAVLink bridge TCP port (default: 4560)')
    parser.add_argument('--output-csv', type=str, default=None,
                        help='Output CSV path (default: results/sitl_<timestamp>.csv)')
    parser.add_argument('--output-dir', type=str, default=None,
                        help='Output directory (default: results/)')
    args = parser.parse_args()

    results_dir = args.output_dir or str(_RESULTS_DIR)
    os.makedirs(results_dir, exist_ok=True)

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    sitl_csv = args.output_csv or os.path.join(results_dir, f'sitl_{timestamp}.csv')

    print()
    print("╔══════════════════════════════════════════════════════╗")
    print("║       M130 SITL RUN                                 ║")
    print("║       PX4 rocket_mpc via MAVLink HIL                ║")
    print("╚══════════════════════════════════════════════════════╝")
    print()

    run_sitl(args.px4_bin, args.sitl_config, args.sim_config, sitl_csv, args.port)

    # Generate interactive HTML report
    try:
        import importlib.util
        _rpt_path = os.path.join(os.path.dirname(__file__), 'sitl_html_report.py')
        _spec = importlib.util.spec_from_file_location('sitl_html_report', _rpt_path)
        _mod  = importlib.util.module_from_spec(_spec)
        _spec.loader.exec_module(_mod)
        html_path = sitl_csv.replace('.csv', '_report.html')
        # Locate PX4 stdout log and ULG
        _px4_log = os.path.join(str(_RESULTS_DIR), 'px4_stdout.log')
        _px4_log = _px4_log if os.path.isfile(_px4_log) else None
        _ulg = _mod._find_ulg(args.px4_bin or '')
        print(f"\n  Generating HTML report...")
        if _ulg: print(f"  ULG: {_ulg}")
        _mod.generate_sitl_html_report(
            sitl_csv, html_path,
            metadata={'timing_mode': 'lockstep', 'px4_bin': args.px4_bin or ''},
            px4_log_path=_px4_log,
            ulg_path=_ulg,
            auto_open=True,
        )
        print(f"  ✓ HTML Report: {html_path}")
    except Exception as _e:
        import traceback
        print(f"\n  [WARNING] Could not generate HTML report: {_e}")
        traceback.print_exc()


if __name__ == '__main__':
    main()
