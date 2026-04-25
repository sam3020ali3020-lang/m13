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
 * @file rocket_mpc_params.c
 * Parameters for Rocket M130 MPC+MHE control module.
 *
 * Mass/propulsion/inertia values must match the acados OCP model
 * used for code generation.
 */

#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

/* ===================================================================
 *  Timing
 * =================================================================== */

/**
 * Control activation delay after launch
 *
 * Time after launch detection before MPC starts commanding fins.
 * Should cover the launcher rail departure time.
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_T_CTRL, 0.3f);

/* ===================================================================
 *  Target
 * =================================================================== */

/**
 * Target downrange distance
 *
 * Distance along the bearing captured at arming.
 *
 * @unit m
 * @min 100.0
 * @max 300000.0
 * @decimal 0
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_XTRGT, 2600.0f);

/**
 * Target altitude AGL
 *
 * Target altitude above launch site (target_alt - launch_alt).
 * 0 = same elevation as launch site.
 *
 * @unit m
 * @min -5000.0
 * @max 50000.0
 * @decimal 0
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_HTRGT, 0.0f);

/**
 * Terminal impact angle
 *
 * Desired flight-path angle at impact. Negative = diving.
 *
 * @unit deg
 * @min -90.0
 * @max 0.0
 * @decimal 1
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_IMP_ANG, -30.0f);

/**
 * Cruise→dive transition progress
 *
 * Fraction of downrange-to-target at which the LOS guidance starts
 * blending from level cruise flight (γ=0) to the impact-angle dive
 * trajectory. The dive is fully active at progress = min(p+0.10, 0.95),
 * with a smooth Hermite blend in between.
 *
 * Lower values start the dive earlier (good for steep impact angles
 * and longer-range targets); higher values keep the vehicle in cruise
 * longer (good for shallow impact angles or shorter ranges).
 *
 * Must match autopilot.mpc.cruise_progress in the Python simulation
 * config to keep sim-vs-flight guidance timing consistent.
 *
 * @min 0.3
 * @max 0.95
 * @decimal 2
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_CRUISE_P, 0.65f);

/* ===================================================================
 *  Fin limits
 *
 *  NOTE: The maximum fin deflection is NOT a parameter — it is baked
 *  into the acados-generated solver at 0.3491 rad (20°). See
 *  m130_ocp_setup.py::delta_max and SOLVER_DELTA_MAX_RAD in RocketMPC.cpp.
 *  To change the limit, modify the Python OCP and regenerate the solver.
 * =================================================================== */

/* ===================================================================
 *  Mass / propulsion  (must match acados OCP model)
 * =================================================================== */

/**
 * Full (wet) mass at ignition
 *
 * @unit kg
 * @min 0.5
 * @max 10000.0
 * @decimal 3
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_MASS_F, 12.74f);

/**
 * Dry mass (burnout)
 *
 * @unit kg
 * @min 0.5
 * @max 10000.0
 * @decimal 3
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_MASS_D, 11.11f);

/**
 * Motor burn time
 *
 * @unit s
 * @min 0.1
 * @max 600.0
 * @decimal 3
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_TBURN, 4.772f);

/**
 * Total impulse
 *
 * @min 1.0
 * @max 10000000.0
 * @decimal 1
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_IMPULS, 3593.2f);

/**
 * Thrust plateau (manual override, advanced)
 *
 * Steady-state thrust during the propulsive phase.
 *
 * Leave at 0 (default) to auto-derive from impulse, burn time and tail-off:
 *     T_plateau = ROCKET_IMPULS / (ROCKET_TBURN - 0.75 * ROCKET_T_TAIL)
 * This keeps the plateau in lock-step with the propulsion parameter set and
 * matches the Python simulation reference.
 *
 * Any positive value is treated as an advanced override and bypasses the
 * derivation — use only when characterising a motor whose profile is not
 * captured by the impulse/burn-time/tail-off triplet. Mismatching the
 * override with the rest of the propulsion params will silently skew MPC
 * gamma tracking during boost.
 *
 * @unit N
 * @min 0.0
 * @max 5000000.0
 * @decimal 1
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_THRUST, 0.0f);

/**
 * Tail-off duration
 *
 * Time for thrust to ramp from plateau to zero.
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_T_TAIL, 1.0f);

/* ===================================================================
 *  Inertias  (must match acados OCP model)
 * =================================================================== */

/**
 * Roll inertia at full mass (Ixx)
 *
 * @unit kg m^2
 * @min 0.001
 * @max 100000.0
 * @decimal 4
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_IXX_F, 0.0389f);

/**
 * Roll inertia at dry mass (Ixx)
 *
 * @unit kg m^2
 * @min 0.001
 * @max 100000.0
 * @decimal 4
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_IXX_D, 0.0356f);

/**
 * Pitch inertia at full mass (Iyy)
 *
 * @unit kg m^2
 * @min 0.001
 * @max 100000.0
 * @decimal 4
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_IYY_F, 1.1651f);

/**
 * Pitch inertia at dry mass (Iyy)
 *
 * @unit kg m^2
 * @min 0.001
 * @max 100000.0
 * @decimal 4
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_IYY_D, 1.0789f);

/**
 * Yaw inertia at full mass (Izz)
 *
 * @unit kg m^2
 * @min 0.001
 * @max 100000.0
 * @decimal 4
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_IZZ_F, 1.166f);

/**
 * Yaw inertia at dry mass (Izz)
 *
 * @unit kg m^2
 * @min 0.001
 * @max 100000.0
 * @decimal 4
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_IZZ_D, 1.0779f);

/* ===================================================================
 *  Servo dynamics
 *
 *  NOTE: The servo time constant is NOT a parameter — it is baked
 *  into the acados-generated solver at 0.015 s (Qabthah1 KST X20-7.4
 *  electric servo). See m130_acados_model.py::tau_servo_val and
 *  SOLVER_TAU_SERVO_S in RocketMPC.cpp. Changing it requires
 *  regenerating the solver AND updating SOLVER_TAU_SERVO_S so the
 *  forward-guess warm start and the MHE lag filter stay consistent
 *  with the solver's internal servo model.
 * =================================================================== */

/* ===================================================================
 *  Launch site
 *
 *  Launch-site altitude and rail elevation are NOT parameters: they are
 *  captured directly from the sensor suite at arming / pre-launch:
 *    - altitude ASL  -> GPS 3D fix (real flight) / lpos.ref_alt (HITL),
 *                       with baro fallback at launch detection
 *    - rail pitch    -> attitude quaternion at arm + every pre-launch
 *                       cycle; see RocketMPC::_pitch_from_quat().
 *  Any "set once in params" design was a porting leftover from the
 *  simulator config and was the root cause of two silent-failure bugs
 *  (1150 m launch_alt mismatch saturating MHE's h state; wrong rail
 *  angle skewing LOS gamma_natural feedforward).
 * =================================================================== */

/* ===================================================================
 *  MHE quality gate
 * =================================================================== */

/**
 * MHE quality gate threshold
 *
 * Minimum MHE quality metric before MHE state is used for MPC.
 * Below this, fallback to EKF2/sensor-derived state.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_MHE_QG, 0.3f);

/* ===================================================================
 *  MPC solver tuning
 * =================================================================== */

/**
 * MPC prediction horizon time
 *
 * @unit s
 * @min 1.0
 * @max 20.0
 * @decimal 1
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_MPC_TF, 4.0f);

/* ===================================================================
 *  Test mode
 * =================================================================== */

/* ===================================================================
 *  SITL workarounds
 * =================================================================== */

/**
 * SITL GPS fallback for MHE
 *
 * Use EKF2 local position as GPS substitute when raw sensor_gps
 * is unavailable (SITL circular dependency workaround).
 * ONLY enable in simulation — never on real hardware.
 *
 * 0 = disabled (real hardware default).
 * 1 = enabled (SITL only).
 *
 * @min 0
 * @max 1
 * @group Rocket MPC
 */
PARAM_DEFINE_INT32(ROCKET_SITL_GPS, 0);

/* ===================================================================
 *  CAN servo feedback path
 * =================================================================== */

/**
 * Use CAN SRV_FB as the source of truth for _de_act/_dr_act/_da_act
 *
 * 0 = disabled (legacy first-order-lag filter on the commanded deflections,
 *     with tau matching the acados solver's internal tau_servo). This path
 *     is the ONLY safe option for SITL (22003) because SITL has no CAN bus
 *     and leaving the feedback path enabled would freeze _de_act at 0 and
 *     bias the MHE aerodynamic model.
 *
 * 1 = enabled (HITL / real flight). RocketMPC reads SRV_FB (debug_array
 *     id=1, name="SRV_FB") published by drivers/xqpower_can, checks the
 *     per-servo age stamped in data[14..17] (ms) against a fresh-threshold
 *     of 120 ms, back-solves de/dr/da from the four physical fin positions
 *     (least-squares inverse of the X-fin mixer), rate-limits the feedback
 *     to 400°/s, and holds the last valid (de,dr,da) when ≥2 servos are
 *     stale. The existing first-order lag filter is skipped in this mode.
 *
 *   Airframes:
 *     22003 SITL                 -> 0 (default, enforced by omission)
 *     22004 HITL                 -> 1 (set via param set-default in airframe)
 *     22005 Real flight          -> 1 (set via param set-default in airframe)
 *
 * Telemetry: rocket_gnc_status.{can_stale_us, can_abort_events,
 *            first_fb_received, valid_mask} reflect the feedback path.
 *
 * @min 0
 * @max 1
 * @group Rocket MPC
 */
PARAM_DEFINE_INT32(ROCKET_SRV_FB, 0);
