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

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>
#include <matrix/math.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/rocket_gnc_status.h>
#include <uORB/topics/debug_array.h>
// NOTE: debug_vect is no longer used by this module. Cycle timing moved
// to rocket_gnc_status.{mhe_solve_us, mpc_solve_us, cycle_us} to avoid
// the shared debug_vect uORB instance colliding with mavlink_receiver's
// inbound DEBUG_VECT republishing.
#include <systemlib/mavlink_log.h>

#include "mpc_controller.h"
#include "mhe_estimator.h"
#include "los_guidance.h"
#include "sensor_bridge.h"

using namespace time_literals;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Dcmf;
using matrix::Vector3f;

#define MODULE_NAME "rocket_mpc"

class RocketMPC : public ModuleBase<RocketMPC>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	RocketMPC();
	~RocketMPC() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;
	void parameters_update(bool force = false);

	// Reset all in-flight state (_launched, actuator caches, solver
	// sub-modules, MHE publishing snapshot, xval). Invoked on both
	// arm→launch transitions and disarm to prevent stale state from
	// one flight bleeding into the next and to stop the MPC/actuator
	// pipeline when the vehicle is disarmed.
	void _reset_flight_state();

	// Extract bearing (heading of body-X projected onto NED horizontal plane)
	// directly from the quaternion. This is numerically stable near pitch=±90°
	// where Eulerf::psi() loses meaning (gimbal lock). When body-X is nearly
	// vertical (|horizontal projection| < 1e-2) the bearing is undefined and
	// `fallback` is returned instead.
	static float _bearing_from_quat(const Quatf &q, float fallback);

	// Extract launch pitch (elevation of body-X above the NED horizontal
	// plane) directly from the DCM column of the quaternion.  Numerically
	// stable through pitch = ±90° (gimbal lock), which is exactly where
	// a near-vertical rocket operates; Eulerf::theta() is not.
	static float _pitch_from_quat(const Quatf &q);

	// ---------------------------------------------------------------
	// Sub-systems
	// ---------------------------------------------------------------
	MpcController  _mpc;
	MheEstimator   _mhe;
	LosGuidance    _los;
	SensorBridge   _sensor;

	// ---------------------------------------------------------------
	// uORB Subscriptions
	// ---------------------------------------------------------------
	uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps)};   /* RAW GPS — independent of EKF2 for MHE redundancy */
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _debug_array_sub{ORB_ID(debug_array)};

	// ---------------------------------------------------------------
	// uORB Publications
	// ---------------------------------------------------------------
	uORB::Publication<actuator_outputs_s>  _actuator_outputs_sim_pub{ORB_ID(actuator_outputs_sim)};
	uORB::Publication<actuator_servos_s>   _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<rocket_gnc_status_s> _rocket_gnc_status_pub{ORB_ID(rocket_gnc_status)};


	// ---------------------------------------------------------------
	// Cycle-timing snapshot (populated once per Run(), consumed by the
	// rocket_gnc_status publisher below). Replaces the former debug_vect
	// "TIMING" publication: a dedicated snapshot inside our own topic is
	// immune to mavlink_receiver's inbound DEBUG_VECT injection and is
	// logged by default without requiring add_debug_topics().
	uint32_t _mhe_solve_us{0};
	uint32_t _mpc_solve_us{0};
	uint32_t _cycle_us{0};

	// ---------------------------------------------------------------
	// State variables
	// ---------------------------------------------------------------
	hrt_abstime _time_launch{0};
	bool        _launched{false};
	float       _launch_dv{0.0f};

	bool  _armed{false};
	float _arm_origin_x{0.0f};
	float _arm_origin_y{0.0f};
	float _arm_origin_z{0.0f};

	float _cos_bearing{1.0f};
	float _sin_bearing{0.0f};
	float _target_downrange{0.0f};

	// Launch-time state captured from sensors at arming (replaces the old
	// ROCKET_L_ALT / ROCKET_L_PITCH params, which were error-prone user
	// inputs that had to be updated by hand for every flight):
	//   _actual_launch_alt_msl — MSL altitude, from GPS (arm / pre-launch)
	//                            or baro (launch-detection fallback)
	//   _actual_launch_pitch_rad — body-X elevation above horizon (rail
	//                              angle), from attitude at arm + every
	//                              pre-launch cycle, used to recompute
	//                              gamma_natural for LOS feedforward
	float _actual_launch_alt_msl{0.0f};
	bool  _launch_alt_captured{false};
	float _actual_launch_pitch_rad{0.0f};
	bool  _launch_pitch_captured{false};

	bool  _hitl{false};  // HITL mode: use lpos reference frame for MHE (avoid GPS origin mismatch)

	// Cached fin commands for lockstep republish
	float _last_fins[4] {0.0f, 0.0f, 0.0f, 0.0f};
	float _last_de{0.0f}, _last_dr{0.0f}, _last_da{0.0f};
	hrt_abstime _last_mpc_solve_time{0};

	// ── Cumulative per-flight event counters (surfaced in rocket_gnc_status
	//    so transient events are not lost between MAVLink DEBUG_FLOAT_ARRAY
	//    samples, which fire at 20 Hz vs. the 100 Hz+ publish rate.) All
	//    reset in _reset_flight_state.
	uint32_t _mpc_fail_count{0};
	uint32_t _mpc_nan_skip_count{0};
	uint32_t _mhe_fail_count{0};
	uint32_t _fin_clamp_total{0};     // solves where any fin clamped (edge counter)
	uint32_t _xval_reset_count{0};
	uint32_t _servo_offline_events{0};

	// Last servo online mask observed from xqpower_can feedback (SRV_FB),
	// used to detect online→offline edge transitions for telemetry.
	uint8_t _prev_servo_online_mask{0};
	bool    _prev_servo_mask_valid{false};

	// Latched GPS quality snapshot (mirrored into rocket_gnc_status).
	uint8_t _gps_fix_type{0};
	uint8_t _gps_sats_used{0};
	uint8_t _gps_jamming_state{0};

	// ── Per-fin saturation-clamp telemetry ──
	// The acados solver's polytopic constraint limits the MIXED fins only on
	// the *actual* servo state (delta_*_act in x[15..17]). The published fin
	// command, however, is derived from the *commanded* servo state
	// (delta_*_s from x[12..14]), which has only individual ±delta_max box
	// bounds — no polytopic coupling. So during aggressive transients, the
	// mixed value (delta_a ± delta_e ± delta_r) can legally exceed delta_max
	// even when every virtual channel is in-box and the solver succeeded.
	//
	// The per-fin math::constrain() in RocketMPC.cpp is therefore NOT just a
	// safety net for SQP_RTI failures — it is the actual enforcement of the
	// polytopic fin-mix bound on the commanded path. These counters measure
	// how often that enforcement actually fires; a persistently non-zero
	// rate is diagnostic and argues for adding polytopic on delta_*_s in
	// m130_ocp_setup.py (requires regenerating the solver).
	uint32_t _fin_clamp_count[4] {};   // per-fin activation count
	uint32_t _fin_clamp_any{0};        // solves with >=1 fin clamped
	uint32_t _fin_clamp_solves{0};     // valid-solve denominator
	uint32_t _fin_clamp_report_at{0};  // next solve index at which to print

	// Tracked actual fin deflections.  Source depends on ROCKET_USE_SRV_FB:
	//   0  -> first-order lag filter on commanded deflections (tau matches
	//         the acados solver's baked-in tau_servo, used in SITL only).
	//   1  -> CAN SRV_FB back-solve (HITL / real flight). See the feedback
	//         block in RocketMPC::Run() and the state variables below.
	float _de_act{0.0f}, _dr_act{0.0f}, _da_act{0.0f};

	// ── CAN feedback path state (used when ROCKET_USE_SRV_FB=1) ──
	// The per-servo last-known feedback angles are retained so that a brief
	// dropout of a single servo can be patched with its last known value and
	// the back-solve still produces the correct de/dr/da triplet.  Ages are
	// re-derived from SRV_FB.data[14..17] (ms since last CAN frame) every
	// time a new SRV_FB message is copied, so _last_fb_per_servo_rad stays
	// consistent with the freshness mask.
	float       _last_fb_per_servo_rad[4] {0.0f, 0.0f, 0.0f, 0.0f};
	bool        _last_fb_per_servo_valid[4] {false, false, false, false};
	hrt_abstime _last_fb_time{0};          // timestamp of last successful update
	bool        _first_fb_received{false}; // latched once a valid frame lands
	uint32_t    _can_stale_us{0};          // microseconds since _last_fb_time
	uint32_t    _can_abort_events{0};      // count of >500 ms stale entries
	bool        _can_abort_warned{false};  // one-shot mavlink warning latch
	uint8_t     _can_valid_mask{0};        // mask of servos used last cycle

	// Freshness / safety thresholds for the feedback path.
	static constexpr hrt_abstime CAN_FRESH_US     = 120_ms;
	static constexpr hrt_abstime CAN_ABORT_US     = 500_ms;
	static constexpr float       MAX_FB_RATE_DPS  = 400.0f;

	// Latest full MPC state vector (x0 passed to the acados solver).  Cached so
	// rocket_gnc_status can publish x_mpc[18] on every cycle, including ticks
	// that reuse the previous fin command without re-solving.
	float _last_x_mpc[18] {};
	bool  _have_x_mpc{false};

	// Latest MPC solver diagnostics
	int      _last_mpc_status{-1};
	uint32_t _last_mpc_sqp_iter{0};

	// Latest MHE diagnostics
	int   _last_mhe_status{-1};
	bool  _last_mhe_valid{false};

	// MHE-derived state cache (for logging & publish)
	bool  _mhe_publishing{false};
	float _mhe_phi{0.0f}, _mhe_theta{0.0f}, _mhe_psi{0.0f};
	float _mhe_x{0.0f}, _mhe_y{0.0f}, _mhe_z{0.0f};
	float _mhe_vx{0.0f}, _mhe_vy{0.0f}, _mhe_vz{0.0f};
	float _mhe_vm{0.0f};

	// EKF-MHE cooperative blend state
	float _blend_alpha{0.0f};  // current blend factor 0=EKF, 1=MHE

	// Cross-validation: EKF↔MHE consistency monitor
	struct {
		int   consec_diverge{0};       // consecutive cycles where MHE diverges from EKF
		float diverge_start_t{0.0f};   // flight time when divergence streak began
		float penalty{0.0f};           // cross-validation penalty applied to blend [0..1]
		float gamma_err{0.0f};         // last |gamma_mhe - gamma_ekf|
		float chi_err{0.0f};           // last |chi_mhe - chi_ekf|
		float alt_err{0.0f};           // last |alt_mhe - alt_ekf|
	} _xval;

	// Baro staleness one-shot warning (reset on each flight)
	bool _baro_stale_warned{false};

	// dt measurement
	hrt_abstime _prev_run_time{0};
	float       _dt_measured{0.02f};
	float       _dt_min{1.0f};
	float       _dt_max{0.0f};
	float       _dt_sum{0.0f};
	uint32_t    _dt_count{0};

	// Performance counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _mpc_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": mpc_solve")};
	perf_counter_t _mhe_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": mhe_solve")};
	orb_advert_t   _mavlink_log_pub{nullptr};

	// ---------------------------------------------------------------
	// Parameters
	// ---------------------------------------------------------------
	DEFINE_PARAMETERS(
		// Timing
		(ParamFloat<px4::params::ROCKET_T_CTRL>)    _param_t_ctrl,

		// Target
		(ParamFloat<px4::params::ROCKET_XTRGT>)     _param_xtrgt,
		(ParamFloat<px4::params::ROCKET_HTRGT>)     _param_htrgt,
		(ParamFloat<px4::params::ROCKET_IMP_ANG>)   _param_imp_ang,
		(ParamFloat<px4::params::ROCKET_CRUISE_P>)  _param_cruise_p,

		// Mass / propulsion
		(ParamFloat<px4::params::ROCKET_MASS_F>)    _param_mass_full,
		(ParamFloat<px4::params::ROCKET_MASS_D>)    _param_mass_dry,
		(ParamFloat<px4::params::ROCKET_TBURN>)     _param_burn_time,
		(ParamFloat<px4::params::ROCKET_IMPULS>)    _param_impulse,
		(ParamFloat<px4::params::ROCKET_THRUST>)    _param_thrust,
		(ParamFloat<px4::params::ROCKET_T_TAIL>)    _param_t_tail,

		// Inertias
		(ParamFloat<px4::params::ROCKET_IXX_F>)     _param_ixx_f,
		(ParamFloat<px4::params::ROCKET_IXX_D>)     _param_ixx_d,
		(ParamFloat<px4::params::ROCKET_IYY_F>)     _param_iyy_f,
		(ParamFloat<px4::params::ROCKET_IYY_D>)     _param_iyy_d,
		(ParamFloat<px4::params::ROCKET_IZZ_F>)     _param_izz_f,
		(ParamFloat<px4::params::ROCKET_IZZ_D>)     _param_izz_d,

		// MHE
		(ParamFloat<px4::params::ROCKET_MHE_QG>)    _param_mhe_qg,

		// MPC solver
		(ParamFloat<px4::params::ROCKET_MPC_TF>)    _param_mpc_tf,

		// SITL: use EKF2 lpos as GPS substitute for MHE (0=disabled, 1=enabled)
		(ParamInt<px4::params::ROCKET_SITL_GPS>)     _param_sitl_gps,

		// CAN SRV_FB feedback path for _de_act/_dr_act/_da_act
		// 0 = legacy first-order-lag filter (SITL)
		// 1 = CAN SRV_FB back-solve (HITL / real flight)
		(ParamInt<px4::params::ROCKET_SRV_FB>)       _param_use_srv_fb
	)
};
