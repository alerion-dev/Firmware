/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "mc_att_control.hpp"

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

HydradroneCommander::HydradroneCommander() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
}

HydradroneCommander::~HydradroneCommander()
{
	perf_free(_loop_perf);
}

bool
HydradroneCommander::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}

	return true;
}

void HydradroneCommander::parameters_updated() {

}

void
HydradroneCommander::parameter_update_poll()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
		parameters_updated();
	}
}

void
HydradroneCommander::publish_rates_setpoint()
{
	_v_rates_sp.roll = _rates_sp(0);
	_v_rates_sp.pitch = _rates_sp(1);
	_v_rates_sp.yaw = _rates_sp(2);
	_v_rates_sp.thrust_body[0] = 0.0f;
	_v_rates_sp.thrust_body[1] = 0.0f;
	_v_rates_sp.thrust_body[2] = -_thrust_sp;
	_v_rates_sp.timestamp = hrt_absolute_time();

	_v_rates_sp_pub.publish(_v_rates_sp);
}

void HydradroneCommander::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	_hydradrone_status_sub.update();


	perf_begin(_loop_perf);

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		const hrt_abstime now = hrt_absolute_time();

		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};

		_actuators.timestamp_sample = angular_velocity.timestamp_sample;

		/* run the rate controller immediately after a gyro update */
		if (_v_control_mode.flag_control_rates_enabled) {
			control_attitude_rates(dt, rates);

			publish_actuator_controls();
			publish_rate_controller_status();
		}

		/* check for updates in other topics */
		_v_control_mode_sub.update(&_v_control_mode);
		_battery_status_sub.update(&_battery_status);
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);
		_landing_gear_sub.update(&_landing_gear);
		vehicle_status_poll();
		vehicle_motor_limits_poll();
		const bool manual_control_updated = _manual_control_sp_sub.update(&_manual_control_sp);
		const bool attitude_updated = vehicle_attitude_poll();

		_attitude_dt += dt;

		/* Check if we are in rattitude mode and the pilot is above the threshold on pitch
			* or roll (yaw can rotate 360 in normal att control). If both are true don't
			* even bother running the attitude controllers */
		if (_v_control_mode.flag_control_rattitude_enabled) {
			_v_control_mode.flag_control_attitude_enabled =
				fabsf(_manual_control_sp.y) <= _param_mc_ratt_th.get() &&
				fabsf(_manual_control_sp.x) <= _param_mc_ratt_th.get();
		}

		bool attitude_setpoint_generated = false;

		const bool is_hovering = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					 && !_vehicle_status.in_transition_mode;

		// vehicle is a tailsitter in transition mode
		const bool is_tailsitter_transition = _vehicle_status.in_transition_mode && _is_tailsitter;

		bool run_att_ctrl = _v_control_mode.flag_control_attitude_enabled && (is_hovering || is_tailsitter_transition);


		if (run_att_ctrl) {
			if (attitude_updated) {
				// Generate the attitude setpoint from stick inputs if we are in Manual/Stabilized mode
				if (_v_control_mode.flag_control_manual_enabled &&
				    !_v_control_mode.flag_control_altitude_enabled &&
				    !_v_control_mode.flag_control_velocity_enabled &&
				    !_v_control_mode.flag_control_position_enabled) {
					generate_attitude_setpoint(_attitude_dt, _reset_yaw_sp);
					attitude_setpoint_generated = true;
				}

				control_attitude();

				if (_v_control_mode.flag_control_yawrate_override_enabled) {
					/* Yaw rate override enabled, overwrite the yaw setpoint */
					_v_rates_sp_sub.update(&_v_rates_sp);
					const auto yawrate_reference = _v_rates_sp.yaw;
					_rates_sp(2) = yawrate_reference;
				}

				publish_rates_setpoint();
			}

		} else {
			/* attitude controller disabled, poll rates setpoint topic */
			if (_v_control_mode.flag_control_manual_enabled && is_hovering) {
				if (manual_control_updated) {
					/* manual rates control - ACRO mode */
					Vector3f man_rate_sp(
						math::superexpo(_manual_control_sp.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
						math::superexpo(-_manual_control_sp.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
						math::superexpo(_manual_control_sp.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get()));
					_rates_sp = man_rate_sp.emult(_acro_rate_max);
					_thrust_sp = _manual_control_sp.z;
					publish_rates_setpoint();
				}

			} else {
				/* attitude controller disabled, poll rates setpoint topic */
				if (_v_rates_sp_sub.update(&_v_rates_sp)) {
					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = -_v_rates_sp.thrust_body[2];
				}
			}
		}

		if (_v_control_mode.flag_control_termination_enabled) {
			if (!_vehicle_status.is_vtol) {
				_rates_sp.zero();
				_rate_control.resetIntegral();
				_thrust_sp = 0.0f;
				_att_control.zero();
				publish_actuator_controls();
			}
		}

		if (attitude_updated) {
			// reset yaw setpoint during transitions, tailsitter.cpp generates
			// attitude setpoint for the transition
			_reset_yaw_sp = (!attitude_setpoint_generated && !_v_control_mode.flag_control_rattitude_enabled) ||
					_vehicle_land_detected.landed ||
					(_vehicle_status.is_vtol && _vehicle_status.in_transition_mode);

			_attitude_dt = 0.f;
		}

		/* calculate loop update rate while disarmed or at least a few times (updating the filter is expensive) */
		if (!_v_control_mode.flag_armed || (now - _task_start) < 3300000) {
			_dt_accumulator += dt;
			++_loop_counter;

			if (_dt_accumulator > 1.f) {
				const float loop_update_rate = (float)_loop_counter / _dt_accumulator;
				_loop_update_rate_hz = _loop_update_rate_hz * 0.5f + loop_update_rate * 0.5f;
				_dt_accumulator = 0;
				_loop_counter = 0;
				_rate_control.setDTermCutoff(_loop_update_rate_hz, _param_mc_dterm_cutoff.get(), true);
			}
		}

		parameter_update_poll();
	}

	perf_end(_loop_perf);
}

int HydradroneCommander::task_spawn(int argc, char *argv[])
{
	HydradroneCommander *instance = new HydradroneCommander();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int HydradroneCommander::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_loop_perf);

	return 0;
}

int HydradroneCommander::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int HydradroneCommander::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the hydradrone commander. It handles the mode switching between the multicopter and aquatic modes.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hydradrone_commander", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int hydradrone_commander_main(int argc, char *argv[])
{
	return HydradroneCommander::main(argc, argv);
}
