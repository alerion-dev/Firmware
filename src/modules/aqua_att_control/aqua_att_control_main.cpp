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
 * @file aqua_att_control_main.cpp
 * Aquatic attitude controller.
 *
 * @author Antoine Richard <antoine.richard@alerion.fr>
 *
 */

#include "aqua_att_control.hpp"

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

AquaticAttitudeControl::AquaticAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, "aqua_att_control"))
{
	/* initialize quaternions in messages to be valid */
	_v_att.q[0] = 1.f;
	_v_att_sp.q_d[0] = 1.f;

	_yaw_sp = 0.0f;

	parameters_updated();
}

AquaticAttitudeControl::~AquaticAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
AquaticAttitudeControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}

	return true;
}

void
AquaticAttitudeControl::parameters_updated()
{
	// yaw control parameters
	_yaw_control.setProportionalGain(_param_aqua_yaw_p.get());
	_yaw_control.setRateLimit(math::radians(_param_aqua_yawrate_max.get()));

	// yaw rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	float K = _param_aqua_yawrate_k.get();
	_yaw_rate_control.setGains(K * _param_aqua_yawrate_p.get(), K * _param_aqua_yawrate_i.get(), K * _param_aqua_yawrate_d.get());
	_yaw_rate_control.setIntegratorLimit(_param_aqua_yr_int_lim.get());
	_yaw_rate_control.setDTermCutoff(_loop_update_rate_hz, _param_aqua_dterm_cutoff.get(), false);
	_yaw_rate_control.setFeedForwardGain(_param_aqua_yawrate_ff.get());

	// Motors output circuit breaker
	_actuators_1_circuit_breaker_enabled = circuit_breaker_enabled_by_val(_param_cbrk_rate_ctrl.get(), CBRK_RATE_CTRL_KEY);
}

void
AquaticAttitudeControl::parameter_update_poll()
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

bool
AquaticAttitudeControl::vehicle_attitude_poll()
{
	if (_v_att_sub.update(&_v_att)) {
		return true;
	}

	return false;
}

float AquaticAttitudeControl::throttle_curve(float throttle_stick_input)
{
	return throttle_stick_input * _param_aqua_thr_max_man.get();
}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void
AquaticAttitudeControl::control_attitude()
{
	_v_att_sp_sub.update(&_v_att_sp);

	// reinitialize the setpoint while not armed to make sure no value from the last mode or flight is still kept
	if (!_v_control_mode.flag_armed) {
		Quatf().copyTo(_v_att_sp.q_d);
		Vector3f().copyTo(_v_att_sp.thrust_body);
	}

	// physical thrust axis is the body X axis
	_thrust_sp = _v_att_sp.thrust_body[0];

	// no desired attitude, stay in current attitude
	if(!_v_att_sp.q_d_valid) {
		_v_att_sp.q_d[0] = _v_att.q[0];
		_v_att_sp.q_d[1] = _v_att.q[1];
		_v_att_sp.q_d[2] = _v_att.q[2];
		_v_att_sp.q_d[3] = _v_att.q[3];
	}

	_yawrate_sp = _yaw_control.update(Quatf(_v_att.q), Quatf(_v_att_sp.q_d), _v_att_sp.yaw_sp_move_rate);
}

/*
 * Attitude rates controller.
 * Input: '_yawrate_sp'
 * Output: '_yaw_torque' output
 */
void
AquaticAttitudeControl::control_attitude_rate(float dt, const float yawrate)
{
	// reset integral if disarmed
	if (!_v_control_mode.flag_armed) {
		_yaw_rate_control.resetIntegral();
	}

	_yaw_torque = _yaw_rate_control.update(yawrate, _yawrate_sp, dt);
}

void
AquaticAttitudeControl::publish_rates_setpoint()
{
	_v_rates_sp.roll = 0.0f;
	_v_rates_sp.pitch = 0.0f;
	_v_rates_sp.yaw = _yawrate_sp;
	_v_rates_sp.thrust_body[0] = _thrust_sp;
	_v_rates_sp.thrust_body[1] = 0.0f;
	_v_rates_sp.thrust_body[2] = 0.0f;
	_v_rates_sp.timestamp = hrt_absolute_time();

	_v_rates_sp_pub.publish(_v_rates_sp);
}

void
AquaticAttitudeControl::publish_rate_controller_status()
{
	struct rate_ctrl_status_s rate_ctrl_status = {};
	rate_ctrl_status.timestamp = hrt_absolute_time();
	_yaw_rate_control.getRateControlStatus(rate_ctrl_status);
	_controller_status_pub.publish(rate_ctrl_status);
}

void
AquaticAttitudeControl::publish_actuator_controls()
{
	// Yaw only
	_actuators.control[0] = 0.0f;
	_actuators.control[1] = 0.0f;
	_actuators.control[2] = PX4_ISFINITE(_yaw_torque) ? _yaw_torque : 0.0f;

	// +X axis thrust
	_actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
	// note: _actuators.timestamp_sample is set in AquaticAttitudeControl::Run()
	_actuators.timestamp = hrt_absolute_time();

	/* scale effort by battery status */
	if (_param_mc_bat_scale_en.get() && _battery_status.scale > 0.0f) {
		for (int i = 0; i < 4; i++) {
			_actuators.control[i] *= _battery_status.scale;
		}
	}

	if (!_actuators_1_circuit_breaker_enabled) {
		orb_publish_auto(ORB_ID(actuator_controls_1), &_actuators_1_pub, &_actuators, nullptr, ORB_PRIO_DEFAULT);
	}
}

void
AquaticAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	_hydradrone_status_sub.update(&_hydradrone_status);
	if (_hydradrone_status.status != hydradrone_status_s::HYDRADRONE_STATUS_AQUA) {
		return;
	}

	perf_begin(_loop_perf);

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		const hrt_abstime now = hrt_absolute_time();

		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);
		_last_run = now;

		const float yawrate{angular_velocity.xyz[2]};

		_actuators.timestamp_sample = angular_velocity.timestamp_sample;

		/* run the rate controller immediately after a gyro update */
		if (_v_control_mode.flag_control_rates_enabled) {
			control_attitude_rate(dt, yawrate);

			publish_actuator_controls();
			publish_rate_controller_status();
		}

		/* check for updates in other topics */
		_v_control_mode_sub.update(&_v_control_mode);
		_battery_status_sub.update(&_battery_status);
		const bool manual_control_updated = _manual_control_sp_sub.update(&_manual_control_sp);
		const bool attitude_updated = vehicle_attitude_poll();

		if(_v_control_mode.flag_control_manual_enabled) {
			if(manual_control_updated) {
				// Manual control of yaw with roll stick for ease of use
				_yawrate_sp = _manual_control_sp.y;

				// Manual control of thrust with thrust stick
				_thrust_sp = throttle_curve(_manual_control_sp.z);

				publish_rates_setpoint();
			}
		} else if (	_v_control_mode.flag_control_attitude_enabled &&
				!_v_control_mode.flag_control_yawrate_override_enabled) {
			if (attitude_updated) {
				// from auto pos controller
				control_attitude();

				publish_rates_setpoint();
			}

		} else {
			/* attitude controller disabled, poll rates setpoint topic */
			if (_v_rates_sp_sub.update(&_v_rates_sp)) {
				_yawrate_sp = _v_rates_sp.yaw;
				_thrust_sp = _v_rates_sp.thrust_body[0];
			}
		}

		if (_v_control_mode.flag_control_termination_enabled) {
			_yawrate_sp = 0.0f;
			_yaw_rate_control.resetIntegral();
			_thrust_sp = 0.0f;
			_yaw_torque = 0.0f;
			publish_actuator_controls();
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
				_yaw_rate_control.setDTermCutoff(_loop_update_rate_hz, _param_aqua_dterm_cutoff.get(), true);
			}
		}

		parameter_update_poll();
	}

	perf_end(_loop_perf);
}

int AquaticAttitudeControl::task_spawn(int argc, char *argv[])
{
	AquaticAttitudeControl *instance = new AquaticAttitudeControl();

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

int AquaticAttitudeControl::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_loop_perf);

	print_message(_actuators);

	return 0;
}

int AquaticAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AquaticAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the aquatic yaw and yaw rate controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) or a yaw rate setpoint (via `manual_control_setpoint` topic)
as inputs and outputs actuator control messages.

The controller has two loops: a P loop for yaw error and a PID loop for yaw rate error.

### Implementation
To reduce control latency, the module directly polls on the gyro topic published by the IMU driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("aqua_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int aqua_att_control_main(int argc, char *argv[])
{
	return AquaticAttitudeControl::main(argc, argv);
}
