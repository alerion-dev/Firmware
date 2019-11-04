/****************************************************************************
 *
 *   Copyright (c) 2013 - 2017 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control_main.cpp
 * Multicopter position controller.
 */

#include "aqua_pos_control.hpp"
#include "PositionControl.hpp"
#include "Utility/ControlMath.hpp"

#include <float.h>

using namespace time_literals;

AquaticPositionControl::AquaticPositionControl() :
	SuperBlock(nullptr, "APC"),
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_control(this),
	_cycle_perf(perf_alloc_once(PC_ELAPSED, MODULE_NAME": cycle time"))
{
	// fetch initial parameter values
	parameters_update(true);
}

AquaticPositionControl::~AquaticPositionControl()
{
	perf_free(_cycle_perf);
}

bool
AquaticPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position callback registration failed!");
		return false;
	}

	_local_pos_sub.set_interval_us(20_ms); // 50 Hz max update rate

	_time_stamp_last_loop = hrt_absolute_time();

	return true;
}

void
AquaticPositionControl::warn_rate_limited(const char *string)
{
	static hrt_abstime last_warn = 0;
	hrt_abstime now = hrt_absolute_time();

	if (now - last_warn > 200_ms) {
		PX4_WARN("%s", string);
		last_warn = now;
	}
}

int
AquaticPositionControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();
		SuperBlock::updateParams();

		_flight_tasks.handleParameterUpdate();
	}

	return OK;
}

void
AquaticPositionControl::poll_subscriptions()
{
	_vehicle_status_sub.update(&_vehicle_status);
	_control_mode_sub.update(&_control_mode);

	if (_att_sub.updated()) {
		vehicle_attitude_s att;

		if (_att_sub.copy(&att) && PX4_ISFINITE(att.q[0])) {
			auto q = Quatf(att.q);
			_states.forwardVector = Vector2f{q.conjugate(Vector3f{1.0f, 0.0f, 0.0f})};	// Unit forward vector
			_states.yaw = Eulerf(q).psi();
		} else {
			_states.forwardVector = Vector2f{NAN, NAN};
			_states.yaw = NAN;
		}
	}
}

void
AquaticPositionControl::set_vehicle_states()
{
	if (_local_pos.timestamp == 0) {
		return;
	}

	// only set position states if valid and finite
	if (PX4_ISFINITE(_local_pos.x) && PX4_ISFINITE(_local_pos.y) && _local_pos.xy_valid) {
		_states.position(0) = _local_pos.x;
		_states.position(1) = _local_pos.y;

	} else {
		_states.position(0) = _states.position(1) = NAN;
	}

	if (PX4_ISFINITE(_local_pos.vx) && PX4_ISFINITE(_local_pos.vy) && _local_pos.v_xy_valid) {
		_states.velocity(0) = _local_pos.vx;
		_states.velocity(1) = _local_pos.vy;
		_states.acceleration(0) = _vel_x_deriv.update(-_states.velocity(0));
		_states.acceleration(1) = _vel_y_deriv.update(-_states.velocity(1));

	} else {
		_states.velocity(0) = _states.velocity(1) = NAN;
		_states.acceleration(0) = _states.acceleration(1) = NAN;

		// since no valid velocity, update derivate with 0
		_vel_x_deriv.update(0.0f);
		_vel_y_deriv.update(0.0f);
	}
}

int
AquaticPositionControl::print_status()
{
	if (_flight_tasks.isAnyTaskActive()) {
		PX4_INFO("Running, active flight task: %i", _flight_tasks.getActiveTask());

	} else {
		PX4_INFO("Running, no flight task active");
	}

	perf_print_counter(_cycle_perf);

	return 0;
}

void
AquaticPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	_hydradrone_status_sub.update(&_hydradrone_status);
	if (hydradrone_status.status != hydradrone_status_s::HYDRADRONE_STATUS_AQUA) {
		return;
	}

	perf_begin(_cycle_perf);

	if (_local_pos_sub.update(&_local_pos)) {

		poll_subscriptions();
		parameters_update(false);

		// set _dt in controllib Block - the time difference since the last loop iteration in seconds
		const hrt_abstime time_stamp_current = hrt_absolute_time();
		setDt((time_stamp_current - _time_stamp_last_loop) / 1e6f);
		_time_stamp_last_loop = time_stamp_current;

		const bool was_in_failsafe = _in_failsafe;
		_in_failsafe = false;

		// check if any task is active
		if (_flight_tasks.isAnyTaskActive()) {
			// setpoints and constraints for the position controller from flighttask or failsafe
			vehicle_local_position_setpoint_s setpoint = FlightTask::empty_setpoint;
			vehicle_constraints_s constraints = FlightTask::empty_constraints;

			// update task
			if (!_flight_tasks.update()) {
				// FAILSAFE
				// Task was not able to update correctly. Do Failsafe.
				failsafe(setpoint, !was_in_failsafe);

			} else {
				setpoint = _flight_tasks.getPositionSetpoint();
				constraints = _flight_tasks.getConstraints();

				// Check if position, velocity or thrust pairs are valid -> trigger failsaife if no pair is valid
				if (!(PX4_ISFINITE(setpoint.x) && PX4_ISFINITE(setpoint.y)) &&
				    !(PX4_ISFINITE(setpoint.vx) && PX4_ISFINITE(setpoint.vy)) &&
				    !(PX4_ISFINITE(setpoint.thrust[0]) && PX4_ISFINITE(setpoint.thrust[1]))) {
					failsafe(setpoint, !was_in_failsafe);
				}
			}

			// publish trajectory setpoint
			_traj_sp_pub.publish(setpoint);

			// check if all local states are valid and map accordingly
			set_vehicle_states();

			// Update states, setpoints and constraints.
			_control.updateConstraints(constraints);
			_control.updateState(_states);

			// update position controller setpoints
			if (!_control.updateSetpoint(setpoint)) {
				warn_rate_limited("Position-Control Setpoint-Update failed");
				failsafe(setpoint, !was_in_failsafe);
				_control.updateSetpoint(setpoint);
				constraints = FlightTask::empty_constraints;
			}

			// Generate desired thrust and yaw.
			_control.generateThrustYawSetpoint(_dt);

			// Fill local position, velocity and thrust setpoint.
			// This message contains setpoints where each type of setpoint is either the input to the PositionController
			// or was generated by the PositionController and therefore corresponds to the PositioControl internal states (states that were generated by P-PID).
			// Example:
			// If the desired setpoint is position-setpoint, _local_pos_sp will contain
			// position-, velocity- and thrust-setpoint where the velocity- and thrust-setpoint were generated by the PositionControlller.
			// If the desired setpoint has a velocity-setpoint only, then _local_pos_sp will contain valid velocity- and thrust-setpoint, but the position-setpoint
			// will remain NAN. Given that the PositionController cannot generate a position-setpoint, this type of setpoint is always equal to the input to the
			// PositionController.
			vehicle_local_position_setpoint_s local_pos_sp{};
			local_pos_sp.timestamp = hrt_absolute_time();
			local_pos_sp.x = setpoint.x;
			local_pos_sp.y = setpoint.y;
			local_pos_sp.z = setpoint.z;
			local_pos_sp.yaw = _control.getYawSetpoint();
			local_pos_sp.yawspeed = _control.getYawspeedSetpoint();
			local_pos_sp.vx = PX4_ISFINITE(_control.getVelSp()(0)) ? _control.getVelSp()(0) : setpoint.vx;
			local_pos_sp.vy = PX4_ISFINITE(_control.getVelSp()(1)) ? _control.getVelSp()(1) : setpoint.vy;
			local_pos_sp.vz = setpoint.vz;
			// XY thrust is copied
			_control.getThrustSetpoint().copyTo(local_pos_sp.thrust);
			// Z thrust is zeroed
			local_pos_sp.thrust[2] = 0.f;

			// Publish local position setpoint
			// This message will be used by other modules (such as Landdetector) to determine
			// vehicle intention.
			_local_pos_sp_pub.publish(local_pos_sp);

			// Inform FlightTask about the input and output of the velocity controller
			// This is used to properly initialize the velocity setpoint when opening the position loop (position unlock)
			_flight_tasks.updateVelocityControllerIO(_control.getVelSp(), Vector3f(local_pos_sp.thrust));

			// Fill attitude setpoint. Attitude is computed from thrust setpoint and forward vector.
			_att_sp = ControlMath::thrustToAttitude(_control.getThrustSetpoint(), _states.forwardVector, _param_aqua_thr_angle_pow.get(), _reversible_thrust);
			_att_sp.yaw_sp_move_rate = _control.getYawspeedSetpoint();
			_att_sp.fw_control_yaw = false;
			_att_sp.apply_flaps = false;

			// publish attitude setpoint
			// Note: this requires review. The reason for not sending
			// an attitude setpoint is because for non-flighttask modes
			// the attitude septoint should come from another source, otherwise
			// they might conflict with each other such as in offboard attitude control.
			publish_attitude();

		} else {
			// no flighttask is active: set attitude setpoint to idle
			_att_sp.roll_body = _att_sp.pitch_body = 0.0f;
			_att_sp.yaw_body = _states.yaw;
			_att_sp.yaw_sp_move_rate = 0.0f;
			_att_sp.fw_control_yaw = false;
			_att_sp.apply_flaps = false;
			matrix::Quatf q_sp = matrix::Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body);
			q_sp.copyTo(_att_sp.q_d);
			_att_sp.q_d_valid = true;
			_att_sp.thrust_body[0] = 0.0f;
		}
	}

	perf_end(_cycle_perf);
}

void
AquaticPositionControl::start_flight_task()
{
	bool task_failure = false;
	bool should_disable_task = true;

	// Auto mode task assignment
	if (_control_mode.flag_control_auto_enabled) {
		// Auto related maneuvers
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		error =  _flight_tasks.switchTask(FlightTaskIndex::AquaticAutoLineSmooth);

		if (error != FlightTaskError::NoError) {
			warn_rate_limited(_flight_tasks.errorToString(error));
			task_failure = true;
		}
	}

	// check task failure
	if (task_failure) {

		// for some reason no flighttask was able to start.
		// go into failsafe flighttask
		FlightTaskError error = _flight_tasks.switchTask(FlightTaskIndex::Failsafe);

		if (error != FlightTaskError::NoError) {
			// No task was activated.
			_flight_tasks.switchTask(FlightTaskIndex::None);
		}

	} else if (should_disable_task) {
		_flight_tasks.switchTask(FlightTaskIndex::None);
	}
}

void
AquaticPositionControl::failsafe(vehicle_local_position_setpoint_s &setpoint, const bool warn)
{
	reset_setpoint_to_nan(setpoint);

	// Stop the drone horizontally
	setpoint.vx = setpoint.vy = 0.0f;

	if (warn) {
		PX4_WARN("Failsafe: stop and wait");
	}

	_in_failsafe = true;
}

void
AquaticPositionControl::reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint)
{
	setpoint.x = setpoint.y = setpoint.z = NAN;
	setpoint.vx = setpoint.vy = setpoint.vz = NAN;
	setpoint.yaw = setpoint.yawspeed = NAN;
	setpoint.acc_x = setpoint.acc_y = setpoint.acc_z = NAN;
	setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = NAN;
}

void
AquaticPositionControl::publish_attitude()
{
	_att_sp.timestamp = hrt_absolute_time();

	orb_publish_auto(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp, nullptr, ORB_PRIO_DEFAULT);
}

int AquaticPositionControl::task_spawn(int argc, char *argv[])
{
	AquaticPositionControl *instance = new AquaticPositionControl();

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

int AquaticPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AquaticPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to yaw and thrust scalar.

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("aqua_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int aqua_pos_control_main(int argc, char *argv[])
{
	return AquaticPositionControl::main(argc, argv);
}
