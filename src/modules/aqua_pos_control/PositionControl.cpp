/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include "Utility/ControlMath.hpp"
#include <px4_defines.h>

using namespace matrix;

PositionControl::PositionControl(ModuleParams *parent) :
	ModuleParams(parent)
{}

void PositionControl::updateState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_vel_dot = states.acceleration;
	_yaw = states.yaw;
}

void PositionControl::_setCtrlFlag(bool value)
{
	for (int i = 0; i <= 1; i++) {
		_ctrl_pos[i] = _ctrl_vel[i] = value;
	}
}

bool PositionControl::updateSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	// by default we use the entire position-velocity control-loop pipeline (flag only for logging purpose)
	_setCtrlFlag(true);

	_pos_sp = Vector2f(setpoint.x, setpoint.y);
	_vel_sp = Vector2f(setpoint.vx, setpoint.vy);
	_thr_sp = Vector2f(setpoint.thrust);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
	bool mapping_succeeded = _interfaceMapping();

	// If full manual is required (thrust already generated), don't run position/velocity
	// controller and just return thrust.
	_skip_controller = PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1));

	return mapping_succeeded;
}

void PositionControl::generateThrustYawSetpoint(const float dt)
{
	if (_skip_controller) {

		// Already received a valid thrust set-point.
		// Limit the thrust vector.
		float thr_mag = _thr_sp.length();

		if (thr_mag > _param_aqua_thr_max.get()) {
			_thr_sp = _thr_sp.normalized() * _param_aqua_thr_max.get();
		}

		// Just set the set-points equal to the current vehicle state.
		_pos_sp = _pos;
		_vel_sp = _vel;

	} else {
		_positionController();
		_velocityController(dt);
	}
}

bool PositionControl::_interfaceMapping()
{
	// if nothing is valid, then apply failsafe landing
	bool failsafe = false;

	// Respects FlightTask interface, where NAN-set-points are of no interest
	// and do not require control. A valid position and velocity setpoint will
	// be mapped to a desired position setpoint with a feed-forward term.
	// States and setpoints which are integrals of the reference setpoint are set to 0.
	// For instance: reference is velocity-setpoint -> position and position-setpoint = 0
	//               reference is thrust-setpoint -> position, velocity, position-/velocity-setpoint = 0
	for (int i = 0; i <= 1; i++) {

		if (PX4_ISFINITE(_pos_sp(i))) {
			// Position control is required

			if (!PX4_ISFINITE(_vel_sp(i))) {
				// Velocity is not used as feedforward term.
				_vel_sp(i) = 0.0f;
			}

			// thrust setpoint is not supported in position control
			_thr_sp(i) = NAN;

			// to run position control, we require valid position and velocity
			if (!PX4_ISFINITE(_pos(i)) || !PX4_ISFINITE(_vel(i))) {
				failsafe = true;
			}

		} else if (PX4_ISFINITE(_vel_sp(i))) {

			// Velocity controller is active without position control.
			// Set integral states and setpoints to 0
			_pos_sp(i) = _pos(i) = 0.0f;
			_ctrl_pos[i] = false; // position control-loop is not used

			// thrust setpoint is not supported in velocity control
			_thr_sp(i) = NAN;

			// to run velocity control, we require valid velocity
			if (!PX4_ISFINITE(_vel(i))) {
				failsafe = true;
			}

		} else if (PX4_ISFINITE(_thr_sp(i))) {

			// Thrust setpoint was generated from sticks directly.
			// Set all integral states and setpoints to 0
			_pos_sp(i) = _pos(i) = 0.0f;
			_vel_sp(i) = _vel(i) = 0.0f;
			_ctrl_pos[i] = _ctrl_vel[i] = false; // position/velocity control loop is not used

			// Reset the Integral term.
			_vel_int(i) = 0.0f;
			// Don't require velocity derivative.
			_vel_dot(i) = 0.0f;

		} else {
			// nothing is valid. do failsafe
			failsafe = true;
		}
	}

	// ensure that vel_dot is finite, otherwise set to 0
	if (!PX4_ISFINITE(_vel_dot(0)) || !PX4_ISFINITE(_vel_dot(1))) {
		_vel_dot(0) = _vel_dot(1) = 0.0f;
	}

	if (!PX4_ISFINITE(_yawspeed_sp)) {
		// Set the yawspeed to 0 since not used.
		_yawspeed_sp = 0.0f;
	}

	if (!PX4_ISFINITE(_yaw_sp)) {
		// Set the yaw-sp equal the current yaw.
		// That is the best we can do and it also
		// agrees with FlightTask-interface definition.
		if (PX4_ISFINITE(_yaw)) {
			_yaw_sp = _yaw;

		} else {
			failsafe = true;
		}
	}

	// check failsafe
	if (failsafe) {
		// cutoff thrust
		_thr_sp.zero();

		_setCtrlFlag(false);
	}

	return !(failsafe);
}

void PositionControl::_positionController()
{
	// P-position controller on XY
	const Vector2f vel_sp_position = Vector2f{_pos_sp - _pos} * _param_aqua_pos_p.get();

	// Feed forward velocity term
	_vel_sp = vel_sp_position + _vel_sp;

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp = ControlMath::constrainXYaqua(vel_sp_position, _vel_sp - vel_sp_position, _param_aqua_vel_max.get());
}

void PositionControl::_velocityController(const float &dt)
{
	// Generate desired thrust setpoint.
	// PID
	// u_des = P(vel_err) + D(vel_err_dot) + I(vel_integral)
	// Umin <= u_des <= Umax
	//
	// Anti-Windup:
	// u_des = _thr_sp; r = _vel_sp; y = _vel
	// u_des >= Umax and r - y >= 0 => Saturation = true
	// u_des >= Umax and r - y <= 0 => Saturation = false
	// u_des <= Umin and r - y <= 0 => Saturation = true
	// u_des <= Umin and r - y >= 0 => Saturation = false
	//
	// 	Notes:
	// - PID implementation is in NED-frame

	const Vector2f vel_err = _vel_sp - _vel;

	if (PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1))) {
		// Thrust set-point is already provided.
	} else {
		// PID-velocity controller.
		Vector2f thrust_desired;
		thrust_desired(0) = _param_aqua_vel_p.get() * vel_err(0) + _param_aqua_vel_d.get() * _vel_dot(0) + _vel_int(0);
		thrust_desired(1) = _param_aqua_vel_p.get() * vel_err(1) + _param_aqua_vel_d.get() * _vel_dot(1) + _vel_int(1);

		if ((thrust_desired * thrust_desired) > (_param_aqua_thr_max.get() * _param_aqua_thr_max.get())) {
			// Thrust setpoint greater than maximum thrust, capping it and using Anti-windup
			float mag = _thr_sp.length();
			_thr_sp(0) = thrust_desired(0) / mag * _param_aqua_thr_max.get();
			_thr_sp(1) = thrust_desired(1) / mag * _param_aqua_thr_max.get();

			// Use tracking Anti-Windup: during saturation, the integrator is used to unsaturate the output
			// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
			float arw_gain = 2.f / _param_aqua_vel_p.get();

			Vector2f vel_err_lim;
			vel_err_lim(0) = vel_err(0) - (thrust_desired(0) - _thr_sp(0)) * arw_gain;
			vel_err_lim(1) = vel_err(1) - (thrust_desired(1) - _thr_sp(1)) * arw_gain;

			// Update integral
			_vel_int(0) += _param_aqua_vel_i.get() * vel_err_lim(0) * dt;
			_vel_int(1) += _param_aqua_vel_i.get() * vel_err_lim(1) * dt;
		} else {
			// Set thrust setpoint (PID output)
			_thr_sp = thrust_desired;

			// Update integral
			_vel_int(0) += _param_aqua_vel_i.get() * vel_err(0) * dt;
			_vel_int(1) += _param_aqua_vel_i.get() * vel_err(1) * dt;
		}
	}
}

void PositionControl::updateConstraints(const vehicle_constraints_s &constraints)
{
	_constraints = constraints;

	// For safety check if adjustable constraints are below global constraints. If they are not stricter than global
	// constraints, then just use global constraints for the limits.

	if (!PX4_ISFINITE(constraints.speed_xy) || !(constraints.speed_xy < _param_aqua_vel_max.get())) {
		_constraints.speed_xy = _param_aqua_vel_max.get();
	}
}

void PositionControl::updateParams()
{
	ModuleParams::updateParams();
}
