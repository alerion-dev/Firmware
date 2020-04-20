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
 * @file FlightAutoLine.cpp
 */

#include "FlightTaskAquaticAutoLineSmooth.hpp"
#include <mathlib/mathlib.h>
#include <lib/systemlib/mavlink_log.h>
#include <float.h>

using namespace matrix;

bool FlightTaskAquaticAutoLineSmooth::activate(vehicle_local_position_setpoint_s last_setpoint)
{
	bool ret = FlightTaskAquaticAuto::activate(last_setpoint);

	// Set setpoints equal current state.
	_velocity_setpoint = _velocity;
	_position_setpoint = _position;

	checkSetpoints(last_setpoint);
	const Vector2f accel_prev(last_setpoint.acc_x, last_setpoint.acc_y);
	const Vector2f vel_prev(last_setpoint.vx, last_setpoint.vy);
	const Vector2f pos_prev(last_setpoint.x, last_setpoint.y);

	for (int i = 0; i < 2; ++i) {
		_trajectory[i].reset(accel_prev(i), vel_prev(i), pos_prev(i));
	}

	_thrust_setpoint = Vector3f(NAN, NAN, NAN);

	_yaw_sp_prev = last_setpoint.yaw;
	_updateTrajConstraints();

	return ret;
}

void FlightTaskAquaticAutoLineSmooth::reActivate()
{
	// On water, reset acceleration and velocity to zero
	for (int i = 0; i < 2; ++i) {
		_trajectory[i].reset(0.f, 0.f, _position(i));
	}
}

void FlightTaskAquaticAutoLineSmooth::checkSetpoints(vehicle_local_position_setpoint_s &setpoints)
{
	// If the position setpoint is unknown, set to the current postion
	if (!PX4_ISFINITE(setpoints.x)) { setpoints.x = _position(0); }

	if (!PX4_ISFINITE(setpoints.y)) { setpoints.y = _position(1); }

	// If the velocity setpoint is unknown, set to the current velocity
	if (!PX4_ISFINITE(setpoints.vx)) { setpoints.vx = _velocity(0); }

	if (!PX4_ISFINITE(setpoints.vy)) { setpoints.vy = _velocity(1); }

	// No acceleration estimate available, set to zero if the setpoint is NAN
	if (!PX4_ISFINITE(setpoints.acc_x)) { setpoints.acc_x = 0.f; }

	if (!PX4_ISFINITE(setpoints.acc_y)) { setpoints.acc_y = 0.f; }

	if (!PX4_ISFINITE(setpoints.yaw)) { setpoints.yaw = _yaw; }
}

void FlightTaskAquaticAutoLineSmooth::_prepareVelocitySetpoints()
{
	// XY Velocity waypoint
	// TODO : Rewiew that. What is the expected behavior?
	_position_setpoint = Vector3f(NAN, NAN, NAN);
	Vector2f vel_sp_xy = Vector2f(_velocity).unit_or_zero() * _cruise_speed;
	_velocity_setpoint = Vector3f(vel_sp_xy(0), vel_sp_xy(1), NAN);
}

bool FlightTaskAquaticAutoLineSmooth::update()
{
	// always reset constraints because they might change depending on the type
	_setDefaultConstraints();

	_velocity_setpoint = Vector3f(NAN, NAN, NAN);

	switch (_type) {
	case WaypointType::idle:
		break;
	case WaypointType::land:
		mavlink_log_critical(&_log_publisher, "Waypoint of type 'Land' in aquatic mode forbidden");
		break;
	case WaypointType::loiter:
	/* fallthrought */
	case WaypointType::position:
		_position_setpoint = Vector3f(_target(0), _target(1), NAN);
		break;
	case WaypointType::takeoff:
		mavlink_log_critical(&_log_publisher, "Waypoint of type 'Takeoff' in aquatic mode forbidden");
		break;
	case WaypointType::velocity:
		_prepareVelocitySetpoints();
		break;
	default:
		_position_setpoint = Vector3f(_target(0), _target(1), NAN);
		break;
	}

	_generateSetpoints();

	return true;
}

/**
 * EKF reset handling functions
 * Those functions are called by the base FlightTask in
 * case of an EKF reset event
 */
void FlightTaskAquaticAutoLineSmooth::_ekfResetHandlerPositionXY()
{
	_trajectory[0].setCurrentPosition(_position(0));
	_trajectory[1].setCurrentPosition(_position(1));
}

void FlightTaskAquaticAutoLineSmooth::_ekfResetHandlerVelocityXY()
{
	_trajectory[0].setCurrentVelocity(_velocity(0));
	_trajectory[1].setCurrentVelocity(_velocity(1));
}

void FlightTaskAquaticAutoLineSmooth::_ekfResetHandlerHeading(float delta_psi)
{
	_yaw_sp_prev += delta_psi;
}

void FlightTaskAquaticAutoLineSmooth::_generateSetpoints()
{
	_prepareSetpoints();
	_generateTrajectory();

	//_generateHeading();
}

void FlightTaskAquaticAutoLineSmooth::_generateHeading()
{
	// Generate heading along trajectory if possible, otherwise hold the previous yaw setpoint
	if (!_generateHeadingAlongTraj()) {
		_yaw_setpoint = _yaw_sp_prev;
	}
}

bool FlightTaskAquaticAutoLineSmooth::_generateHeadingAlongTraj()
{
	bool res = false;
	Vector2f vel_sp_xy(_velocity_setpoint);
	Vector2f traj_to_target = Vector2f(_target) - Vector2f(_position);

	if ((vel_sp_xy.length() > .1f) &&
	    (traj_to_target.length() > _target_acceptance_radius)) {
		// Generate heading from velocity vector, only if it is long enough
		// and if the drone is far enough from the target
		_compute_heading_from_2D_vector(_yaw_setpoint, vel_sp_xy);
		res = true;
	}

	return res;
}

/* Constrain some value vith a constrain depending on the sign of the constraint
 * Example: 	- if the constrain is -5, the value will be constrained between -5 and 0
 * 		- if the constrain is 5, the value will be constrained between 0 and 5
 */
inline float FlightTaskAquaticAutoLineSmooth::_constrainOneSide(float val, float constraint)
{
	const float min = (constraint < FLT_EPSILON) ? constraint : 0.f;
	const float max = (constraint > FLT_EPSILON) ? constraint : 0.f;

	return math::constrain(val, min, max);
}

inline float FlightTaskAquaticAutoLineSmooth::_constrainAbs(float val, float min, float max)
{
	return math::sign(val) * math::constrain(fabsf(val), fabsf(min), fabsf(max));
}

float FlightTaskAquaticAutoLineSmooth::_getSpeedAtTarget()
{
	// Compute the maximum allowed speed at the waypoint assuming that we want to
	// connect the two lines (prev-current and current-next)
	// with a tangent circle with constant speed and desired centripetal acceleration: a_centripetal = speed^2 / radius
	// The circle should in theory start and end at the intersection of the lines and the waypoint's acceptance radius.
	// This is not exactly true in reality since Navigator switches the waypoint so we have to take in account that
	// the real acceptance radius is smaller.
	// It can be that the next waypoint is the last one or that the drone will have to stop for some other reason
	// so we have to make sure that the speed at the current waypoint allows to stop at the next waypoint.
	float speed_at_target = 0.0f;

	const float distance_current_next = Vector2f{_target - _next_wp}.length();
	const bool waypoint_overlap = Vector2f{_target - _prev_wp}.length() < _target_acceptance_radius;

	if (distance_current_next > 0.001f &&
	    !waypoint_overlap) {
		// Max speed between current and next
		const float max_speed_current_next = _getMaxSpeedFromDistance(distance_current_next);
		const float alpha = acosf(Vector2f{_target - _prev_wp}.unit_or_zero() *
					  Vector2f{_target - _next_wp}.unit_or_zero());
		// We choose a maximum centripetal acceleration of MPC_ACC_HOR * MPC_XY_TRAJ_P to take in account
		// that there is a jerk limit (a direct transition from line to circle is not possible)
		// MPC_XY_TRAJ_P should be between 0 and 1.
		float accel_tmp = _param_aqua_traj_p.get() * _param_aqua_acc_fwd_max.get();
		float max_speed_in_turn = math::trajectory::computeMaxSpeedInWaypoint(alpha,
					  accel_tmp,
					  _target_acceptance_radius);
		speed_at_target = math::min(math::min(max_speed_in_turn, max_speed_current_next), _cruise_speed);
	}

	return speed_at_target;
}

float FlightTaskAquaticAutoLineSmooth::_getMaxSpeedFromDistance(float braking_distance)
{
	float max_speed = math::trajectory::computeMaxSpeedFromBrakingDistance(_param_aqua_jerk_max.get(), _param_aqua_acc_bwd.get(), braking_distance);
	// To avoid high gain at low distance due to the sqrt, we take the minimum
	// of this velocity and a slope of "traj_p" m/s per meter
	max_speed = math::min(max_speed, braking_distance * _param_aqua_traj_p.get());

	return max_speed;
}

void FlightTaskAquaticAutoLineSmooth::_prepareSetpoints()
{
	// Interface: A valid position setpoint generates a velocity target using a P controller. If a velocity is specified
	// that one is used as a velocity limit.
	// If the position setpoints are set to NAN, the values in the velocity setpoints are used as velocity targets: nothing to do here.

	if (PX4_ISFINITE(_position_setpoint(0)) && PX4_ISFINITE(_position_setpoint(1))) {
		// Use position setpoints to generate velocity setpoints

		// Get various path specific vectors
		Vector2f pos_traj;
		pos_traj(0) = _trajectory[0].getCurrentPosition();
		pos_traj(1) = _trajectory[1].getCurrentPosition();
		Vector2f pos_traj_to_dest(Vector2f{_position_setpoint} - pos_traj);
		Vector2f u_pos_traj_to_dest(pos_traj_to_dest.unit_or_zero());

		// Unconstrained desired velocity vector
		Vector2f vel_sp = u_pos_traj_to_dest * _cruise_speed;

		Vector2f vel_max;
		vel_max(0) = _getMaxSpeedFromDistance(fabsf(pos_traj_to_dest(0)));
		vel_max(1) = _getMaxSpeedFromDistance(fabsf(pos_traj_to_dest(1)));

		// Compute the minimum speed in NE frame. This is used
		// to force the drone to pass the waypoint with a desired speed
		Vector2f vel_min;
		Vector2f u_prev_to_target(Vector2f{_target - _prev_wp}.unit_or_zero());
		vel_min = u_prev_to_target * _getSpeedAtTarget();

		// Constrain the norm of each component using min and max values
		Vector2f vel_sp_constrained;
		vel_sp_constrained(0) = _constrainAbs(vel_sp(0), vel_min(0), vel_max(0));
		vel_sp_constrained(1) = _constrainAbs(vel_sp(1), vel_min(1), vel_max(1));

		for (int i = 0; i < 2; i++) {
			// If available, constrain the velocity using _velocity_setpoint(.)
			if (PX4_ISFINITE(_velocity_setpoint(i))) {
				_velocity_setpoint(i) = _constrainOneSide(vel_sp_constrained(i), _velocity_setpoint(i));

			} else {
				_velocity_setpoint(i) = vel_sp_constrained(i);
			}
		}

	}
}

void FlightTaskAquaticAutoLineSmooth::_updateTrajConstraints()
{
	// Update the constraints of the trajectories
	_trajectory[0].setMaxAccel(_param_aqua_acc_fwd_max.get()); // TODO : Should be computed using heading
	_trajectory[1].setMaxAccel(_param_aqua_acc_fwd_max.get());
	_trajectory[0].setMaxVel(_param_aqua_vel_max.get());
	_trajectory[1].setMaxVel(_param_aqua_vel_max.get());
	_trajectory[0].setMaxJerk(_param_aqua_jerk_max.get()); // TODO : Should be computed using heading
	_trajectory[1].setMaxJerk(_param_aqua_jerk_max.get());
}

void FlightTaskAquaticAutoLineSmooth::_generateTrajectory()
{
	if (!PX4_ISFINITE(_velocity_setpoint(0)) || !PX4_ISFINITE(_velocity_setpoint(1))) {
		return;
	}

	/* Slow down the trajectory by decreasing the integration time based on the position error.
	 * This is only performed when the drone is behind the trajectory
	 */
	Vector2f position_trajectory_xy(_trajectory[0].getCurrentPosition(), _trajectory[1].getCurrentPosition());
	Vector2f position_xy(_position);
	Vector2f vel_traj_xy(_trajectory[0].getCurrentVelocity(), _trajectory[1].getCurrentVelocity());
	Vector2f drone_to_trajectory_xy(position_trajectory_xy - position_xy);
	float position_error = drone_to_trajectory_xy.length();

	float time_stretch = 1.f - math::constrain(position_error * 0.5f, 0.f, 1.f);

	// Don't stretch time if the drone is ahead of the position setpoint
	if (drone_to_trajectory_xy.dot(vel_traj_xy) < 0.f) {
		time_stretch = 1.f;
	}

	Vector2f jerk_sp_smooth;
	Vector2f accel_sp_smooth;
	Vector2f vel_sp_smooth;
	Vector2f pos_sp_smooth;

	for (int i = 0; i < 2; ++i) {
		_trajectory[i].updateTraj(_deltatime, time_stretch);
		jerk_sp_smooth(i) = _trajectory[i].getCurrentJerk();
		accel_sp_smooth(i) = _trajectory[i].getCurrentAcceleration();
		vel_sp_smooth(i) = _trajectory[i].getCurrentVelocity();
		pos_sp_smooth(i) = _trajectory[i].getCurrentPosition();
	}

	_updateTrajConstraints();

	for (int i = 0; i < 2; ++i) {
		_trajectory[i].updateDurations(_velocity_setpoint(i));
	}

	VelocitySmoothing::timeSynchronization(_trajectory, 2); // Synchronize x and y only

	_jerk_setpoint = Vector3f{jerk_sp_smooth(0), jerk_sp_smooth(1), NAN};
	_acceleration_setpoint = Vector3f{accel_sp_smooth(0), accel_sp_smooth(1), NAN};
	_velocity_setpoint = Vector3f{vel_sp_smooth(0), vel_sp_smooth(1), NAN};
	_position_setpoint = Vector3f{pos_sp_smooth(0), pos_sp_smooth(1), NAN};
}
