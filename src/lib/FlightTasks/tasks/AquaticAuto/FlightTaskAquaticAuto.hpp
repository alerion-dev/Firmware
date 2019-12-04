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
 * @file FlightTaskAuto.hpp
 *
 * Map from global triplet to local quadruple.
 */

#pragma once

#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <lib/ecl/geo/geo.h>

#include "FlightTaskAuto.hpp"

class FlightTaskAquaticAuto : public FlightTask
{
public:
	FlightTaskAquaticAuto() = default;

	virtual ~FlightTaskAquaticAuto() = default;
	bool activate(vehicle_local_position_setpoint_s last_setpoint) override;
	bool updateInitialize() override;
	bool updateFinalize() override;

protected:
	void _setDefaultConstraints() override;
	void _updateInternalWaypoints(); /**< Depending on state of vehicle, the internal waypoints might differ from target (for instance if offtrack). */
	bool _compute_heading_from_2D_vector(float &heading, matrix::Vector2f v); /**< Computes and sets heading a 2D vector */

	matrix::Vector2f _prev_wp{}; /**< Previous waypoint  (local frame). If no previous triplet is available, the prev_wp is set to current position. */
	matrix::Vector2f _target{}; /**< Target waypoint  (local frame).*/
	matrix::Vector2f _next_wp{}; /**< The next waypoint after target (local frame). If no next setpoint is available, next is set to target. */

	float _cruise_speed{0.0f}; /**< Requested cruise speed. If not valid, default cruise speed is used. */
	WaypointType _type{WaypointType::idle}; /**< Type of current target triplet. */

	uORB::SubscriptionData<home_position_s>			_sub_home_position{ORB_ID(home_position)};
	uORB::SubscriptionData<manual_control_setpoint_s>	_sub_manual_control_setpoint{ORB_ID(manual_control_setpoint)};
	uORB::SubscriptionData<vehicle_status_s>		_sub_vehicle_status{ORB_ID(vehicle_status)};

	State _current_state{State::none};
	float _target_acceptance_radius{0.0f}; /**< Acceptances radius of the target */

	float _yaw_sp_prev{NAN};
	bool _yaw_sp_aligned{false};

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
					(ParamFloat<px4::params::AQUA_VEL_MAX>) _param_aqua_vel_max_automode,
					(ParamFloat<px4::params::AQUA_YAWR_MAX>) _param_aqua_yawrate_max_automode,
					(ParamFloat<px4::params::MIS_YAW_ERR>) _param_mis_yaw_err	// yaw-error threshold
				       );

private:
	matrix::Vector2f _lock_position_xy{NAN, NAN}; /**< if no valid triplet is received, lock positition to current position */
	bool _yaw_lock{false}; /**< if within acceptance radius, lock yaw to current yaw */

	uORB::SubscriptionData<position_setpoint_triplet_s> _sub_triplet_setpoint{ORB_ID(position_setpoint_triplet)};

	matrix::Vector2f _triplet_target; /**< current triplet from navigator which may differ from the intenal one (_target) depending on the vehicle state. */
	matrix::Vector2f _triplet_prev_wp; /**< previous triplet from navigator which may differ from the intenal one (_prev_wp) depending on the vehicle state.*/
	matrix::Vector2f _triplet_next_wp; /**< next triplet from navigator which may differ from the intenal one (_next_wp) depending on the vehicle state.*/
	matrix::Vector2f _closest_pt; /**< closest point to the vehicle position on the line previous - target */

	map_projection_reference_s _reference_position{}; /**< Structure used to project lat/lon setpoint into local frame. */
	float _reference_altitude{NAN};  /**< Altitude relative to ground. */
	hrt_abstime _time_stamp_reference{0}; /**< time stamp when last reference update occured. */

	void _limitYawRate(); /**< Limits the rate of change of the yaw setpoint. */
	bool _evaluateTriplets(); /**< Checks and sets triplets. */
	bool _isFinite(const position_setpoint_s &sp); /**< Checks if all waypoint triplets are finite. */
	bool _evaluateGlobalReference(); /**< Check is global reference is available. */
	State _getCurrentState(); /**< Computes the current vehicle state based on the vehicle position and navigator triplets. */
};
