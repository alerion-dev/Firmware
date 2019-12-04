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
 * @file FlightTaskAquaticAutoLineSmooth.hpp
 *
 * Flight task for autonomous, gps driven mode on aquatic drone. The vehicle flies
 * along a straight line in between waypoints.
 */

#pragma once

#include "VelocitySmoothing.hpp"

#include "FlightTaskAquaticAuto.hpp"

class FlightTaskAquaticAutoLineSmooth : public FlightTaskAquaticAuto
{
public:
	FlightTaskAquaticAutoLineSmooth() = default;
	virtual ~FlightTaskAquaticAutoLineSmooth() = default;

	bool activate(vehicle_local_position_setpoint_s last_setpoint) override;
	void reActivate() override;

	bool update() override;

protected:

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskAquaticAuto,
					(ParamFloat<px4::params::AQUA_VEL_MAX>) _param_aqua_vel_max,
					(ParamFloat<px4::params::AQUA_ACC_FWD>) _param_aqua_acc_fwd_max,
					(ParamFloat<px4::params::AQUA_ACC_BWD>) _param_aqua_acc_bwd, // breaking by reversing thrust direction
					(ParamFloat<px4::params::AQUA_JERK_MAX>) _param_aqua_jerk_max,
					(ParamFloat<px4::params::AQUA_TRAJ_P>) _param_aqua_traj_p
				       );

	void checkSetpoints(vehicle_local_position_setpoint_s &setpoints);

	/** Reset position or velocity setpoints in case of EKF reset event */
	void _ekfResetHandlerPositionXY() override;
	void _ekfResetHandlerVelocityXY() override;
	void _ekfResetHandlerHeading(float delta_psi) override;

	void _prepareVelocitySetpoints();

	void _generateSetpoints(); /**< Generate setpoints along line. */
	void _generateHeading();
	bool _generateHeadingAlongTraj(); /**< Generates heading along trajectory. */

	inline float _constrainOneSide(float val, float constraint); /**< Constrain val between INF and constraint */
	inline float _constrainAbs(float val, float min, float max); /**< Constrain absolute value of val between min and max */

	float _getSpeedAtTarget();
	float _getMaxSpeedFromDistance(float braking_distance);

	void _prepareSetpoints(); /**< Generate velocity target points for the trajectory generator. */
	void _updateTrajConstraints();
	void _generateTrajectory();

	/** No takeoff in aquatic tasks */
	bool _checkTakeoff() override { return false; };

	orb_advert_t _log_publisher = nullptr;

	VelocitySmoothing _trajectory[2]; ///< Trajectories in x and y directions
};
