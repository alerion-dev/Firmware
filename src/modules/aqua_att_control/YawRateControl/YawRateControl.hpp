/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file YawRateControl.hpp
 *
 * PID yaw angular rate / angular velocity control.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include <uORB/topics/rate_ctrl_status.h>

class YawRateControl
{
public:
	YawRateControl() = default;
	~YawRateControl() = default;

	/**
	 * Set the rate control gains
	 * @param P proportional gain for body z axis: yaw
	 * @param I integral gain
	 * @param D derivative gain
	 */
	void setGains(const float P, const float I, const float D);

	/**
	 * Set the maximum absolute value of the integrator
	 * @param integrator_limit limit value
	 */
	void setIntegratorLimit(const float integrator_limit) { _lim_int = integrator_limit; };

	/**
	 * Set update frequency and low-pass filter cutoff that is applied to the derivative term
	 * @param loop_rate [Hz] rate with which update function is called
	 * @param cutoff [Hz] cutoff frequency for the low-pass filter on the dervative term
	 * @param force flag to force an expensive update even if the cutoff didn't change
	 */
	void setDTermCutoff(const float loop_rate, const float cutoff, const bool force);

	/**
	 * Set direct rate to torque feed forward gain
	 * @see _gain_ff
	 * @param FF feed forward gains for body z axis
	 */
	void setFeedForwardGain(const float FF) { _gain_ff = FF; };

	/**
	 * Run one control loop cycle calculation
	 * @param yawrate estimation of the current vehicle yaw rate
	 * @param yawrate_sp desired vehicle yaw rate setpoint
	 * @param dt time delta
	 * @return [-1,1] normalized yaw torque to apply to the vehicle
	 */
	float update(const float yawrate, const float yawrate_sp, const float dt);

	/**
	 * Set the integral term to 0 to prevent windup
	 * @see _rate_int
	 */
	void resetIntegral() { _rate_int = 0.0f; }

	/**
	 * Get status message of controller for logging/debugging
	 * @param rate_ctrl_status status message to fill with internal states
	 */
	void getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status);

private:
	void updateIntegral(const float rate_error, const float dt);

	// Gains
	float _gain_p; ///< rate control proportional gain
	float _gain_i; ///< rate control integral gain
	float _gain_d; ///< rate control derivative gain
	float _lim_int; ///< integrator term maximum absolute value
	float _gain_ff; ///< direct rate to torque feed forward gain useful to counteract water drag

	// States
	float _rate_prev; ///< angular rates of previous update
	float _rate_prev_filtered; ///< low-pass filtered angular rates of previous update
	float _rate_int; ///< integral term of the rate controller
	math::LowPassFilter2p _lp_filter_d{0.f, 0.f}; ///< low-pass filters for D-term
};
