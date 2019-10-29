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
 * @file YawRateControl.cpp
 */

#include <YawRateControl.hpp>
#include <px4_defines.h>

void YawRateControl::setGains(const float P, const float I, const float D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

void YawRateControl::setDTermCutoff(const float loop_rate, const float cutoff, const bool force)
{
	// only do expensive filter update if the cutoff changed
	if (force || fabsf(_lp_filter_d.get_cutoff_freq() - cutoff) > 0.01f) {
		_lp_filter_d.set_cutoff_frequency(loop_rate, cutoff);
		_lp_filter_d.reset(_rate_prev);
	}
}

float YawRateControl::update(const float yawrate, const float yawrate_sp, const float dt)
{
	// angular rates error
	float rate_error = yawrate_sp - yawrate;

	// prepare D-term based on low-pass filtered rates
	float rate_filtered(_lp_filter_d.apply(rate));
	float rate_d;

	if (dt > FLT_EPSILON) {
		rate_d = (rate_filtered - _rate_prev_filtered) / dt;
	}

	// PID control with feed forward
	float torque = _gain_p * rate_error + _rate_int - _gain_d * rate_d + _gain_ff * rate_sp;

	_rate_prev = rate;
	_rate_prev_filtered = rate_filtered;

	updateIntegral(rate_error, dt);

	return torque;
}

void YawRateControl::updateIntegral(const float rate_error, const float dt)
{
	// Perform the integration using a first order method
	float rate_i = _rate_int + _gain_i * rate_error * dt;

	// do not propagate the result if out of range or invalid
	if (PX4_ISFINITE(rate_i)) {
		_rate_int = math::constrain(rate_i, -_lim_int, _lim_int);
	}
}

void YawRateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = 0.0f;
	rate_ctrl_status.pitchspeed_integ = 0.0f;
	rate_ctrl_status.yawspeed_integ = _rate_int;
}
