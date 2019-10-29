/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file aqua_att_control_params.c
 * Parameters for aquatic attitude controller.
 *
 * @author Antoine Richard <antoine.richard@alerion.fr>
 */

/**
 * Yaw P gain
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 5
 * @decimal 2
 * @increment 0.1
 * @group Aquatic Attitude Control
 */
PARAM_DEFINE_FLOAT(AQUA_YAW_P, 2.8f);

/**
 * Yaw rate P gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.6
 * @decimal 2
 * @increment 0.01
 * @group Aquatic Attitude Control
 */
PARAM_DEFINE_FLOAT(AQUA_YAWRATE_P, 0.2f);

/**
 * Yaw rate I gain
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Aquatic Attitude Control
 */
PARAM_DEFINE_FLOAT(AQUA_YAWRATE_I, 0.1f);

/**
 * Yaw rate integrator limit
 *
 * Yaw rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large yaw moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Aquatic Attitude Control
 */
PARAM_DEFINE_FLOAT(AQUA_YR_INT_LIM, 0.30f);

/**
 * Yaw rate D gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Aquatic Attitude Control
 */
PARAM_DEFINE_FLOAT(AQUA_YAWRATE_D, 0.0f);

/**
 * Yaw rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.01
 * @group Aquatic Attitude Control
 */
PARAM_DEFINE_FLOAT(AQUA_YAWRATE_FF, 0.0f);

/**
 * Yaw rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = AQUA_YAWRATE_K * (AQUA_YAWRATE_P * error
 * 			     + AQUA_YAWRATE_I * error_integral
 * 			     + AQUA_YAWRATE_D * error_derivative)
 * Set AQUA_YAWRATE_P=1 to implement a PID in the ideal form.
 * Set AQUA_YAWRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Aquatic Attitude Control
 */
PARAM_DEFINE_FLOAT(AQUA_YAWRATE_K, 1.0f);

/**
 * Max yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Aquatic Attitude Control
 */
PARAM_DEFINE_FLOAT(AQUA_YAWR_MAX, 90.0f);

/**
 * Cutoff frequency for the low pass filter on the D-term in the rate controller
 *
 * The D-term uses the derivative of the rate and thus is the most susceptible to noise.
 * Therefore, using a D-term filter allows to decrease the driver-level filtering, which
 * leads to reduced control latency and permits to increase the P gains.
 * A value of 0 disables the filter.
 *
 * @unit Hz
 * @min 0
 * @max 1000
 * @decimal 0
 * @increment 10
 * @group Aquatic Attitude Control
 */
PARAM_DEFINE_FLOAT(AQUA_DTERM_CUT, 0.f);

/**
 * Maximum throttle input in manual aquatic mode
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Aquatic Attitude Control
 */
PARAM_DEFINE_FLOAT(AQUA_THR_MAX_MAN, 0.1f);

