/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file aqua_pos_control_params.c
 * Parameters for aquatic position controller.
 *
 * @author Antoine Richard <antoine.richard@alerion.fr>
 */

/**
 * Proportional gain for horizontal position error
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Aquatic Position Control
 */
PARAM_DEFINE_FLOAT(AQUA_POS_P, 0.95f);

/**
 * Proportional gain for horizontal velocity error
 *
 * @min 0.06
 * @max 0.15
 * @decimal 2
 * @group Aquatic Position Control
 */
PARAM_DEFINE_FLOAT(AQUA_VEL_P, 0.09f);

/**
 * Integral gain for horizontal velocity error
 *
 * Non-zero value allows to eliminate steady state errors in the presence of disturbances like wind.
 *
 * @min 0.0
 * @max 3.0
 * @decimal 3
 * @group Aquatic Position Control
 */
PARAM_DEFINE_FLOAT(AQUA_VEL_I, 0.02f);

/**
 * Differential gain for horizontal velocity error. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.005
 * @max 0.1
 * @decimal 3
 * @group Aquatic Position Control
 */
PARAM_DEFINE_FLOAT(AQUA_VEL_D, 0.01f);

/**
 * Influence of thrust to forward vector angle on thrust reduction
 *
 * The higher the value, the more the drone will wait for alignment with the target before accelerating
 *
 * @min 2.0
 * @max 100.0
 * @decimal 1
 * @group Aquatic Position Control
 */
PARAM_DEFINE_FLOAT(AQUA_THR_ANGLE, 5.0f);

/**
 * Proportional gain for horizontal trajectory stifness
 *
 * @min 0.1
 * @max 1.0
 * @decimal 1
 * @group Aquatic Position Control
 */
PARAM_DEFINE_FLOAT(AQUA_TRAJ_P, 0.5f);

/**
 * Maximum horizontal velocity in mission
 *
 * Normal horizontal velocity in AUTO modes (includes
 * also RTL / hold / etc.) and endpoint for
 * position stabilized mode (POSCTRL).
 *
 * @unit m/s
 * @min 0.5
 * @max 5.0
 * @increment 0.1
 * @decimal 2
 * @group Aquatic Position Control
 */
PARAM_DEFINE_FLOAT(AQUA_VEL_MAX, 1.0f);

/**
 * Acceleration limit in forward direction for auto mode
 *
 * @unit m/s/s
 * @min 0.5
 * @max 5.0
 * @increment 0.1
 * @decimal 2
 * @group Aquatic Position Control
 */

PARAM_DEFINE_FLOAT(AQUA_ACC_FWD, 2.0f);

/**
 * Acceleration value in backwards direction for breaking in auto mode
 *
 * This is not a contrain on backwards acceleration but an indication of its value
 * For non-reversible motors, this value shall be small
 *
 * @unit m/s/s
 * @min 0.5
 * @max 5.0
 * @increment 0.1
 * @decimal 2
 * @group Aquatic Position Control
 */

PARAM_DEFINE_FLOAT(AQUA_ACC_BWD, 1.0f);

/**
 * Jerk limit in auto mode
 *
 * Limit the maximum jerk of the vehicle (how fast the acceleration can change).
 * A lower value leads to smoother vehicle motions, but it also limits its
 * agility.
 *
 * @unit m/s/s/s
 * @min 5.0
 * @max 80.0
 * @increment 1
 * @decimal 1
 * @group Aquatic Position Control
 */
PARAM_DEFINE_FLOAT(AQUA_JERK_MAX, 8.0f);

/**
 * Maximum thrust in auto thrust control
 *
 * Limit max allowed thrust
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Aquatic Position Control
 */
PARAM_DEFINE_FLOAT(AQUA_THR_MAX, 1.0f);

