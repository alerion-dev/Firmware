/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file mixer_simple.cpp
 *
 * Simple summing mixer.
 */

#include "mixer.h"
#include <stdio.h>
#include <stdlib.h>

#define debug(fmt, args...)	do { } while(0)
//#define debug(fmt, args...)	do { printf("[mixer] " fmt "\n", ##args); } while(0)

AquaticMixer::AquaticMixer(ControlCallback control_cb,
		uintptr_t cb_handle,
		float yaw_scale) :
	Mixer(control_cb, cb_handle)
{
	static_assert(sizeof(_rotor_scales) / sizeof(RotorScales) == _rotor_count);
	for(unsigned i = 0; i < sizeof(_left_rotors) / sizeof(unsigned); ++i) {
		_rotor_scales[_left_rotors[i]].yaw_scale = yaw_scale;
	}
	for(unsigned i = 0; i < sizeof(_right_rotors) / sizeof(unsigned); ++i) {
		_rotor_scales[_right_rotors[i]].yaw_scale = -yaw_scale;
	}
	for(unsigned i = 0; i < sizeof(_thrust_rotors) / sizeof(unsigned); ++i) {
		_rotor_scales[_thrust_rotors[i]].thrust_scale = 1.f;
	}
}

AquaticMixer*
AquaticMixer::from_text(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf, unsigned &buflen)
{
	AquaticMixer *am = nullptr;
	int s[1];

	/* enforce that the mixer ends with a new line */
	if (!string_well_formed(buf, buflen)) {
		return nullptr;
	}

	/* get the base info for the mixer */
	if (sscanf(buf, "A: %d", &s[0]) != 1) {
		debug("simple parse failed on '%s'", buf);
		goto out;
	}

	buf = skipline(buf, buflen);

	if (buf == nullptr) {
		debug("no line ending, line is incomplete");
		goto out;
	}

	am = new AquaticMixer(
		control_cb,
		cb_handle,
		s[0] / 10000.f);

	if (am != nullptr) {
		debug("loaded aquatic mixer");
	} else {
		debug("could not allocate memory for mixer");
	}

out:
	return am;
}

unsigned AquaticMixer::mix(float *outputs, unsigned space) {
	if (space < _rotor_count) {
		return 0;
	}

	float yaw = math::constrain(get_control(1, 2), -1.0f, 1.0f);

	// Non-reversible thrust is limited to [0, 1]
	float thrust_min = _reversible_thrust ? -1.f : 0.f;
	float thrust = math::constrain(get_control(1, 3), thrust_min, 1.0f);

	// clean out class variable used to capture saturation
	_saturation_status.value = 0;

	// Compute required thrust offsets to obey yaw command
	float thrust_offset_pos = 0.f, thrust_offset_neg = 0.f;
	for (unsigned i = 0; i < _rotor_count; i++) {
		float thrust_normalized = thrust * _rotor_scales[i].thrust_scale;
		if(!_reversible_thrust) thrust_normalized = thrust_normalized * 2.f - 1.f;
		float cmd = thrust_normalized + yaw * _rotor_scales[i].yaw_scale;

		if(cmd < -1.f) {
			thrust_offset_pos = math::max(thrust_offset_pos, -1.f - cmd);
			_saturation_status.flags.motor_neg = true;
			update_saturation_status(i, false, true);
		} else if(cmd > 1.f) {
			thrust_offset_neg = math::min(thrust_offset_neg, 1.f - cmd);
			_saturation_status.flags.motor_pos = true;
			update_saturation_status(i, true, false);
		}
	}
	if(thrust_offset_pos > 0.f && thrust_offset_neg < 0.f) {
		// This means that we hit thrust floor and ceiling: shouldn't happen, set outputs to 0
		for (unsigned i = 0; i < _rotor_count; i++) outputs[i] = -1.f;
		return _rotor_count;
	}

	// Thrust is in [0, 1] so offsets are doubled in cmd: divide them by two before
	if(!_reversible_thrust) {
		thrust_offset_pos /= 2.f;
		thrust_offset_neg /= 2.f;
	}

	// Apply offset thrust and yaw commands to outputs
	for (unsigned i = 0; i < _rotor_count; i++) {
		float cmd_thrust = (thrust + thrust_offset_pos + thrust_offset_neg) * _rotor_scales[i].thrust_scale;
		if(!_reversible_thrust) cmd_thrust = cmd_thrust * 2.f - 1.f;
		outputs[i] = cmd_thrust + yaw * _rotor_scales[i].yaw_scale;
	}

	return _rotor_count;
}

void AquaticMixer::update_saturation_status(unsigned i, bool clipping_high, bool clipping_low) {
	if(clipping_high) {
		// check if the yaw input is saturating
		if (_rotor_scales[i].yaw_scale > 0.0f) {
			// A positive change in yaw will increase saturation
			_saturation_status.flags.yaw_pos |= true;

		} else if (_rotor_scales[i].yaw_scale < 0.0f) {
			// A negative change in yaw will increase saturation
			_saturation_status.flags.yaw_neg |= true;
		}

		if (_rotor_scales[i].thrust_scale > 0.0f) {
			// A positive change in thrust will increase saturation
			_saturation_status.flags.thrust_pos |= true;
		}
	} else if(clipping_low) {
		// check if the yaw input is saturating
		if (_rotor_scales[i].yaw_scale < 0.0f) {
			// A positive change in yaw will increase saturation
			_saturation_status.flags.yaw_pos |= true;

		} else if (_rotor_scales[i].yaw_scale > 0.0f) {
			// A negative change in yaw will increase saturation
			_saturation_status.flags.yaw_neg |= true;
		}

		if (_rotor_scales[i].thrust_scale > 0.0f) {
			// A negative change in thrust will increase saturation
			_saturation_status.flags.thrust_neg |= true;
		}
	}
	_saturation_status.flags.valid = true;
}

uint16_t
AquaticMixer::get_saturation_status()
{
	return _saturation_status.value;
}

void
AquaticMixer::groups_required(uint32_t &groups)
{
	// We only need ctrl group 1 (AUX controls) for the aquatic mixer.
	groups |= 1 << 1;
}

