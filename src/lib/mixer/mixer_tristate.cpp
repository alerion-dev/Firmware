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
 * @file mixer_tristate.cpp
 *
 * Tristate mixer. -1.0 / 0.0 / 1.0 --> NAN / OFF / ON.
 */

#include "mixer.h"
#include <stdio.h>
#include <stdlib.h>

#define debug(fmt, args...)	do { } while(0)
//#define debug(fmt, args...)	do { printf("[mixer] " fmt "\n", ##args); } while(0)

TristateMixer::TristateMixer(ControlCallback control_cb,
		uintptr_t cb_handle,
		uint8_t input_group,
		uint8_t input_index,
		float off_value,
		float on_value) :
	Mixer(control_cb, cb_handle),
	_input_group(input_group),
	_input_index(input_index),
	_off_value(off_value),
	_on_value(on_value)
{}

TristateMixer*
TristateMixer::from_text(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf, unsigned &buflen)
{
	TristateMixer *tm = nullptr;

	unsigned u[2];
	int s[2];

	/* enforce that the mixer ends with a new line */
	if (!string_well_formed(buf, buflen)) {
		return nullptr;
	}

	/* get the base info for the mixer */
	if (sscanf(buf, "T: %u %u %d %d", &u[0], &u[1], &s[0], &s[1]) != 4) {
		debug("simple parse failed on '%s'", buf);
		goto out;
	}

	buf = skipline(buf, buflen);

	if (buf == nullptr) {
		debug("no line ending, line is incomplete");
		goto out;
	}

	tm = new TristateMixer(
		control_cb,
		cb_handle,
		u[0],
		u[1],
		s[0] / 10000.f,
		s[1] / 10000.f);

	if (tm != nullptr) {
		debug("loaded tristate mixer");
	} else {
		debug("could not allocate memory for mixer");
	}

out:
	return tm;
}

unsigned TristateMixer::mix(float *outputs, unsigned space) {
	if (space < 1) {
		return 0;
	}

	float input = get_control(_input_group, _input_index);
	if (input < -0.5f) {
		*outputs = NAN;
	} else if (input < 0.5f) {
		*outputs = _off_value;
	} else {
		*outputs = _on_value;
	}

	return 1u;
}

