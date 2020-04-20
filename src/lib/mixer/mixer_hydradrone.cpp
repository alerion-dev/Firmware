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

HydradroneMixer::HydradroneMixer(ControlCallback control_cb,
			 uintptr_t cb_handle,
			 MultirotorMixer* multirotor,
			 AquaticMixer* aquatic) :
	Mixer(control_cb, cb_handle),
	_multirotor(multirotor),
	_aquatic(aquatic)
{
}

HydradroneMixer::~HydradroneMixer()
{
	if (_multirotor != nullptr) {
		delete _multirotor;
	}
	if(_aquatic != nullptr) {
		delete _aquatic;
	}
}

HydradroneMixer*
HydradroneMixer::from_text(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf, unsigned &buflen)
{
	HydradroneMixer *hm = nullptr;
	MultirotorMixer* mm = nullptr;
	AquaticMixer* am = nullptr;
	const char *end = buf + buflen;

	am = AquaticMixer::from_text(control_cb, cb_handle, buf, buflen);
	if(am == nullptr) {
		goto out;
	}

	/* update buffer start address using buflen */
	buf = end - buflen;

	while (buflen > 2 && (buf[0] != 'A' || buf[1] != 'R' || buf[2] != ':')) {
		buflen--;
		buf++;
	}
	if (buflen <= 2) {
		debug("hydradrone has to have an aquatic (A:) and multirotor (AR:) definition");
		goto out;
	}
	buflen--;	// Buf starts with AR: but multirotor parser needs R:
	buf++;

	mm = MultirotorMixer::from_text(control_cb, cb_handle, buf, buflen);
	if(mm == nullptr) {
		goto out;
	}

	hm = new HydradroneMixer(control_cb, cb_handle, mm, am);

	if (hm != nullptr) {
		debug("loaded hydradrone mixer");
	} else {
		debug("could not allocate memory for mixer");
	}

out:
	if(am != nullptr) delete am;
	if(mm != nullptr) delete mm;

	return hm;
}

unsigned
HydradroneMixer::mix(float *outputs, unsigned space)
{
	unsigned mix_count = _multirotor->mix(outputs, space);

	if(mix_count == 0) return 0;	// Not enough space for aquatic, abort

	float* outputs_aquatic = (float*)malloc(space * sizeof(float));
	unsigned aquatic_count = _aquatic->mix(outputs_aquatic, space);

	if(aquatic_count == 0) {	// Not enough space for aquatic, abort
		return 0;
	} else if(aquatic_count > mix_count) {
		mix_count = aquatic_count;
	}

	// Sum both mix outputs. This assumes only one is outputting non-null values at a time.
	for(unsigned i = 0; i < aquatic_count; ++i) {
		outputs[i] += outputs_aquatic[i];
	}

	return mix_count;
}

uint16_t
HydradroneMixer::get_saturation_status()
{
	return _multirotor->get_saturation_status() | _aquatic->get_saturation_status();
}

void
HydradroneMixer::groups_required(uint32_t &groups)
{
	_multirotor->groups_required(groups);
	_aquatic->groups_required(groups);
}
