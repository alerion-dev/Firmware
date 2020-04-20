
/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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
 * @file ControlMath.cpp
 */

#include "ControlMath.hpp"
#include <platforms/px4_defines.h>
#include <float.h>
#include <mathlib/mathlib.h>

using namespace matrix;

namespace ControlMath
{
vehicle_attitude_setpoint_s thrustToAttitude(const Vector2f &thr_sp,
	const Vector2f& forwardVector,
	const float thrustPow,
	const bool reversible_thrust)
{
	vehicle_attitude_setpoint_s att_sp = {};

	// thrust setpoint
	const float thrust_scalar = thr_sp.length();
	att_sp.thrust_body[0] = 0.0f;
	att_sp.thrust_body[1] = 0.0f;
	att_sp.thrust_body[2] = 0.0f;

	// desired body_z axis is vertical: drone is flat on the water
	Vector3f body_x, body_y, body_z{0.f, 0.f, 1.f};

	// Check for invalid or zero thrust
	if (PX4_ISFINITE(thr_sp(0)) && PX4_ISFINITE(thr_sp(1)) &&
		PX4_ISFINITE(forwardVector(0)) && PX4_ISFINITE(forwardVector(1)) &&
		thrust_scalar > 0.00001f) {

		body_x = Vector3f{thr_sp(0), thr_sp(1), 0.f};
		body_x /= thrust_scalar;

		// Thrust is modulated based on the dot product with forward
		const float thrustDotX = thr_sp * forwardVector / thrust_scalar;
		// If we are pointing opposite of thrust we should use reversing if available
		const bool use_reversing = reversible_thrust && (thrustDotX < -0.8f);

		if(thrustDotX > FLT_EPSILON || use_reversing) {
			att_sp.thrust_body[0] = powf(fabsf(thrustDotX), thrustPow) * thrust_scalar;
		} else {	// thrust perpendicular or slightly backwards has to stay 0
			att_sp.thrust_body[0] = 0.f;
		}

		if(use_reversing) {
			att_sp.thrust_body[0] = -att_sp.thrust_body[0];
			body_x = -body_x;
		}

	} else if(PX4_ISFINITE(forwardVector(0)) && PX4_ISFINITE(forwardVector(1))) {
		// invalid thrust keeps current orientation
		body_x = Vector3f{forwardVector(0), forwardVector(1), 0.f};
	} else {
		// Everything is invalid
		att_sp.q_d_valid = false;
		att_sp.yaw_body = NAN;
		return att_sp;
	}

	// desired body_y axis
	body_y = body_z % body_x;

	Dcmf R_sp;

	// fill rotation matrix
	for (int i = 0; i < 3; i++) {
		R_sp(i, 0) = body_x(i);
		R_sp(i, 1) = body_y(i);
		R_sp(i, 2) = body_z(i);
	}

	//copy quaternion setpoint to attitude setpoint topic
	Quatf q_sp = R_sp;
	q_sp.copyTo(att_sp.q_d);
	att_sp.q_d_valid = true;

	// for logging only, must not be used for control
	Eulerf euler{R_sp};
	att_sp.roll_body = 0.0f;
	att_sp.pitch_body = 0.0f;
	att_sp.yaw_body = euler(2);

	return att_sp;
}

Vector2f constrainXYaqua(const Vector2f &v0, const Vector2f &v1, const float &max)
{
	if (Vector2f(v0 + v1).norm() <= max) {
		// vector does not exceed maximum magnitude
		return v0 + v1;

	} else if (v0.length() >= max) {
		// the magnitude along v0, which has priority, already exceeds maximum.
		return v0.normalized() * max;

	} else if (fabsf(Vector2f(v1 - v0).norm()) < 0.001f) {
		// the two vectors are equal
		return v0.normalized() * max;

	} else if (v0.length() < 0.001f) {
		// the first vector is 0.
		return v1.normalized() * max;

	} else {
		// vf = final vector with ||vf|| <= max
		// s = scaling factor
		// u1 = unit of v1
		// vf = v0 + v1 = v0 + s * u1
		// constraint: ||vf|| <= max
		//
		// solve for s: ||vf|| = ||v0 + s * u1|| <= max
		//
		// Derivation:
		// For simplicity, replace v0 -> v, u1 -> u
		// 				   		   v0(0/1/2) -> v0/1/2
		// 				   		   u1(0/1/2) -> u0/1/2
		//
		// ||v + s * u||^2 = (v0+s*u0)^2+(v1+s*u1)^2+(v2+s*u2)^2 = max^2
		// v0^2+2*s*u0*v0+s^2*u0^2 + v1^2+2*s*u1*v1+s^2*u1^2 + v2^2+2*s*u2*v2+s^2*u2^2 = max^2
		// s^2*(u0^2+u1^2+u2^2) + s*2*(u0*v0+u1*v1+u2*v2) + (v0^2+v1^2+v2^2-max^2) = 0
		//
		// quadratic equation:
		// -> s^2*a + s*b + c = 0 with solution: s1/2 = (-b +- sqrt(b^2 - 4*a*c))/(2*a)
		//
		// b = 2 * u.dot(v)
		// a = 1 (because u is normalized)
		// c = (v0^2+v1^2+v2^2-max^2) = -max^2 + ||v||^2
		//
		// sqrt(b^2 - 4*a*c) =
		// 		sqrt(4*u.dot(v)^2 - 4*(||v||^2 - max^2)) = 2*sqrt(u.dot(v)^2 +- (||v||^2 -max^2))
		//
		// s1/2 = ( -2*u.dot(v) +- 2*sqrt(u.dot(v)^2 - (||v||^2 -max^2)) / 2
		//      =  -u.dot(v) +- sqrt(u.dot(v)^2 - (||v||^2 -max^2))
		// m = u.dot(v)
		// s = -m + sqrt(m^2 - c)
		//
		//
		//
		// notes:
		// 	- s (=scaling factor) needs to be positive
		// 	- (max - ||v||) always larger than zero, otherwise it never entered this if-statement

		Vector2f u1 = v1.normalized();
		float m = u1.dot(v0);
		float c = v0.dot(v0) - max * max;
		float s = -m + sqrtf(m * m - c);
		return v0 + u1 * s;
	}
}

}
