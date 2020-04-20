#include "StateMachine.hpp"

#include <px4_log.h>
#include <lib/mathlib/mathlib.h>


HydradroneStateMachine::HydradroneStateMachine(HydradroneStateMachine::TransitionCallback tcb) :
	ScheduledWorkItem("hydradrone_state_machine", px4::wq_configurations::lp_default),
	_tcb(tcb) {}

void HydradroneStateMachine::set_loc_rot_time(float lock_time, float rot_time) {
	_lock_time = static_cast<uint32_t>(lock_time * 1e6f);	// Convertion to microseconds
	_rot_time = static_cast<uint32_t>(rot_time * 1e6f);
}

void HydradroneStateMachine::transition_to_state(uint8_t previous, uint8_t state) {
	if(!_ready) {
		PX4_ERR("Not ready to transition, currently transitioning to state %u", _target_state);
		return;
	}

	_ready = false;
	_next_transition = Transition::UNLOCK;
	switch (state) {
	case hydradrone_status_s::HYDRADRONE_STATUS_MC:
	/* FALLTHRU */
	case hydradrone_status_s::HYDRADRONE_STATUS_AQUA:
		_target_state = state;
		break;
	default:
		PX4_ERR("Cannot transition to state %u", state);
		_next_transition = Transition::NONE;
		_target_state = previous;	// Fallback to previous state
		break;
	}
	ScheduleNow();
}

void HydradroneStateMachine::_unlock() {
	_publish_actuators(false);
}

void HydradroneStateMachine::_lock() {
	_publish_actuators(true);
}

void HydradroneStateMachine::_rotate_arm(bool aquatic) {
	_publish_actuators(false, true, aquatic);
}

void HydradroneStateMachine::Run() {
	switch (_next_transition) {
	case Transition::NONE:
		_ready = true;	// Nothing to do, we're ready.
		PX4_DEBUG("Finished transition, triggering callback");
		_tcb(_target_state);	// Target state was reached, confirm
		break;
	case Transition::LOCK:
		_lock();
		_next_transition = Transition::NONE;
		PX4_DEBUG("Locking");
		ScheduleDelayed(_lock_time);
		break;
	case Transition::UNLOCK:
		_unlock();
		_next_transition = _get_rot_transition(_target_state);
		PX4_DEBUG("Unlocking");
		ScheduleDelayed(_lock_time);
		break;
	case Transition::ROTATE_MC:
		_rotate_arm(false);
		_next_transition = Transition::LOCK;
		PX4_DEBUG("rotating to MC");
		ScheduleDelayed(_rot_time);
		break;
	case Transition::ROTATE_AQUA:
		_rotate_arm(true);
		_next_transition = Transition::LOCK;
		PX4_DEBUG("Rotating to Aqua");
		ScheduleDelayed(_rot_time);
	}
}

void HydradroneStateMachine::_publish_actuators(bool lock, bool rotate, bool aquatic) {
	auto& actuator = _actuators_1_pub.get();
	actuator.timestamp = hrt_absolute_time();
	actuator.control[LOCK_ID] = lock ? LOCK_ON : LOCK_OFF;
	actuator.control[ROTATION_ID] = rotate ? (aquatic ? ROTATION_AQUATIC : ROTATION_MULTICOPTER) : ROTATION_FLOATING;
	actuator.control[PROBE_ID] = PROBE_RETRACT;
	if(!_actuators_1_pub.update()) {
		PX4_ERR("Couldn't publish in actuator control group 1");
	}
}

HydradroneStateMachine::Transition HydradroneStateMachine::_get_rot_transition(uint8_t target_state) {
	if(target_state == hydradrone_status_s::HYDRADRONE_STATUS_MC) return Transition::ROTATE_MC;
	if(target_state == hydradrone_status_s::HYDRADRONE_STATUS_AQUA) return Transition::ROTATE_AQUA;
	return Transition::LOCK;	// Invalid target state, lock for safety
}
