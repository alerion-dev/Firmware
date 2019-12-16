/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file hydradrone_commander_main.cpp
 *
 * Hydradrone commander for mode switching.
 *
 * @author Antoine Richard		<antoine.richard@alerion.fr>
 *
 */

#include "hydradrone_commander.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

static constexpr auto STATUS_UNKNOWN = hydradrone_status_s::HYDRADRONE_STATUS_UNKNOWN;
static constexpr auto STATUS_MC = hydradrone_status_s::HYDRADRONE_STATUS_MC;
static constexpr auto STATUS_AQUA = hydradrone_status_s::HYDRADRONE_STATUS_AQUA;
static constexpr auto STATUS_TRANSIENT = hydradrone_status_s::HYDRADRONE_STATUS_TRANSIENT;

HydradroneCommander::HydradroneCommander() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, "hydradrone_commander")),
	_state_machine(&HydradroneCommander::_state_machine_finished)
{
	parameters_updated();
}

HydradroneCommander::~HydradroneCommander() {
	perf_free(_loop_perf);
}

bool HydradroneCommander::init() {
	if (!_manual_control_sp_sub.registerCallback() ||
		!_vehicle_status_sub.registerCallback() ||
		!_hydradrone_status_sub.registerCallback() ||
		!_vehicle_command_sub.registerCallback())
	{
		PX4_ERR("Workitem callbacks registration failed!");
		return false;
	}

	return true;
}

void HydradroneCommander::parameters_updated() {
	_state_machine.set_loc_rot_time(_param_hydra_lock_time.get(), _param_hydra_rot_time.get());
}

void HydradroneCommander::parameter_update_poll() {
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
		parameters_updated();
	}
}

void HydradroneCommander::_publish_hydradrone_status() {
	auto& status_s = _hydradrone_status_pub.get();
	status_s.timestamp = hrt_absolute_time();
	status_s.status = _status;
	if(!_hydradrone_status_pub.update()) {
		PX4_ERR("Couldn't publish in hydradrone status topic");
	}
}

void HydradroneCommander::_state_machine_finished(uint8_t final_status) {
	auto cmdr = ModuleBase<HydradroneCommander>::get_instance();	// Get self
	if(cmdr == nullptr)  {
		PX4_PANIC("No instance of Hydradrone Commander running but State Machine callback called!!");
		return;
	}
	cmdr->_status = final_status;
	cmdr->_publish_hydradrone_status();
	PX4_INFO("State machine finished transition to state %u", final_status);
}

void HydradroneCommander::Run()
{
	if (should_exit()) {
		_manual_control_sp_sub.unregisterCallback();
		_vehicle_status_sub.unregisterCallback();
		_hydradrone_status_sub.unregisterCallback();
		_vehicle_command_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	bool mode_command_received = false;
	if(_vehicle_command_sub.update(&_vehicle_command)) {
		if(_vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_MODE_AQUATIC) {
			_desired = STATUS_AQUA;
			mode_command_received = true;
		} else if (_vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_MODE_MULTIROTOR) {
			_desired = STATUS_MC;
			mode_command_received = true;
		}
	}

	if (_manual_control_sp_sub.update(&_manual_control_sp) ||
		_vehicle_status_sub.update(&_vehicle_status) ||
		_hydradrone_status_sub.update(&_hydradrone_status) ||
		mode_command_received) {

		// Check that our status was not overriden by other modules
		if(_hydradrone_status.status != _status) {
			_publish_hydradrone_status();
		}

		bool disarmed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY;

		// Somebody has published a valid desired state.
		if(mode_command_received) {
			if(_status == STATUS_TRANSIENT) {
				PX4_INFO("Hydradrone commander: Waiting for transition to finish before switching mode");
			}
			if(!disarmed) {
				PX4_INFO("Hydradrone commander: Waiting for disarm before switching mode");
			}
		}

		parameter_update_poll();

		// Only automatically switch modes if we are in standby (disarmed)
		if(disarmed) {
			bool manual_aqua = _manual_control_sp.transition_switch == manual_control_setpoint_s::SWITCH_POS_ON;
			bool manual_mc = _manual_control_sp.transition_switch == manual_control_setpoint_s::SWITCH_POS_OFF;

			switch (_status) {
			case STATUS_UNKNOWN:
				if(manual_aqua) {
					_go_to_state(STATUS_AQUA);
				} else if(manual_mc) {
					_go_to_state(STATUS_MC);
				} else {
					if(_desired != STATUS_UNKNOWN) {
						_go_to_state(_desired);
					} else {
						// Default state if nothing is specified
						_go_to_state(STATUS_MC);
					}
				}
				break;
			case STATUS_MC:
				if(manual_aqua) {
					_go_to_state(STATUS_AQUA);
				} else if(manual_mc) {
				} else {
					if(_desired == STATUS_AQUA) {
						_go_to_state(STATUS_AQUA);
					}
				}
				break;
			case STATUS_AQUA:
				if(manual_aqua) {
				} else if(manual_mc) {
					_go_to_state(STATUS_MC);
				} else {
					if(_desired == STATUS_MC) {
						_go_to_state(STATUS_MC);
					}
				}
				break;
			case STATUS_TRANSIENT:
				break;
			}
		}
	}

	perf_end(_loop_perf);
}

bool HydradroneCommander::_go_to_state(uint8_t desired_state) {
	uint8_t previous_state = _status;
	_status = hydradrone_status_s::HYDRADRONE_STATUS_TRANSIENT;
	_publish_hydradrone_status();

	_state_machine.transition_to_state(previous_state, desired_state);

	PX4_INFO("Transitioning from state %d to state %d", previous_state, desired_state);

	return true;
}


int HydradroneCommander::task_spawn(int argc, char *argv[]) {
	HydradroneCommander *instance = new HydradroneCommander();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int HydradroneCommander::print_status() {
	PX4_INFO("Running");

	switch(_hydradrone_status.status){
	case hydradrone_status_s::HYDRADRONE_STATUS_UNKNOWN:
		PX4_INFO("Hydradrone: Unknown");
		break;
	case hydradrone_status_s::HYDRADRONE_STATUS_MC:
		PX4_INFO("Hydradrone: Multicopter mode");
		break;
	case hydradrone_status_s::HYDRADRONE_STATUS_AQUA:
		PX4_INFO("Hydradrone: Aquatic mode");
		break;
	case hydradrone_status_s::HYDRADRONE_STATUS_TRANSIENT:
		PX4_INFO("Hydradrone: Transient");
		break;
	default:
		PX4_ERR("Unrecognized hydradrone status!");
		break;
	}

	perf_print_counter(_loop_perf);

	return 0;
}

int HydradroneCommander::custom_command(int argc, char *argv[]) {
	if(!strcmp(argv[0], "switch-mc")) {
		if(!ModuleBase<HydradroneCommander>::is_running()) {
			PX4_ERR("Cannot switch hydradrone mode, hydradrone_commander not running");
			return -1;
		}
		ModuleBase<HydradroneCommander>::get_instance()->_go_to_state(hydradrone_status_s::HYDRADRONE_STATUS_MC);
	} else if(!strcmp(argv[0], "switch-aqua")) {
		if(!ModuleBase<HydradroneCommander>::is_running()) {
			PX4_ERR("Cannot switch hydradrone mode, hydradrone_commander not running");
			return -1;
		}
		ModuleBase<HydradroneCommander>::get_instance()->_go_to_state(hydradrone_status_s::HYDRADRONE_STATUS_AQUA);
	} else {
		print_usage("unknown command");
	}
	return 0;
}

int HydradroneCommander::print_usage(const char *reason) {
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the hydradrone commander. It handles the mode switching between the multicopter and aquatic modes.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hydradrone_commander", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND_DESCR("switch-mc", "switches to multicopter");
	PRINT_MODULE_USAGE_COMMAND_DESCR("switch-aqua", "switches to aquatic");

	return 0;
}

int hydradrone_commander_main(int argc, char *argv[]) {
	return HydradroneCommander::main(argc, argv);
}
