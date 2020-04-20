/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

#include <lib/mixer/mixer.h>
#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/hydradrone_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_land_detected.h>

#include "StateMachine.hpp"

/**
 * Hydradrone commander control app start / stop handling function
 */
extern "C" __EXPORT int hydradrone_commander_main(int argc, char *argv[]);

/**
 * This module listens on the manual control topic for now
 * it reads a custom switch on the RC.
 */
class HydradroneCommander : public ModuleBase<HydradroneCommander>, public ModuleParams,
	public px4::WorkItem
{
public:
	HydradroneCommander();

	virtual ~HydradroneCommander();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();

private:

	/**
	 * Check for parameter update and handle it.
	 */
	void parameter_update_poll();
	void parameters_updated();

	void _publish_hydradrone_status();
	void _acknowledge_command(uint8_t result);

	bool _go_to_state(uint8_t desired_state);

	static void _state_machine_finished(uint8_t final_status);

	uORB::SubscriptionCallbackWorkItem _hydradrone_status_sub{this, ORB_ID(hydradrone_status)};		/**< hydradrone status subscription */
	uORB::SubscriptionCallbackWorkItem _vehicle_status_sub{this, ORB_ID(vehicle_status)};
	uORB::SubscriptionCallbackWorkItem _manual_control_sp_sub{this, ORB_ID(manual_control_setpoint)};
	uORB::SubscriptionCallbackWorkItem _vehicle_command_sub{this, ORB_ID(vehicle_command)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};		/**< parameter updates subscription */
	uORB::SubscriptionData<vehicle_land_detected_s> _land_detector_sub{ORB_ID(vehicle_land_detected)};

	uORB::PublicationData<hydradrone_status_s> _hydradrone_status_pub{ORB_ID(hydradrone_status)};
	uORB::PublicationData<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};

	struct manual_control_setpoint_s _manual_control_sp{};
	struct vehicle_status_s _vehicle_status{};
	struct hydradrone_status_s _hydradrone_status{};
	struct vehicle_command_s _vehicle_command{};

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	uint8_t _status = hydradrone_status_s::HYDRADRONE_STATUS_UNKNOWN;
	uint8_t _desired = hydradrone_status_s::HYDRADRONE_STATUS_UNKNOWN;
	HydradroneStateMachine _state_machine;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::HYDRA_LCK_TIME>) _param_hydra_lock_time,
		(ParamFloat<px4::params::HYDRA_ROT_TIME>) _param_hydra_rot_time
	)
};

