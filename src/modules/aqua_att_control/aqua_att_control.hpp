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
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/hydradrone_status.h>

#include <YawControl.hpp>
#include <YawRateControl.hpp>

/**
 * Aquatic attitude control app start / stop handling function
 */
extern "C" __EXPORT int aqua_att_control_main(int argc, char *argv[]);

class AquaticAttitudeControl : public ModuleBase<AquaticAttitudeControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	AquaticAttitudeControl();

	virtual ~AquaticAttitudeControl();

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
	 * initialize some vectors/matrices from parameters
	 */
	void		parameters_updated();

	/**
	 * Check for parameter update and handle it.
	 */
	void		parameter_update_poll();
	bool		vehicle_attitude_poll();

	void		publish_actuator_controls();
	void		publish_rates_setpoint();
	void		publish_rate_controller_status();

	float		throttle_curve(float throttle_stick_input);


	/**
	 * Attitude controller.
	 */
	void		control_attitude();

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rate(float dt, const float yawrate);

	YawControl _yaw_control; ///< class for yaw control calculations
	YawRateControl _yaw_rate_control; ///< class for rate control calculations

	uORB::Subscription _v_att_sub{ORB_ID(vehicle_attitude)};			/**< vehicle attitude subscription */
	uORB::Subscription _v_att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};		/**< vehicle attitude setpoint subscription */
	uORB::Subscription _v_rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};		/**< vehicle rates setpoint subscription */
	uORB::Subscription _v_control_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle control mode subscription */
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};		/**< parameter updates subscription */
	uORB::Subscription _manual_control_sp_sub{ORB_ID(manual_control_setpoint)};	/**< manual control setpoint subscription */
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};			/**< battery status subscription */
	uORB::Subscription _hydradrone_status_sub{ORB_ID(hydradrone_status)};		/**< hydradrone status subscription */

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status), ORB_PRIO_DEFAULT};	/**< controller status publication */
	uORB::Publication<vehicle_rates_setpoint_s>	_v_rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};			/**< rate setpoint publication */

	orb_advert_t	_actuators_1_pub{nullptr};		/**< yaw + thrust actuator controls publication */

	bool		_actuators_1_circuit_breaker_enabled{false};	/**< circuit breaker to suppress output */

	struct vehicle_attitude_s		_v_att {};		/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_v_att_sp {};		/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp {};		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp {};	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode {};	/**< vehicle control mode */
	struct actuator_controls_s		_actuators {};		/**< actuator controls */
	struct battery_status_s			_battery_status {};	/**< battery status */
	struct hydradrone_status_s		_hydradrone_status {};	/**< hydradrone status */

	MultirotorMixer::saturation_status _saturation_status{};

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	static constexpr float initial_update_rate_hz = 250.f; /**< loop update rate used for initialization */
	float _loop_update_rate_hz{initial_update_rate_hz};          /**< current rate-controller loop update rate in [Hz] */

	float _yawrate_sp;			/**< angular rates setpoint */
	float _yaw_sp;

	float _yaw_torque{0.0f};	/**< yaw control */
	float _thrust_sp{0.0f};		/**< thrust setpoint on +X axis*/

	hrt_abstime _task_start{hrt_absolute_time()};
	hrt_abstime _last_run{0};

	// Yaw rate controller d-filtering setup
	float _dt_accumulator{0.0f};
	int _loop_counter{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::AQUA_YAW_P>) _param_aqua_yaw_p,
		(ParamFloat<px4::params::AQUA_YAWRATE_P>) _param_aqua_yawrate_p,
		(ParamFloat<px4::params::AQUA_YAWRATE_I>) _param_aqua_yawrate_i,
		(ParamFloat<px4::params::AQUA_YR_INT_LIM>) _param_aqua_yr_int_lim,
		(ParamFloat<px4::params::AQUA_YAWRATE_D>) _param_aqua_yawrate_d,
		(ParamFloat<px4::params::AQUA_YAWRATE_FF>) _param_aqua_yawrate_ff,
		(ParamFloat<px4::params::AQUA_YAWRATE_K>) _param_aqua_yawrate_k,
		(ParamFloat<px4::params::AQUA_DTERM_CUT>) _param_aqua_dterm_cutoff,	/**< Cutoff frequency for the D-term filter */

		(ParamFloat<px4::params::AQUA_YAWR_MAX>) _param_aqua_yawrate_max,
		(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max,		/**< scaling factor from stick to yaw rate */

		(ParamBool<px4::params::MC_BAT_SCALE_EN>) _param_mc_bat_scale_en,

		/* Throttle max */
		(ParamFloat<px4::params::AQUA_THR_MAX_MAN>) _param_aqua_thr_max_man,

		/* Circuit breaker for actuator[1] output */
		(ParamInt<px4::params::CBRK_RATE_CTRL>) _param_cbrk_rate_ctrl
	)

};

