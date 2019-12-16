#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/hydradrone_status.h>
#include <uORB/topics/actuator_controls.h>

class HydradroneStateMachine : px4::ScheduledWorkItem {
public:
	typedef void (*TransitionCallback)(uint8_t);

	enum class Transition {
		NONE, LOCK, UNLOCK, ROTATE_MC, ROTATE_AQUA
	};

	HydradroneStateMachine(TransitionCallback tcb);

	void set_loc_rot_time(float lock_time, float rot_time);

	void transition_to_state(uint8_t previous, uint8_t state);

private:

	void Run() override;

	void _unlock();
	void _lock();
	void _rotate_arm(bool aquatic);

	void _publish_actuators(bool lock, bool rotate = false, bool aquatic = false);

	Transition _get_rot_transition(uint8_t target_state);

	uint32_t _lock_time, _rot_time;

	uint8_t _target_state = hydradrone_status_s::HYDRADRONE_STATUS_MC;
	bool _ready = true;

	Transition _next_transition = Transition::NONE;

	TransitionCallback _tcb;

	static constexpr uint8_t LOCK_ID = 4;
	static constexpr float LOCK_ON = -1.0f;
	static constexpr float LOCK_OFF = 1.0f;

	static constexpr uint8_t ROTATION_ID = 5;
	static constexpr float ROTATION_FLOATING = -1.f;
	static constexpr float ROTATION_MULTICOPTER = 0.f;
	static constexpr float ROTATION_AQUATIC = 1.f;

	static constexpr uint8_t PROBE_ID = 6;
	static constexpr float PROBE_RETRACT = -1.0f;
	static constexpr float PROBE_DEPLOY = 1.0f;

	/**
	 * yaw_aqua + thrust_aqua actuator controls publication on 2, 3.
	 * lock servo + rotation servo + probe deploy servo on 4, 5, 6.
	 */
	uORB::PublicationData<actuator_controls_s> _actuators_1_pub{ORB_ID(actuator_controls_1)};
};
