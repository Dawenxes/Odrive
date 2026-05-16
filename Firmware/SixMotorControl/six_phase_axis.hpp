#ifndef SIX_PHASE_AXIS_HPP
#define SIX_PHASE_AXIS_HPP

#include <array>
#include <cstdint>
#include <functional>

#include "six_phase_controller.hpp"
#include "phase_offset_provider.hpp"
#include "six_phase_motor.hpp"
#include <autogen/interfaces.hpp>

// Forward declarations
class Motor;
class Encoder;
class SensorlessEstimator;
class TrapezoidalTrajectory;
class Endstop;
class MechanicalBrake;
class OpenLoopController;

/**
 * @brief Unified axis controller for a dual-three-phase (6-phase) motor.
 *
 * Unlike the original approach of composing two Axis objects, this class
 * directly owns the unified mechanical controller and operates on two
 * Motor objects (one per 3-phase inverter). This avoids redundant threads,
 * redundant state machines, and conflicting step/dir interfaces.
 *
 * Architecture:
 *   SixPhaseAxis (one thread, one state machine)
 *   ├── SixPhaseController     (unified P/PI mechanical loop)
 *   ├── OpenLoopController     (shared open-loop reference for calibration)
 *   ├── PhaseOffsetProvider    (30° electrical offset for 2nd winding)
 *   ├── AcimEstimator          (optional, for ACIM six-phase)
 *   ├── Encoder&               (single position feedback source)
 *   ├── SensorlessEstimator&   (optional fallback)
 *   ├── Motor& motor0_         (inverter 0 → winding set ABC1)
 *   └── Motor& motor1_         (inverter 1 → winding set ABC2)
 */
class SixPhaseAxis {
public:
    using State = ODriveIntf::AxisIntf::AxisState;
    using Error = ODriveIntf::AxisIntf::Error;
    using LockinConfig_t = Axis::LockinConfig_t;

    struct Config_t {
        bool startup_motor_calibration = false;
        bool startup_encoder_offset_calibration = false;
        bool startup_closed_loop_control = false;
        bool startup_homing = false;

        bool enable_sensorless_mode = false;
        bool enable_watchdog = false;
        float watchdog_timeout = 0.0f;

        // 6-phase specific
        float phase_offset = 0.5235987756f;  // 30° [rad]
        float current_share = 0.5f;
        bool require_dual_ready = true;

        LockinConfig_t calibration_lockin;
        LockinConfig_t sensorless_ramp;
        LockinConfig_t general_lockin;

        // custom setters
        SixPhaseAxis* parent = nullptr;
        void set_phase_offset(float value) {
            phase_offset = value;
            if (parent) parent->phase_offset_provider_.set_phase_offset(value);
        }
    };

    SixPhaseAxis(int axis_num,
                 Motor& motor0,
                 Motor& motor1,
                 Encoder& encoder,
                 SensorlessEstimator& sensorless_estimator,
                 TrapezoidalTrajectory& trap,
                 Endstop& min_endstop,
                 Endstop& max_endstop,
                 MechanicalBrake& mechanical_brake);

    bool apply_config();
    void clear_config();

    void start_thread();
    bool wait_for_control_iteration();

    bool do_checks(uint32_t timestamp);
    void watchdog_feed();
    bool watchdog_check();

    bool check_for_errors() const { return error_ == ERROR_NONE; }

    // State-machine helpers
    bool start_closed_loop_control();
    bool stop_closed_loop_control();
    bool run_closed_loop_control_loop();
    bool run_lockin_spin(const LockinConfig_t& lockin_config, bool remain_armed,
                         std::function<bool(bool)> loop_cb = {});
    bool run_homing();
    bool run_idle_loop();
    bool run_motor_calibration();
    bool run_encoder_offset_calibration();

    // Main state machine (runs in dedicated thread)
    void run_state_machine_loop();

    // Called from ODrive::control_loop_cb at PWM frequency
    void update(uint32_t timestamp);

    // Exposed components
    SixPhaseController controller_;
    OpenLoopController open_loop_controller_;
    PhaseOffsetProvider phase_offset_provider_;
    AcimEstimator acim_estimator_;

    // Hardware references
    Motor& motor0_;
    Motor& motor1_;
    Encoder& encoder_;
    SensorlessEstimator& sensorless_estimator_;
    TrapezoidalTrajectory& trap_traj_;
    Endstop& min_endstop_;
    Endstop& max_endstop_;
    MechanicalBrake& mechanical_brake_;

    int axis_num_ = 0;
    Config_t config_;

    // State / error
    Error error_ = ERROR_NONE;
    float last_error_time_ = 0.0f;
    State requested_state_ = State::AXIS_STATE_STARTUP_SEQUENCE;
    std::array<State, 10> task_chain_ = { State::AXIS_STATE_UNDEFINED };
    State& current_state_ = task_chain_.front();

    // Threading
    osPriority thread_priority_ = osPriorityHigh;
    osThreadId thread_id_ = 0;
    const uint32_t stack_size_ = 2048;
    volatile bool thread_id_valid_ = false;

    // Watchdog
    uint32_t watchdog_current_value_ = 0;
    constexpr uint32_t get_watchdog_reset() {
        return static_cast<uint32_t>(std::clamp<float>(config_.watchdog_timeout, 0, UINT32_MAX / (current_meas_hz + 1)) * current_meas_hz);
    }

    // Debug / telemetry
    float unified_pos_estimate_ = 0.0f;
    float unified_vel_estimate_ = 0.0f;
    float unified_torque_estimate_ = 0.0f;
    float Iq_winding1_ = 0.0f;
    float Iq_winding2_ = 0.0f;

private:
    void connect_closed_loop_routing();
    void disconnect_closed_loop_routing();
    void connect_open_loop_routing();
    void disconnect_open_loop_routing();

    bool is_routing_active_ = false;
};

#endif // SIX_PHASE_AXIS_HPP
