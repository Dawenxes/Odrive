#ifndef SIX_PHASE_CONTROLLER_HPP
#define SIX_PHASE_CONTROLLER_HPP

#include "component.hpp"
#include "six_phase_motor.hpp"
#include <autogen/interfaces.hpp>

// Forward declarations
class Axis;
class SixPhaseAxis;

/**
 * @brief Unified controller for a dual-three-phase (6-phase) motor.
 *
 * This controller replaces the per-axis Controller when running in 6-phase
 * mode. It implements a single mechanical position/velocity/torque control
 * loop and drives two 3-phase FOC controllers underneath.
 *
 * Architecture:
 *   SixPhaseController (mechanical loop)
 *        ├── Motor0::current_control_ (FOC for winding set 1, ABC1)
 *        └── Motor1::current_control_ (FOC for winding set 2, ABC2)
 *
 * The two FOC controllers share the same Idq/Vdq setpoints but receive
 * phase angles offset by `config_.winding_offset`.
 */
class SixPhaseController : public ComponentBase {
public:
    struct Config_t {
        // Mechanical control gains (same semantics as Controller)
        float pos_gain = 20.0f;                  // [(turn/s) / turn]
        float vel_gain = 1.0f / 6.0f;            // [Nm/(turn/s)]
        float vel_integrator_gain = 2.0f / 6.0f; // [Nm/(turn/s * s)]
        float vel_limit = 2.0f;                  // [turn/s]
        float vel_limit_tolerance = 1.2f;
        float vel_integrator_limit = INFINITY;
        float vel_ramp_rate = 1.0f;              // [(turn/s) / s]
        float torque_ramp_rate = 0.01f;          // [Nm / sec]
        float torque_limit = INFINITY;           // [Nm]

        bool enable_vel_limit = true;
        bool enable_gain_scheduling = false;
        float gain_scheduling_width = 10.0f;

        // 6-phase specific
        SixPhaseMotorConfig six_phase;

        // custom setters
        SixPhaseController* parent = nullptr;
        void set_pos_gain(float value) { pos_gain = value; }
        void set_vel_gain(float value) { vel_gain = value; }
        void set_vel_integrator_gain(float value) { vel_integrator_gain = value; }
    };

    SixPhaseController() = default;

    void reset();

    /**
     * @brief Run one iteration of the unified mechanical control loop.
     *
     * Computes a single torque setpoint and propagates it (along with the
     * current sharing ratio) to both underlying FOC controllers.
     */
    void update(uint32_t timestamp) final;

    Config_t config_;
    SixPhaseAxis* six_phase_axis_ = nullptr;

    // Error state
    enum Error {
        ERROR_NONE = 0,
        ERROR_ENCODER_FAILED = 0x00000001,
        ERROR_UNSTABLE_GAIN = 0x00000002,
    };
    Error error_ = ERROR_NONE;

    // ---- Inputs (from unified encoder/estimator) ----
    InputPort<float> pos_estimate_src_;
    InputPort<float> vel_estimate_src_;

    // ---- Setpoints (user/external input) ----
    float pos_setpoint_ = 0.0f;     // [turns]
    float vel_setpoint_ = 0.0f;     // [turn/s]
    float torque_setpoint_ = 0.0f;  // [Nm]

    // ---- Internal state ----
    float vel_integrator_torque_ = 0.0f;  // [Nm]
    float prev_torque_cmd_ = 0.0f;        // [Nm] for ramp rate limiting

    // ---- Outputs (to both Motor FOCs) ----
    OutputPort<float> torque_output_ = 0.0f;        // unified torque [Nm]
    OutputPort<float2D> Idq_setpoint_1_ = {{0.0f, 0.0f}};  // winding set 1 [A]
    OutputPort<float2D> Idq_setpoint_2_ = {{0.0f, 0.0f}};  // winding set 2 [A]
    OutputPort<float2D> Vdq_setpoint_1_ = {{0.0f, 0.0f}};  // winding set 1 [V]
    OutputPort<float2D> Vdq_setpoint_2_ = {{0.0f, 0.0f}};  // winding set 2 [V]

private:
    /**
     * @brief Compute the unified torque command from position/velocity loops.
     */
    float compute_torque_command(float pos_estimate, float vel_estimate);

    /**
     * @brief Distribute the unified torque/current command to both winding sets.
     */
    void distribute_current_commands(float Id, float Iq);

    uint32_t last_timestamp_ = 0;
};

#endif // SIX_PHASE_CONTROLLER_HPP
