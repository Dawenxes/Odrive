#ifndef SIX_PHASE_FOC_HPP
#define SIX_PHASE_FOC_HPP

#include "phase_control_law.hpp"
#include "component.hpp"
#include "six_phase_motor.hpp"

/**
 * @brief Unified 6-phase FOC controller (future advanced implementation).
 *
 * This is an optional advanced controller that replaces running two
 * independent 3-phase FOCs. It treats the 6-phase machine as a single
 * entity in the stationary (α-β) frame with an extended Clarke transform
 * or in a combined (d-q) + (z1-z2) frame.
 *
 * For the initial framework the recommended approach is to run two
 * independent FieldOrientedControllers (one per inverter) and use
 * PhaseOffsetProvider to shift the angle of the second. This class
 * is provided as a placeholder for future optimization.
 *
 * References:
 *   - "Vector Control of Asymmetrical Six-Phase Induction Machines"
 *   - "Modeling and Control of Six-Phase PMSM"
 */
class SixPhaseFoc : public PhaseControlLaw<6>, public ComponentBase {
public:
    void update(uint32_t timestamp) final;
    void reset() final;

    ODriveIntf::MotorIntf::Error on_measurement(
            std::optional<float> vbus_voltage,
            std::optional<std::array<float, 6>> currents,
            uint32_t input_timestamp) final;

    ODriveIntf::MotorIntf::Error get_output(
            uint32_t output_timestamp,
            float (&pwm_timings)[6],
            std::optional<float>* ibus) final;

    // Config
    SixPhaseMotorConfig motor_config_;
    std::optional<std::array<float, 4>> pi_gains_;  // [d, q, z1, z2]

    // Inputs
    InputPort<float2D> Idq_setpoint_src_;    // fundamental d-q
    InputPort<float2D> Iz_setpoint_src_;     // harmonic z1-z2 (optional)
    InputPort<float> phase_src_;
    InputPort<float> phase_vel_src_;

    // Measurements / State (populated by on_measurement)
    uint32_t i_timestamp_ = 0;
    std::optional<float> vbus_voltage_measured_;
    std::optional<std::array<float, 6>> Iph_measured_;  // A1,B1,C1,A2,B2,C2
    float Id_ = 0.0f, Iq_ = 0.0f;
    float Iz1_ = 0.0f, Iz2_ = 0.0f;

    // Integrators
    float v_integral_d_ = 0.0f;
    float v_integral_q_ = 0.0f;
    float v_integral_z1_ = 0.0f;
    float v_integral_z2_ = 0.0f;

    // Outputs for debug
    float final_v_alpha_ = 0.0f;
    float final_v_beta_ = 0.0f;
    float power_ = 0.0f;
};

#endif // SIX_PHASE_FOC_HPP
