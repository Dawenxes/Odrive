#ifndef PHASE_OFFSET_PROVIDER_HPP
#define PHASE_OFFSET_PROVIDER_HPP

#include "component.hpp"
#include <cmath>

/**
 * @brief Provides a phase signal with a configurable electrical offset.
 *
 * In a dual-three-phase (6-phase) motor the second winding set is displaced
 * by a fixed electrical angle (typically 30°). This component takes the
 * raw phase estimate from an encoder/sensorless estimator and produces an
 * offset phase suitable for the FOC of the second inverter.
 *
 * Usage:
 *   Motor0 FOC phase_src connects to encoder.phase_ (no offset).
 *   Motor1 FOC phase_src connects to phase_offset_provider.phase_out_ (offset).
 */
class PhaseOffsetProvider : public ComponentBase {
public:
    /**
     * @brief Update the offset phase based on the connected input.
     *
     * Must be called once per control loop iteration, after the source
     * phase has been updated.
     */
    void update(uint32_t timestamp) final {
        (void)timestamp;

        std::optional<float> phase_in = phase_in_.present();
        std::optional<float> phase_vel_in = phase_vel_in_.present();

        if (phase_in.has_value()) {
            float offset_phase = *phase_in + phase_offset_;
            // Wrap to [-pi, pi)
            offset_phase = wrap_pm_pi(offset_phase);
            phase_out_ = offset_phase;
        }

        if (phase_vel_in.has_value()) {
            phase_vel_out_ = *phase_vel_in;
        }
    }

    /**
     * @brief Set the electrical offset in radians.
     */
    void set_phase_offset(float offset_rad) {
        phase_offset_ = offset_rad;
    }

    float phase_offset() const { return phase_offset_; }

    // Inputs
    InputPort<float> phase_in_;
    InputPort<float> phase_vel_in_;

    // Outputs
    OutputPort<float> phase_out_ = 0.0f;
    OutputPort<float> phase_vel_out_ = 0.0f;

private:
    static float wrap_pm_pi(float x) {
        constexpr float two_pi = 6.28318530717958647692f;
        float y = std::fmod(x + 0.5f * two_pi, two_pi);
        if (y < 0.0f) {
            y += two_pi;
        }
        return y - 0.5f * two_pi;
    }

    float phase_offset_ = 0.5235987756f;  // default 30°
};

#endif // PHASE_OFFSET_PROVIDER_HPP
