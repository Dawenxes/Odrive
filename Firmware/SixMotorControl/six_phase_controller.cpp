
#include "six_phase_controller.hpp"
#include "six_phase_axis.hpp"
#include "axis.hpp"
#include "motor.hpp"
#include "encoder.hpp"
#include <algorithm>
#include <cmath>

void SixPhaseController::reset() {
    vel_integrator_torque_ = 0.0f;
    prev_torque_cmd_ = 0.0f;
    pos_setpoint_ = 0.0f;
    vel_setpoint_ = 0.0f;
    torque_setpoint_ = 0.0f;
    error_ = ERROR_NONE;
}

float SixPhaseController::compute_torque_command(float pos_estimate, float vel_estimate) {
    float pos_err = pos_setpoint_ - pos_estimate;
    float vel_cmd = pos_err * config_.pos_gain;

    // Velocity limiting
    if (config_.enable_vel_limit) {
        float vel_limit = config_.vel_limit * config_.vel_limit_tolerance;
        vel_cmd = std::clamp(vel_cmd, -vel_limit, vel_limit);
    }

    float vel_err = vel_cmd - vel_estimate;

    // Integrator with anti-windup
    float new_integrator = vel_integrator_torque_ + vel_err * config_.vel_integrator_gain * (1.0f / current_meas_hz);
    if (config_.vel_integrator_limit != INFINITY) {
        new_integrator = std::clamp(new_integrator, -config_.vel_integrator_limit, config_.vel_integrator_limit);
    }
    vel_integrator_torque_ = new_integrator;

    float torque_cmd = vel_err * config_.vel_gain + vel_integrator_torque_;

    // Apply torque ramp rate limit
    float max_torque_delta = config_.torque_ramp_rate / current_meas_hz;
    torque_cmd = std::clamp(torque_cmd, prev_torque_cmd_ - max_torque_delta, prev_torque_cmd_ + max_torque_delta);
    prev_torque_cmd_ = torque_cmd;

    // Apply torque limit
    if (config_.torque_limit != INFINITY) {
        torque_cmd = std::clamp(torque_cmd, -config_.torque_limit, config_.torque_limit);
    }

    return torque_cmd;
}

void SixPhaseController::distribute_current_commands(float Id, float Iq) {
    float share = std::clamp(config_.six_phase.current_share, 0.0f, 1.0f);

    // Simple current sharing: split the q-axis current proportionally.
    // d-axis current is typically the same (flux-producing).
    float Iq1 = Iq * share / (share + (1.0f - share));
    float Iq2 = Iq * (1.0f - share) / (share + (1.0f - share));

    // TODO: add harmonic injection here if enabled
    // For now, fundamental only.

    Idq_setpoint_1_ = {Id, Iq1};
    Idq_setpoint_2_ = {Id, Iq2};
}

void SixPhaseController::update(uint32_t timestamp) {
    (void)timestamp;

    if (error_ != ERROR_NONE) {
        torque_output_ = 0.0f;
        Idq_setpoint_1_ = {{0.0f, 0.0f}};
        Idq_setpoint_2_ = {{0.0f, 0.0f}};
        return;
    }

    auto pos = pos_estimate_src_.present();
    auto vel = vel_estimate_src_.present();

    if (!pos.has_value() || !vel.has_value()) {
        // Missing feedback
        torque_output_ = 0.0f;
        Idq_setpoint_1_ = {{0.0f, 0.0f}};
        Idq_setpoint_2_ = {{0.0f, 0.0f}};
        return;
    }

    float torque_cmd = compute_torque_command(*pos, *vel);
    torque_output_ = torque_cmd;

    // Convert torque to current setpoint using motor torque constant.
    // We assume both winding sets use the same motor parameters.
    // In 6-phase mode the torque constant applies to the combined motor.
    // TODO: get torque constant from SixPhaseMotorConfig or average both motors
    float torque_constant = 0.04f;  // placeholder [Nm/A]
    float Iq = torque_cmd / torque_constant;

    distribute_current_commands(0.0f, Iq);  // Id=0 for SPM, TODO: add flux weakening
}
