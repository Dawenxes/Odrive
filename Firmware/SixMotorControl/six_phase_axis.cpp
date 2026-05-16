
#include "six_phase_axis.hpp"
#include "motor.hpp"
#include "encoder.hpp"
#include "controller.hpp"
#include "open_loop_controller.hpp"
#include "sensorless_estimator.hpp"
#include "trapTraj.hpp"
#include "endstop.hpp"
#include "mechanical_brake.hpp"
#include "foc.hpp"
#include "odrive_main.h"
#include "utils.hpp"
#include "low_level.h"

#include <algorithm>

// ------------------------------------------------------------------------
// Construction / Config
// ------------------------------------------------------------------------

SixPhaseAxis::SixPhaseAxis(int axis_num,
                           Motor& motor0,
                           Motor& motor1,
                           Encoder& encoder,
                           SensorlessEstimator& sensorless_estimator,
                           TrapezoidalTrajectory& trap,
                           Endstop& min_endstop,
                           Endstop& max_endstop,
                           MechanicalBrake& mechanical_brake)
    : axis_num_(axis_num),
      motor0_(motor0),
      motor1_(motor1),
      encoder_(encoder),
      sensorless_estimator_(sensorless_estimator),
      trap_traj_(trap),
      min_endstop_(min_endstop),
      max_endstop_(max_endstop),
      mechanical_brake_(mechanical_brake)
{
    // Link sub-components back to us (where needed for telemetry/debug)
    controller_.six_phase_axis_ = this;
}

bool SixPhaseAxis::apply_config() {
    config_.parent = this;
    phase_offset_provider_.set_phase_offset(config_.phase_offset);
    return true;
}

void SixPhaseAxis::clear_config() {
    config_ = {};
}

// ------------------------------------------------------------------------
// Threading
// ------------------------------------------------------------------------

static void run_state_machine_loop_wrapper(void* ctx) {
    reinterpret_cast<SixPhaseAxis*>(ctx)->run_state_machine_loop();
    reinterpret_cast<SixPhaseAxis*>(ctx)->thread_id_valid_ = false;
}

void SixPhaseAxis::start_thread() {
    osThreadDef(thread_def, run_state_machine_loop_wrapper, thread_priority_, 0, stack_size_ / sizeof(StackType_t));
    thread_id_ = osThreadCreate(osThread(thread_def), this);
    thread_id_valid_ = true;
}

bool SixPhaseAxis::wait_for_control_iteration() {
    osSignalWait(0x0001, osWaitForever);
    osSignalWait(0x0001, osWaitForever);
    osSignalWait(0x0001, osWaitForever);
    return true;
}

// ------------------------------------------------------------------------
// Watchdog
// ------------------------------------------------------------------------

void SixPhaseAxis::watchdog_feed() {
    watchdog_current_value_ = get_watchdog_reset();
}

bool SixPhaseAxis::watchdog_check() {
    if (!config_.enable_watchdog) return true;
    if (watchdog_current_value_ > 0) {
        watchdog_current_value_--;
        return true;
    } else {
        error_ |= Error::ERROR_WATCHDOG_TIMER_EXPIRED;
        return false;
    }
}

// ------------------------------------------------------------------------
// Signal Routing
// ------------------------------------------------------------------------

void SixPhaseAxis::connect_closed_loop_routing() {
    if (is_routing_active_) return;

    CRITICAL_SECTION() {
        bool sensorless = config_.enable_sensorless_mode;

        // ---- Unified controller inputs ----
        controller_.pos_estimate_src_.connect_to(&encoder_.pos_estimate_);
        controller_.vel_estimate_src_.connect_to(&encoder_.vel_estimate_);

        // ---- Motor 0 (winding set 1) ----
        motor0_.torque_setpoint_src_.connect_to(&controller_.torque_output_);
        motor0_.direction_ = sensorless ? 1.0f : encoder_.config_.direction;

        motor0_.current_control_.enable_current_control_src_ =
            (motor0_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL);
        motor0_.current_control_.Idq_setpoint_src_.connect_to(&controller_.Idq_setpoint_1_);
        motor0_.current_control_.Vdq_setpoint_src_.connect_to(&controller_.Vdq_setpoint_1_);

        OutputPort<float>* phase_src = sensorless ? &sensorless_estimator_.phase_ : &encoder_.phase_;
        motor0_.current_control_.phase_src_.connect_to(phase_src);
        motor0_.phase_vel_src_.connect_to(sensorless ? &sensorless_estimator_.phase_vel_ : &encoder_.phase_vel_);
        motor0_.current_control_.phase_vel_src_.connect_to(sensorless ? &sensorless_estimator_.phase_vel_ : &encoder_.phase_vel_);

        // ---- Motor 1 (winding set 2) ----
        motor1_.torque_setpoint_src_.connect_to(&controller_.torque_output_);
        motor1_.direction_ = sensorless ? 1.0f : encoder_.config_.direction;

        motor1_.current_control_.enable_current_control_src_ =
            (motor1_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL);
        motor1_.current_control_.Idq_setpoint_src_.connect_to(&controller_.Idq_setpoint_2_);
        motor1_.current_control_.Vdq_setpoint_src_.connect_to(&controller_.Vdq_setpoint_2_);

        // Phase offset for 2nd winding
        phase_offset_provider_.phase_in_.connect_to(phase_src);
        phase_offset_provider_.phase_vel_in_.connect_to(sensorless ? &sensorless_estimator_.phase_vel_ : &encoder_.phase_vel_);
        motor1_.current_control_.phase_src_.connect_to(&phase_offset_provider_.phase_out_);
        motor1_.current_control_.phase_vel_src_.connect_to(&phase_offset_provider_.phase_vel_out_);
    }

    is_routing_active_ = true;
}

void SixPhaseAxis::disconnect_closed_loop_routing() {
    if (!is_routing_active_) return;

    CRITICAL_SECTION() {
        motor0_.torque_setpoint_src_.disconnect();
        motor1_.torque_setpoint_src_.disconnect();

        motor0_.current_control_.Idq_setpoint_src_.disconnect();
        motor0_.current_control_.Vdq_setpoint_src_.disconnect();
        motor1_.current_control_.Idq_setpoint_src_.disconnect();
        motor1_.current_control_.Vdq_setpoint_src_.disconnect();

        motor0_.current_control_.phase_src_.disconnect();
        motor0_.current_control_.phase_vel_src_.disconnect();
        motor1_.current_control_.phase_src_.disconnect();
        motor1_.current_control_.phase_vel_src_.disconnect();
    }

    is_routing_active_ = false;
}

void SixPhaseAxis::connect_open_loop_routing() {
    // Connect both motors to the shared open_loop_controller_ for calibration
    CRITICAL_SECTION() {
        motor0_.current_control_.enable_current_control_src_ =
            (motor0_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL);
        motor0_.current_control_.Idq_setpoint_src_.connect_to(&open_loop_controller_.Idq_setpoint_);
        motor0_.current_control_.Vdq_setpoint_src_.connect_to(&open_loop_controller_.Vdq_setpoint_);
        motor0_.current_control_.phase_src_.connect_to(&open_loop_controller_.phase_);
        motor0_.current_control_.phase_vel_src_.connect_to(&open_loop_controller_.phase_vel_);

        motor1_.current_control_.enable_current_control_src_ =
            (motor1_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL);
        motor1_.current_control_.Idq_setpoint_src_.connect_to(&open_loop_controller_.Idq_setpoint_);
        motor1_.current_control_.Vdq_setpoint_src_.connect_to(&open_loop_controller_.Vdq_setpoint_);

        // Motor1 phase with offset even during open loop
        phase_offset_provider_.phase_in_.connect_to(&open_loop_controller_.phase_);
        phase_offset_provider_.phase_vel_in_.connect_to(&open_loop_controller_.phase_vel_);
        motor1_.current_control_.phase_src_.connect_to(&phase_offset_provider_.phase_out_);
        motor1_.current_control_.phase_vel_src_.connect_to(&phase_offset_provider_.phase_vel_out_);
    }
}

void SixPhaseAxis::disconnect_open_loop_routing() {
    CRITICAL_SECTION() {
        motor0_.current_control_.Idq_setpoint_src_.disconnect();
        motor0_.current_control_.Vdq_setpoint_src_.disconnect();
        motor0_.current_control_.phase_src_.disconnect();
        motor0_.current_control_.phase_vel_src_.disconnect();

        motor1_.current_control_.Idq_setpoint_src_.disconnect();
        motor1_.current_control_.Vdq_setpoint_src_.disconnect();
        motor1_.current_control_.phase_src_.disconnect();
        motor1_.current_control_.phase_vel_src_.disconnect();
    }
}

// ------------------------------------------------------------------------
// Closed-loop helpers
// ------------------------------------------------------------------------

bool SixPhaseAxis::start_closed_loop_control() {
    connect_closed_loop_routing();
    controller_.reset();

    wait_for_control_iteration();

    bool armed0 = motor0_.arm(&motor0_.current_control_);
    bool armed1 = motor1_.arm(&motor1_.current_control_);

    return armed0 && armed1;
}

bool SixPhaseAxis::stop_closed_loop_control() {
    disconnect_closed_loop_routing();
    motor0_.disarm();
    motor1_.disarm();
    return check_for_errors();
}

bool SixPhaseAxis::run_closed_loop_control_loop() {
    start_closed_loop_control();

    while ((requested_state_ == State::AXIS_STATE_UNDEFINED) && motor0_.is_armed_ && motor1_.is_armed_) {
        osDelay(1);
    }

    stop_closed_loop_control();
    return check_for_errors();
}

// ------------------------------------------------------------------------
// Lock-in spin (for sensorless ramp or general lock-in)
// ------------------------------------------------------------------------

bool SixPhaseAxis::run_lockin_spin(const LockinConfig_t& lockin_config, bool remain_armed,
                                   std::function<bool(bool)> loop_cb) {
    CRITICAL_SECTION() {
        open_loop_controller_.Idq_setpoint_ = {0.0f, 0.0f};
        open_loop_controller_.Vdq_setpoint_ = {0.0f, 0.0f};
        open_loop_controller_.phase_ = 0.0f;
        open_loop_controller_.phase_vel_ = 0.0f;

        open_loop_controller_.max_current_ramp_ = lockin_config.current / lockin_config.ramp_time;
        open_loop_controller_.max_voltage_ramp_ = lockin_config.current / lockin_config.ramp_time;
        open_loop_controller_.max_phase_vel_ramp_ = lockin_config.accel;
        open_loop_controller_.target_current_ = (motor0_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL)
                                                    ? lockin_config.current : 0.0f;
        open_loop_controller_.target_voltage_ = (motor0_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL)
                                                    ? 0.0f : lockin_config.current;
        open_loop_controller_.target_vel_ = lockin_config.vel;
        open_loop_controller_.total_distance_ = 0.0f;

        connect_open_loop_routing();
    }

    wait_for_control_iteration();

    motor0_.arm(&motor0_.current_control_);
    motor1_.arm(&motor1_.current_control_);

    bool success = false;
    float dir = lockin_config.vel >= 0.0f ? 1.0f : -1.0f;

    while ((requested_state_ == State::AXIS_STATE_UNDEFINED)
           && motor0_.is_armed_ && motor1_.is_armed_) {
        bool reached_target_vel = std::abs(open_loop_controller_.phase_vel_.any().value_or(0.0f)
                                           - lockin_config.vel) <= std::numeric_limits<float>::epsilon();
        bool reached_target_dist = open_loop_controller_.total_distance_.any().value_or(0.0f) * dir
                                   >= lockin_config.finish_distance * dir;

        bool terminal = (reached_target_vel && lockin_config.finish_on_vel)
                     || (reached_target_dist && lockin_config.finish_on_distance);
        if (terminal) {
            success = true;
            break;
        }

        if (loop_cb && !loop_cb(reached_target_vel))
            break;

        asm volatile("" ::: "memory");
        osDelay(1);
    }

    if (!success || !remain_armed) {
        motor0_.disarm();
        motor1_.disarm();
        disconnect_open_loop_routing();
    }

    return success;
}

// ------------------------------------------------------------------------
// Calibration
// ------------------------------------------------------------------------

bool SixPhaseAxis::run_motor_calibration() {
    // Run resistance / inductance measurement on both motors sequentially
    bool ok0 = motor0_.run_calibration();
    bool ok1 = motor1_.run_calibration();
    return ok0 && ok1;
}

bool SixPhaseAxis::run_encoder_offset_calibration() {
    const float start_lock_duration = 1.0f;

    if (encoder_.config_.use_index && !encoder_.index_found_) {
        encoder_.set_error(Encoder::ERROR_INDEX_NOT_FOUND_YET);
        return false;
    }

    encoder_.shadow_count_ = encoder_.count_in_cpr_;

    CRITICAL_SECTION() {
        open_loop_controller_.Idq_setpoint_ = {0.0f, 0.0f};
        open_loop_controller_.Vdq_setpoint_ = {0.0f, 0.0f};
        open_loop_controller_.phase_ = 0.0f;
        open_loop_controller_.phase_vel_ = 0.0f;

        float max_current_ramp = motor0_.config_.calibration_current / start_lock_duration * 2.0f;
        open_loop_controller_.max_current_ramp_ = max_current_ramp;
        open_loop_controller_.max_voltage_ramp_ = max_current_ramp;
        open_loop_controller_.max_phase_vel_ramp_ = INFINITY;
        open_loop_controller_.target_current_ = (motor0_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL)
                                                    ? motor0_.config_.calibration_current : 0.0f;
        open_loop_controller_.target_voltage_ = (motor0_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL)
                                                    ? 0.0f : motor0_.config_.calibration_current;
        open_loop_controller_.target_vel_ = 0.0f;
        open_loop_controller_.total_distance_ = 0.0f;
        open_loop_controller_.phase_ = open_loop_controller_.initial_phase_ =
            wrap_pm_pi(0.0f - encoder_.config_.calib_scan_distance / 2.0f);

        connect_open_loop_routing();
    }

    wait_for_control_iteration();
    motor0_.arm(&motor0_.current_control_);
    motor1_.arm(&motor1_.current_control_);

    // Wait for motor current to settle
    osDelay(static_cast<int>(start_lock_duration * 1000.0f));

    int32_t init_enc_val = encoder_.count_in_cpr_;
    int32_t enc_val_accum = 0;
    int32_t enc_val_min = init_enc_val;
    int32_t enc_val_max = init_enc_val;

    float phase = open_loop_controller_.phase_.any().value_or(0.0f);
    float vel = encoder_.config_.calib_scan_omega;
    open_loop_controller_.target_vel_ = vel;

    // Scan forward
    while ((requested_state_ == State::AXIS_STATE_UNDEFINED)
           && motor0_.is_armed_ && motor1_.is_armed_) {
        phase = open_loop_controller_.phase_.any().value_or(0.0f);
        if (phase >= wrap_pm_pi(encoder_.config_.calib_scan_distance / 2.0f))
            break;
        osDelay(1);
    }

    // Scan backward
    open_loop_controller_.target_vel_ = -vel;
    while ((requested_state_ == State::AXIS_STATE_UNDEFINED)
           && motor0_.is_armed_ && motor1_.is_armed_) {
        phase = open_loop_controller_.phase_.any().value_or(0.0f);
        if (phase <= wrap_pm_pi(-encoder_.config_.calib_scan_distance / 2.0f))
            break;

        enc_val_accum += encoder_.count_in_cpr_ - init_enc_val;
        if (encoder_.count_in_cpr_ > enc_val_max) enc_val_max = encoder_.count_in_cpr_;
        if (encoder_.count_in_cpr_ < enc_val_min) enc_val_min = encoder_.count_in_cpr_;
        osDelay(1);
    }

    motor0_.disarm();
    motor1_.disarm();
    disconnect_open_loop_routing();

    // Validate scan range
    if ((enc_val_max - enc_val_min) < (encoder_.config_.cpr * encoder_.config_.calib_range)) {
        encoder_.set_error(Encoder::ERROR_CPR_POLEPAIRS_MISMATCH);
        return false;
    }

    // Store offset (same logic as Encoder::run_offset_calibration)
    float offset = encoder_.config_.direction * (float)enc_val_accum / (float)(enc_val_max - enc_val_min);
    encoder_.config_.phase_offset = static_cast<int32_t>(offset);
    encoder_.config_.phase_offset_float = offset - (float)encoder_.config_.phase_offset;
    return true;
}

// ------------------------------------------------------------------------
// Homing
// ------------------------------------------------------------------------

bool SixPhaseAxis::run_homing() {
    if (!min_endstop_.config_.enabled) {
        error_ |= Error::ERROR_HOMING_WITHOUT_ENDSTOP;
        return false;
    }

    // Phase 1: drive toward endstop at homing speed
    controller_.config_.pos_gain = 0.0f;  // pure velocity mode
    controller_.vel_setpoint_ = -controller_.config_.homing_speed;
    controller_.pos_setpoint_ = 0.0f;

    bool done = false;
    start_closed_loop_control();

    while ((requested_state_ == State::AXIS_STATE_UNDEFINED)
           && motor0_.is_armed_ && motor1_.is_armed_
           && !(done = min_endstop_.get_state())) {
        osDelay(1);
    }

    stop_closed_loop_control();
    controller_.vel_setpoint_ = 0.0f;

    if (!done) return false;

    error_ &= ~Error::ERROR_MIN_ENDSTOP_PRESSED;

    auto pos_estimate = encoder_.pos_estimate_.any();
    if (!pos_estimate.has_value()) {
        error_ |= Error::ERROR_UNKNOWN_POSITION;
        return false;
    }

    // Phase 2: move to offset position
    controller_.pos_setpoint_ = *pos_estimate + min_endstop_.config_.offset;
    start_closed_loop_control();

    // Wait until settled (simplified)
    osDelay(100);

    stop_closed_loop_control();
    encoder_.set_linear_count(0);
    controller_.pos_setpoint_ = 0.0f;
    controller_.vel_setpoint_ = 0.0f;
    return check_for_errors();
}

bool SixPhaseAxis::run_idle_loop() {
    mechanical_brake_.engage();
    while (requested_state_ == State::AXIS_STATE_UNDEFINED) {
        motor0_.setup();
        motor1_.setup();
        osDelay(1);
    }
    return check_for_errors();
}

// ------------------------------------------------------------------------
// Control loop callback
// ------------------------------------------------------------------------

void SixPhaseAxis::update(uint32_t timestamp) {
    if (current_state_ == State::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        if (!is_routing_active_) {
            connect_closed_loop_routing();
        }

        // Update unified controller
        controller_.update(timestamp);

        // Update phase offset for 2nd winding
        phase_offset_provider_.update(timestamp);

        // Update ACIM estimator if needed
        if (motor0_.config_.motor_type == Motor::MOTOR_TYPE_ACIM) {
            acim_estimator_.update(timestamp);
        }

        // Update both motors (load torque setpoint into FOC inputs)
        motor0_.update(timestamp);
        motor1_.update(timestamp);

        // Update both FOC controllers
        motor0_.current_control_.update(timestamp);
        motor1_.current_control_.update(timestamp);

        // Debug telemetry
        unified_pos_estimate_ = encoder_.pos_estimate_.any().value_or(0.0f);
        unified_vel_estimate_ = encoder_.vel_estimate_.any().value_or(0.0f);
        unified_torque_estimate_ = controller_.torque_output_.any().value_or(0.0f);
        auto idq1 = controller_.Idq_setpoint_1_.any();
        auto idq2 = controller_.Idq_setpoint_2_.any();
        Iq_winding1_ = idq1 ? idq1->second : 0.0f;
        Iq_winding2_ = idq2 ? idq2->second : 0.0f;
    } else {
        if (is_routing_active_) {
            disconnect_closed_loop_routing();
        }
    }
}

bool SixPhaseAxis::do_checks(uint32_t timestamp) {
    motor0_.effective_current_lim();
    motor1_.effective_current_lim();
    motor0_.do_checks(timestamp);
    motor1_.do_checks(timestamp);

    if (min_endstop_.config_.enabled && min_endstop_.rose()
        && (current_state_ != State::AXIS_STATE_HOMING)) {
        error_ |= Error::ERROR_MIN_ENDSTOP_PRESSED;
    } else if (max_endstop_.config_.enabled && max_endstop_.rose()
               && (current_state_ != State::AXIS_STATE_HOMING)) {
        error_ |= Error::ERROR_MAX_ENDSTOP_PRESSED;
    }

    return check_for_errors();
}

// ------------------------------------------------------------------------
// State Machine
// ------------------------------------------------------------------------

void SixPhaseAxis::run_state_machine_loop() {
    for (;;) {
        // Load task chain if a specific request is pending
        if (requested_state_ != State::AXIS_STATE_UNDEFINED) {
            size_t pos = 0;
            if (requested_state_ == State::AXIS_STATE_STARTUP_SEQUENCE) {
                if (config_.startup_motor_calibration)
                    task_chain_[pos++] = State::AXIS_STATE_MOTOR_CALIBRATION;
                if (config_.startup_encoder_offset_calibration)
                    task_chain_[pos++] = State::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
                if (config_.startup_homing)
                    task_chain_[pos++] = State::AXIS_STATE_HOMING;
                if (config_.startup_closed_loop_control)
                    task_chain_[pos++] = State::AXIS_STATE_CLOSED_LOOP_CONTROL;
                task_chain_[pos++] = State::AXIS_STATE_IDLE;
            } else if (requested_state_ == State::AXIS_STATE_FULL_CALIBRATION_SEQUENCE) {
                task_chain_[pos++] = State::AXIS_STATE_MOTOR_CALIBRATION;
                task_chain_[pos++] = State::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
                task_chain_[pos++] = State::AXIS_STATE_IDLE;
            } else if (requested_state_ != State::AXIS_STATE_UNDEFINED) {
                task_chain_[pos++] = requested_state_;
                task_chain_[pos++] = State::AXIS_STATE_IDLE;
            }
            task_chain_[pos++] = State::AXIS_STATE_UNDEFINED;
            requested_state_ = State::AXIS_STATE_UNDEFINED;
            error_ &= ~Error::ERROR_INVALID_STATE;
        }

        bool status;
        switch (current_state_) {
            case State::AXIS_STATE_MOTOR_CALIBRATION: {
                status = run_motor_calibration();
            } break;

            case State::AXIS_STATE_ENCODER_OFFSET_CALIBRATION: {
                if (!motor0_.is_calibrated_ || !motor1_.is_calibrated_)
                    goto invalid_state;
                status = run_encoder_offset_calibration();
            } break;

            case State::AXIS_STATE_HOMING: {
                if (!motor0_.is_calibrated_)
                    goto invalid_state;
                status = run_homing();
            } break;

            case State::AXIS_STATE_LOCKIN_SPIN: {
                if (!motor0_.is_calibrated_)
                    goto invalid_state;
                status = run_lockin_spin(config_.general_lockin, false);
            } break;

            case State::AXIS_STATE_CLOSED_LOOP_CONTROL: {
                if (!motor0_.is_calibrated_ || (encoder_.config_.direction == 0 && !config_.enable_sensorless_mode))
                    goto invalid_state;
                watchdog_feed();
                status = run_closed_loop_control_loop();
            } break;

            case State::AXIS_STATE_IDLE: {
                run_idle_loop();
                status = true;
            } break;

            default:
            invalid_state:
                error_ |= Error::ERROR_INVALID_STATE;
                status = false;
                break;
        }

        if (!status) {
            std::fill(task_chain_.begin(), task_chain_.end(), State::AXIS_STATE_UNDEFINED);
            current_state_ = State::AXIS_STATE_IDLE;
        } else {
            std::rotate(task_chain_.begin(), task_chain_.begin() + 1, task_chain_.end());
            task_chain_.back() = State::AXIS_STATE_UNDEFINED;
        }
    }
}
