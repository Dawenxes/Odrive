#ifndef DUAL_HALL_SENSOR_HPP
#define DUAL_HALL_SENSOR_HPP

#include <array>
#include <cstdint>
#include <cmath>

#include <board.h>

class DualHallSensor {
public:
    DualHallSensor(Stm32Gpio hall1, Stm32Gpio hall2, Stm32Gpio hall3,
                   Stm32Gpio hall4, Stm32Gpio hall5, Stm32Gpio hall6)
        : hall_gpios_{hall1, hall2, hall3, hall4, hall5, hall6} {}

    // Capture all GPIO input registers in the high-priority sampling phase.
    void sample_now() {
        for (size_t i = 0; i < port_samples_.size(); ++i) {
            port_samples_[i] = ports_to_sample[i]->IDR;
        }
    }

    void update_pll_gains(float bandwidth) {
        pll_kp_ = 2.0f * bandwidth;
        pll_ki_ = 0.25f * pll_kp_ * pll_kp_;
    }

    void reset() {
        initialized_ = false;
        is_valid_ = false;
        raw_state_ = 0;
        count_in_cpr_ = 0;
        shadow_count_ = 0;
        pos_estimate_counts_ = 0.0f;
        pos_cpr_counts_ = 0.0f;
        vel_estimate_counts_ = 0.0f;
        angle_degrees_ = 0.0f;
        phase_rad_ = 0.0f;
        phase_vel_rad_s_ = 0.0f;
        pos_estimate_turns_ = 0.0f;
        vel_estimate_turns_s_ = 0.0f;
        consecutive_illegal_state_count_ = 0;
        debounced_state_ = 0;
        debounce_count_ = 0;
    }

    // Set debounce threshold (in number of consecutive valid samples)
    // Recommended: 2-3 for most applications (at 8kHz: 0.25-0.375ms debounce)
    void set_debounce_threshold(uint8_t threshold) {
        debounce_threshold_ = threshold;
    }

    // 解码采样的霍尔信号并运行 PLL 估算器以生成平滑的相位和速度。
    bool update(float dt) {
        raw_state_ = read_hall_state_sampled();
        
        // Debounce logic: only accept state change after consecutive valid samples
        if (raw_state_ != debounced_state_) {
            debounce_count_++;
            if (debounce_count_ < debounce_threshold_) {
                // Not yet confirmed; ignore and keep using debounced_state_
                raw_state_ = debounced_state_;
            } else {
                // State confirmed after threshold samples
                debounced_state_ = raw_state_;
                debounce_count_ = 0;
            }
        } else {
            // State is stable
            debounce_count_ = 0;
        }
        
        int32_t hall_cnt = 0;
        float discrete_angle = 0.0f;
        const bool ok = decode_state(raw_state_, &hall_cnt, &discrete_angle);
        is_valid_ = ok;

        // PLL预测步骤：在霍尔边缘之间保持状态连续.
        pos_estimate_counts_ += dt * vel_estimate_counts_;
        pos_cpr_counts_ += dt * vel_estimate_counts_;
        pos_cpr_counts_ = fmodf_pos(pos_cpr_counts_, (float)kStatesPerElecRev);

        if (!ok) {
            // illegal_state_count_: lifetime counter, useful for diagnostics.
            illegal_state_count_++;
            // consecutive_illegal_state_count_: protection trigger candidate.
            consecutive_illegal_state_count_++;
            update_outputs();
            return false;
        }

        consecutive_illegal_state_count_ = 0;

        if (!initialized_) {
            initialized_ = true;
            count_in_cpr_ = hall_cnt;
            shadow_count_ = hall_cnt;
            pos_estimate_counts_ = (float)hall_cnt;
            pos_cpr_counts_ = (float)hall_cnt;
            vel_estimate_counts_ = 0.0f;
            angle_degrees_ = discrete_angle;
            update_outputs();
            return true;
        }

        int32_t delta = mod(hall_cnt - count_in_cpr_, kStatesPerElecRev);
        if (delta > kStatesPerElecRev / 2) {
            delta -= kStatesPerElecRev;
        }
        shadow_count_ += delta;
        count_in_cpr_ = hall_cnt;

        float delta_pos = (float)(shadow_count_ - encoder_model(pos_estimate_counts_));
        float delta_pos_cpr = (float)(count_in_cpr_ - encoder_model(pos_cpr_counts_));
        // Keep circular error in [-states/2, states/2) for stable correction.
        delta_pos_cpr = wrap_pm(delta_pos_cpr, (float)kStatesPerElecRev);

        // PLL correction step:
        // - P term corrects position estimate
        // - I term updates speed estimate
        pos_estimate_counts_ += dt * pll_kp_ * delta_pos;
        pos_cpr_counts_ += dt * pll_kp_ * delta_pos_cpr;
        pos_cpr_counts_ = fmodf_pos(pos_cpr_counts_, (float)kStatesPerElecRev);

        vel_estimate_counts_ += dt * pll_ki_ * delta_pos_cpr;
        // Deadband to suppress low-speed jitter from hall quantization.
        if (std::abs(vel_estimate_counts_) < 0.5f * dt * pll_ki_) {
            vel_estimate_counts_ = 0.0f;
        }

        update_outputs();
        return true;
    }

    uint8_t raw_state() const { return raw_state_; }
    float angle_degrees() const { return angle_degrees_; }
    bool is_valid() const { return is_valid_; }
    float phase_rad() const { return phase_rad_; }
    float phase_vel_rad_s() const { return phase_vel_rad_s_; }
    float pos_estimate_turns() const { return pos_estimate_turns_; }
    float vel_estimate_turns_s() const { return vel_estimate_turns_s_; }
    uint32_t illegal_state_count() const { return illegal_state_count_; }
    uint32_t consecutive_illegal_state_count() const { return consecutive_illegal_state_count_; }

private:
    static constexpr std::array<GPIO_TypeDef*, 3> ports_to_sample = {GPIOA, GPIOB, GPIOC};
    static constexpr int32_t kStatesPerElecRev = 12;
    static constexpr float kTwoPi = 6.28318530717958647692f;

    static int32_t mod(int32_t x, int32_t n) {
        int32_t r = x % n;
        return r < 0 ? r + n : r;
    }

    static float fmodf_pos(float x, float n) {
        float r = std::fmod(x, n);
        return r < 0.0f ? r + n : r;
    }

    static float wrap_pm(float x, float period) {
        float y = std::fmod(x + 0.5f * period, period);
        if (y < 0.0f) {
            y += period;
        }
        return y - 0.5f * period;
    }

    static float wrap_pm_pi(float x) {
        return wrap_pm(x, kTwoPi);
    }

    bool read_sampled_gpio(Stm32Gpio gpio) const {
        for (size_t i = 0; i < ports_to_sample.size(); ++i) {
            if (ports_to_sample[i] == gpio.port_) {
                return (port_samples_[i] & gpio.pin_mask_) != 0;
            }
        }
        return false;
    }

    uint8_t read_hall_state_sampled() const {
        uint8_t state = 0;
        // Map hall1..hall6 to bit5..bit0.
        for (size_t i = 0; i < hall_gpios_.size(); ++i) {
            if (read_sampled_gpio(hall_gpios_[i])) {
                state |= (1u << (5u - i));
            }
        }
        return state;
    }

    static bool decode_state(uint8_t state, int32_t* hall_cnt, float* angle_degrees) {
        switch (state) {
            case 0b001001: *hall_cnt = 0;  *angle_degrees = 0.0f; return true;
            case 0b001101: *hall_cnt = 1;  *angle_degrees = 30.0f; return true;
            case 0b101101: *hall_cnt = 2;  *angle_degrees = 60.0f; return true;
            case 0b101100: *hall_cnt = 3;  *angle_degrees = 90.0f; return true;
            case 0b100100: *hall_cnt = 4;  *angle_degrees = 120.0f; return true;
            case 0b100110: *hall_cnt = 5;  *angle_degrees = 150.0f; return true;
            case 0b110110: *hall_cnt = 6;  *angle_degrees = 180.0f; return true;
            case 0b110010: *hall_cnt = 7;  *angle_degrees = 210.0f; return true;
            case 0b010010: *hall_cnt = 8;  *angle_degrees = 240.0f; return true;
            case 0b010011: *hall_cnt = 9;  *angle_degrees = 270.0f; return true;
            case 0b011011: *hall_cnt = 10; *angle_degrees = 300.0f; return true;
            case 0b011001: *hall_cnt = 11; *angle_degrees = 330.0f; return true;
            default: return false;
        }
    }

    static int32_t encoder_model(float internal_pos) {
        return (int32_t)std::floor(internal_pos);
    }

    void update_outputs() {
        // Linear multi-turn estimate (turn / turn/s).
        pos_estimate_turns_ = pos_estimate_counts_ / (float)kStatesPerElecRev;
        vel_estimate_turns_s_ = vel_estimate_counts_ / (float)kStatesPerElecRev;

        // Circular electrical phase for FOC usage.
        const float phase_turn = fmodf_pos(pos_cpr_counts_ / (float)kStatesPerElecRev, 1.0f);
        phase_rad_ = wrap_pm_pi(phase_turn * kTwoPi);
        phase_vel_rad_s_ = vel_estimate_turns_s_ * kTwoPi;
        // Human-readable degree output for telemetry/debug.
        angle_degrees_ = phase_turn * 360.0f;
    }

    std::array<Stm32Gpio, 6> hall_gpios_;
    std::array<uint16_t, ports_to_sample.size()> port_samples_{};
    uint8_t raw_state_ = 0;
    uint8_t debounced_state_ = 0;  // Debounced hall state
    uint8_t debounce_count_ = 0;   // Counter for confirming state transitions
    uint8_t debounce_threshold_ = 2; // Default: require 2 consecutive matches to confirm change (at 8kHz ≈ 0.25ms)

    bool initialized_ = false;
    int32_t count_in_cpr_ = 0;
    int32_t shadow_count_ = 0;
    float pos_estimate_counts_ = 0.0f;
    float pos_cpr_counts_ = 0.0f;
    float vel_estimate_counts_ = 0.0f;
    float pll_kp_ = 2000.0f;
    float pll_ki_ = 1000000.0f;

    float angle_degrees_ = 0.0f;
    float phase_rad_ = 0.0f;
    float phase_vel_rad_s_ = 0.0f;
    float pos_estimate_turns_ = 0.0f;
    float vel_estimate_turns_s_ = 0.0f;
    bool is_valid_ = false;
    uint32_t illegal_state_count_ = 0;
    uint32_t consecutive_illegal_state_count_ = 0;
};

#endif // DUAL_HALL_SENSOR_HPP
