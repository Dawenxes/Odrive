#ifndef DUAL_HALL_SENSOR_HPP
#define DUAL_HALL_SENSOR_HPP

#include <array>
#include <cstdint>
#include <cmath>

#include <board.h>

class HallSensor {
public:
    HallSensor(Stm32Gpio hallA, Stm32Gpio hallB, Stm32Gpio hallC)
        : hall_gpios_{hallA, hallB, hallC} {}

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
    }

    // 解码采样的霍尔信号并运行 PLL 估算器以生成平滑的相位和速度。
    bool update(float dt) {
        raw_state_ = read_hall_state_sampled();
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
        delta *= direction_;

        shadow_count_ += delta;
        count_in_cpr_ += delta;
        count_in_cpr_ = mod(count_in_cpr_, kStatesPerElecRev);

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

    void set_direction(int32_t dir) { direction_ = dir; }
    void set_phase_offset(float offset) { phase_offset_ = offset; }

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
    static constexpr int32_t kStatesPerElecRev = 6;
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
        // Map hallA, hallB, hallC to bit0, bit1, bit2.
        if (read_sampled_gpio(hall_gpios_[0])) state |= 0b001;
        if (read_sampled_gpio(hall_gpios_[1])) state |= 0b010;
        if (read_sampled_gpio(hall_gpios_[2])) state |= 0b100;
        return state;
    }

    static bool decode_state(uint8_t state, int32_t* hall_cnt, float* angle_degrees) {
        switch (state) {
            case 0b001: *hall_cnt = 0; *angle_degrees = 0.0f;   return true;
            case 0b011: *hall_cnt = 1; *angle_degrees = 60.0f;  return true;
            case 0b010: *hall_cnt = 2; *angle_degrees = 120.0f; return true;
            case 0b110: *hall_cnt = 3; *angle_degrees = 180.0f; return true;
            case 0b100: *hall_cnt = 4; *angle_degrees = 240.0f; return true;
            case 0b101: *hall_cnt = 5; *angle_degrees = 300.0f; return true;
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
        const float phase_turn = fmodf_pos(
            (pos_cpr_counts_ - phase_offset_) / (float)kStatesPerElecRev, 1.0f);
        phase_rad_ = wrap_pm_pi(phase_turn * kTwoPi) * (float)direction_;
        phase_vel_rad_s_ = vel_estimate_turns_s_ * kTwoPi * (float)direction_;
        // Human-readable degree output for telemetry/debug.
        angle_degrees_ = phase_turn * 360.0f;
    }

    std::array<Stm32Gpio, 3> hall_gpios_;
    std::array<uint16_t, ports_to_sample.size()> port_samples_{};
    uint8_t raw_state_ = 0;

    bool initialized_ = false;
    int32_t count_in_cpr_ = 0;
    int32_t shadow_count_ = 0;
    float pos_estimate_counts_ = 0.0f;
    float pos_cpr_counts_ = 0.0f;
    float vel_estimate_counts_ = 0.0f;
    float pll_kp_ = 2000.0f;
    float pll_ki_ = 1000000.0f;

    int32_t direction_ = 1;
    float phase_offset_ = 0.0f;

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
