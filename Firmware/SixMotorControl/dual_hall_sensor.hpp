#ifndef DUAL_HALL_SENSOR_HPP
#define DUAL_HALL_SENSOR_HPP

#include <array>
#include <cstdint>

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

    // Decode the sampled 6-bit hall code into angle (0..330 deg in 30 deg steps).
    void update() {
        raw_state_ = read_hall_state_sampled();
        const bool ok = decode_state(raw_state_, &angle_degrees_);
        is_valid_ = ok;
    }

    uint8_t raw_state() const { return raw_state_; }
    float angle_degrees() const { return angle_degrees_; }
    bool is_valid() const { return is_valid_; }

private:
    static constexpr std::array<GPIO_TypeDef*, 3> ports_to_sample = {GPIOA, GPIOB, GPIOC};

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

    static bool decode_state(uint8_t state, float* angle_degrees) {
        switch (state) {
            case 0b001001: *angle_degrees = 0.0f; return true;
            case 0b001101: *angle_degrees = 30.0f; return true;
            case 0b101101: *angle_degrees = 60.0f; return true;
            case 0b101100: *angle_degrees = 90.0f; return true;
            case 0b100100: *angle_degrees = 120.0f; return true;
            case 0b100110: *angle_degrees = 150.0f; return true;
            case 0b110110: *angle_degrees = 180.0f; return true;
            case 0b110010: *angle_degrees = 210.0f; return true;
            case 0b010010: *angle_degrees = 240.0f; return true;
            case 0b010011: *angle_degrees = 270.0f; return true;
            case 0b011011: *angle_degrees = 300.0f; return true;
            case 0b011001: *angle_degrees = 330.0f; return true;
            default: return false;
        }
    }

    std::array<Stm32Gpio, 6> hall_gpios_;
    std::array<uint16_t, ports_to_sample.size()> port_samples_{};
    uint8_t raw_state_ = 0;
    float angle_degrees_ = 0.0f;
    bool is_valid_ = false;
};

#endif // DUAL_HALL_SENSOR_HPP