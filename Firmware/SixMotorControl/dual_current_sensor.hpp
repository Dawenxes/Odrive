#ifndef DUAL_CURRENT_SENSOR_HPP
#define DUAL_CURRENT_SENSOR_HPP

#include <optional>

#include "autogen/interfaces.hpp"  // for Iph_ABC_t, Iph_DualABC_t

class DualCurrentSensor {
public:
    // Aggregate two 3-phase current measurements into one 6-phase measurement.
    // This should be called from the control loop after both motors have run
    // their current_meas_cb().
    void update(const std::optional<Iph_ABC_t>& current0,
                const std::optional<Iph_ABC_t>& current1) {
        if (current0.has_value() && current1.has_value()) {
            current_meas_ = {
                current0->phA, current0->phB, current0->phC,
                current1->phA, current1->phB, current1->phC
            };
            valid_ = true;
        } else {
            current_meas_ = std::nullopt;
            valid_ = false;
        }
    }

    std::optional<Iph_DualABC_t> current_meas() const { return current_meas_; }
    bool valid() const { return valid_; }

private:
    std::optional<Iph_DualABC_t> current_meas_;
    bool valid_ = false;
};

#endif // DUAL_CURRENT_SENSOR_HPP
