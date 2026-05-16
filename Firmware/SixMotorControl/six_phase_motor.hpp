#ifndef SIX_PHASE_MOTOR_HPP
#define SIX_PHASE_MOTOR_HPP

/**
 * @brief Configuration and parameters specific to dual-three-phase (6-phase) motors.
 *
 * A 6-phase motor consists of two independent 3-phase windings spatially
 * displaced by a fixed electrical angle (typically 30°). This structure
 * holds the parameters that describe the coupling and geometry of those
 * windings.
 */
struct SixPhaseMotorConfig {
    /**
     * @brief Electrical angle offset between winding set 1 (ABC1) and
     *        winding set 2 (ABC2) in radians.
     *
     * Default is 30° = pi/6 rad. Positive means ABC2 leads ABC1.
     */
    float winding_offset = 0.5235987756f;  // 30° [rad]

    /**
     * @brief Current sharing ratio between the two winding sets.
     *
     * Value in [0, 1]. 0.5 means equal current in both windings.
     * Useful for thermal balancing or fault-tolerant operation.
     */
    float current_share = 0.5f;

    /**
     * @brief Enable 5th/7th harmonic current injection for torque ripple
     *        reduction (advanced feature, requires proper motor model).
     */
    bool enable_harmonic_injection = false;

    /**
     * @brief Amplitude of 5th harmonic injection current relative to
     *        fundamental (advanced, typically 0.0).
     */
    float harmonic_5th_gain = 0.0f;

    /**
     * @brief Amplitude of 7th harmonic injection current relative to
     *        fundamental (advanced, typically 0.0).
     */
    float harmonic_7th_gain = 0.0f;

    /**
     * @brief Mutual inductance between the two winding sets [H].
     *        Used for advanced decoupling control.
     */
    float mutual_inductance = 0.0f;

    /**
     * @brief Enable mutual inductance decoupling feedforward.
     */
    bool enable_decoupling_ff = false;
};

#endif // SIX_PHASE_MOTOR_HPP
