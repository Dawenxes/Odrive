# SixMotorControl — 6-Phase (Dual-Three-Phase) Motor Control Extension

This module extends ODrive to support **asymmetrical six-phase motors** (also called dual-three-phase or split-phase motors) using the two existing 3-phase inverters on ODrive v3 hardware.

## Physical Model

A 6-phase motor has two independent 3-phase windings (ABC1 and ABC2) displaced by a fixed electrical angle, typically **30°** (π/6 rad). The two inverters drive the two winding sets with synchronized currents.

```
        A1 ──┐
        B1 ──┼── Rotor ── A2 ──┐
        C1 ──┘                 ├── Rotor (same shaft)
                               B2 ──┘
                               C2

Electrical offset: θ₂ = θ₁ + 30°
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     SixPhaseAxis                              │
│  ┌─────────────────┐        ┌─────────────────────────────┐ │
│  │ SixPhaseController│        │  PhaseOffsetProvider (30°)  │ │
│  │  (mechanical loop)│        │                             │ │
│  └────────┬────────┘        └──────────────┬──────────────┘ │
│           │ torque / Idq                      │ offset_phase │
│           ▼                                   ▼              │
│  ┌─────────────────┐              ┌─────────────────┐       │
│  │  Axis 0 / Motor 0 │              │  Axis 1 / Motor 1 │       │
│  │  FOC (winding 1)  │              │  FOC (winding 2)  │       │
│  │  phase = θ        │              │  phase = θ + 30°  │       │
│  └────────┬────────┘              └────────┬────────┘       │
│           │ PWM ABC1                         │ PWM ABC2       │
└───────────┼─────────────────────────────────┼────────────────┘
            ▼                                 ▼
        Inverter M0                       Inverter M1
```

### Key Design Decisions

1. **Unified Mechanical Loop**
   - One position/velocity/torque controller (`SixPhaseController`) serves both winding sets.
   - Both FOC controllers receive the same torque command, split by `current_share`.

2. **Phase Offset via Signal Routing**
   - Motor 0 uses the raw encoder phase.
   - Motor 1 uses the encoder phase shifted by `PhaseOffsetProvider`.
   - This reuses the existing FOC without modification.

3. **Minimal Intrusion into Existing Code**
   - No changes to `Motor`, `Axis`, `FOC`, or `Encoder` internals.
   - Signal routing is rewired at runtime when 6-phase mode is active.
   - The underlying `Axis` state machines still run independently in the background.

## Files

| File | Purpose |
|------|---------|
| `six_phase_motor.hpp` | 6-phase motor parameters (winding offset, current sharing, harmonics) |
| `phase_offset_provider.hpp` | Shifts electrical angle by configurable offset for 2nd winding |
| `six_phase_controller.hpp/cpp` | Unified P/PI controller; outputs two Idq/Vdq sets |
| `six_phase_axis.hpp/cpp` | Orchestrates two underlying axes; handles arming/sync/faults |
| `dual_current_sensor.hpp` *(existing)* | Aggregates two 3-phase current measurements into 6-phase |
| `dual_hall_sensor.hpp` *(existing)* | Decodes 6-channel hall state for 6-phase commutation |

## Usage Flow

1. **Configure** the motor parameters in `SixPhaseMotorConfig` (winding offset, current share).
2. **Instantiate** `SixPhaseAxis` with references to `axis0`, `axis1`, and a unified encoder.
3. **Call** `six_phase_axis.apply_config()` to initialize parameters.
4. **Arm** both inverters with `six_phase_axis.start_closed_loop_control()`.
5. **Set** position/velocity/torque setpoints on `six_phase_axis.controller_`.
6. **Call** `six_phase_axis.update(timestamp)` once per control loop iteration.

## Future Enhancements

- **Harmonic Injection**: Inject 5th/7th harmonics to reduce torque ripple.
- **Fault Tolerance**: Degrade to single 3-phase operation if one inverter fails.
- **Mutual Inductance Decoupling**: Add cross-coupling feedforward terms.
- **Unified Current Limiter**: Coordinate thermal limits across both inverters.
- **Six-Phase SVM**: Replace dual independent SVM with a unified 6-phase modulation scheme.
