# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Firmware (Arduino/Teensyduino, C++) for a Furuta pendulum (rotary inverted pendulum) driven by a
Teensy 4.1. Two control modes: classic swing-up (energy-based) + linear state-feedback balance,
and an on-device tabular Q-learning agent. Not a git repo currently, no build system checked in —
this is a single-sketch Arduino project (`FurutaPendulum.ino` + sibling `.cpp`/`.h` modules, all
compiled together by the Arduino build process).

**Status: written blind against the hardware spec, never compiled or run.** Treat every gain,
sign constant, and pin assignment as unverified until confirmed against real hardware behavior
(see README.md's "Mise en route" bring-up sequence).

## Build / compile / upload

No `arduino-cli` config exists yet in this repo. To set one up:

```bash
# one-time environment setup
arduino-cli core update-index --additional-urls https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli core install teensy:avr --additional-urls https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli lib install "Adafruit GFX Library" "Adafruit GC9A01A" "Adafruit BusIO" "Simple FOC"
# Encoder and SD come bundled with Teensyduino, not from the library manager

# compile (board FQBN for Teensy 4.1)
arduino-cli compile --fqbn teensy:avr:teensy41 .

# upload (port varies; teensy_loader_cli / arduino-cli with the Teensy core handles the
# bootloader-press step)
arduino-cli upload --fqbn teensy:avr:teensy41 -p <PORT> .
```

There is no test suite, linter, or CI — this is control-loop firmware validated on hardware, not
via automated tests. "Testing" means the on-device Debug menu routines (Angles live, Test moteur
auto, Jog manuel) described in README.md's bring-up procedure.

`encoders.h` has a commented-out `#define USE_HW_QUADENCODER` that switches from the interrupt-driven
`Encoder` library to the Teensy 4.x hardware quadrature decoder (requires installing the
`Teensy-4.x-Quad-Encoder-Library` by mjs513, not in the library manager under that name).

## Architecture

**Control loop split**: a 1 kHz `IntervalTimer` ISR (`controlTick()` in `FurutaPendulum.ino`) owns
sensor reads, safety checks, and motor output. `loop()` only handles UI polling, screen redraw
(10 Hz), SD logging (50 Hz), and Q-table autosave — it must never touch hardware the ISR owns.
Q-learning runs inside the same 1 kHz ISR but is rate-divided to 50 Hz via `RL_DIVIDER`.

**Shared-state discipline**: `sysState` (the state machine enum) and `faultCode` are `volatile`,
written by the ISR, read by `loop()`. The `PendulumState` working copy (`isrState`) is
ISR-exclusive; `loop()` only ever sees `snapshot`, a copy published under `noInterrupts()`/
`interrupts()` guarded by the `snapReady` flag. Any new field added to `PendulumState` that
`loop()` needs must flow through this same snapshot mechanism, not be read directly from `isrState`.

State transitions (`enterState()`, in `FurutaPendulum.ino`) always run from `loop()`, never from
the ISR, and wrap their work in `noInterrupts()`/`interrupts()` to atomically swap controller mode,
reset the PI/Safety/Q-learning internals, and stop the motor.

**Second interrupt source**: the menu rotary encoder (KY-040) is decoded independently in
`ui.cpp` via `attachInterrupt` on CLK/DT (full quadrature state-table decode into `rotDelta`,
consumed one detent per `UI::poll()` call) — this runs concurrently with the 1kHz control ISR and
its own `noInterrupts()` critical section is only around reading/decrementing `rotDelta`, not
shared with the control loop's state. When touching either ISR, check for priority interaction
(`ctrlTimer.priority(64)` in `setup()`) and keep critical sections minimal.

**Module responsibilities** (each is a `namespace`, not a class — all state is file-local statics):
- `config.h` — single source of truth for pins, physical/gear ratios, the `SysState`/`FaultCode`
  enums, and the shared `PendulumState` struct. The tuning constants here (signs, gains, motor
  limits, physical model, safety limits) are now **compile-time defaults only** — the live value
  applied by the firmware is `Settings::cfg.<field>` (see `settings.*`). Editing a constant in
  `config.h` changes the default used at first boot or after "Defauts".
- `settings.*` — runtime-editable tuning parameters (`Settings::cfg`, a RAM struct) with an
  on-device editor (`ST_SETTINGS` menu) and EEPROM persistence (magic+version+FNV checksum;
  invalid/absent → defaults from `config.h`). A table-driven `Desc[]` (name/pointer-to-member/
  min/max/step/decimals) drives both the editor UI and clamping — add a param by adding a struct
  field + one `TABLE[]` row + one line in `loadDefaults()`. Each field is a lone aligned `float`,
  so the 1 kHz ISR reads it without a lock; `load()` (bulk struct overwrite) runs under
  `noInterrupts()`. Derived `eTop()`/`pendJ()` are computed from the editable mass/length.
  Q-learning table dimensions stay compile-time (they size a fixed DMAMEM array).
- `encoders.*` — quadrature counts → radians, applying `ARM_SIGN`/`PEND_SIGN` and the gear ratios
  (`ARM_ENC_RATIO`). All angles/rates in the rest of the codebase are already in vertical-axis
  units — never re-apply a gear ratio downstream. `alpha=0` is pendulum-up by convention;
  calibration (`calibrateBottom()`) assumes the pendulum is physically at rest, hanging down.
  **Calibration is a software offset — the hardware counters are never written.** This is
  load-bearing: FOC commutation reads the *raw* arm counter via `motorShaftAngle()`, so zeroing
  it would destroy the electrical zero set by `initFOC()`. `rawArm()`/`rawPend()` stay raw.
- `motor.*` — **BLDC driven by FOC** (SimpleFOC + SimpleFOCMini/DRV8313). The commutation sensor
  is **the existing arm encoder**, not a dedicated one: motor and arm encoder are both 2:1 off the
  vertical axis, so they turn 1:1 with each other (`MOTOR_ENC_RATIO`). A small `Sensor` subclass
  (`ArmShaftSensor`) adapts it, reading through `Encoders::motorShaftAngle()` rather than creating
  a second `Encoder` object on pins 0/1 (which would double-attach interrupts).
  `setDuty(u)` now means a **normalized torque** in [-1,1], scaled by `MOTOR_VOLT_LIMIT` into a
  q-axis voltage. **Critical split**: `setDuty`/`velocityStep`/`hardStop` run in the 1 kHz ISR and
  only update a `volatile float` — they touch no peripheral. `Motor::spin()` does the actual FOC
  commutation and runs in its **own `focTimer` IntervalTimer at `FOC_FREQ_HZ` (10 kHz), at higher
  priority (32) than the control loop (64)** — never from `loop()`, which stalls for tens of ms on
  SPI redraws/SD/Serial and would make the motor stutter. This is only possible because the
  commutation sensor is the arm encoder (a counter read, ISR-safe) rather than an I2C device.
  `Motor::openLoopEnable()` reconfigures SimpleFOC and so guards with `noInterrupts()`.
  `Motor::begin()` runs `initFOC()`, which **physically moves the
  motor** for alignment — it runs before `Encoders::begin()` so calibration happens after.
  `MOTOR_POLE_PAIRS` must match the motor (7 for a 12N14P GBM2804) or alignment fails.
  Deadband compensation is gone (FOC is smooth from 0 rpm) — but note that only removes *motor*
  deadband; the gear train's static friction is real and is handled by the `kThi` integral term
  in `control_classic`, not here.
- `safety.*` — pure function `check()` called only while a motor-active state is running; returns
  a `FaultCode`, does not itself stop the motor (caller does).
- `control_classic.*` — internal `Phase` (SWINGUP/BALANCE) state machine, switches on `s.alpha`
  thresholds (`BAL_ENTER_RAD`/`BAL_EXIT_RAD` hysteresis band). `balanceOnlyMode` disables the
  swing-up half entirely for the "Balance seul" menu mode. The balance law carries an optional
  integral on theta (`cfg.kThi`, default 0) to break gear-train stiction; it only accumulates
  near vertical and is clamped by back-calculation to `TH_I_MAX`, and `thetaInt` is zeroed on
  every path that leaves balance so it never carries stale charge into a new attempt. Once the
  arm is back home *and* stopped (`TH_I_DEAD_RAD`/`TH_I_DEAD_DOT`) the integral **fades out** over
  `TH_I_FADE_S` rather than being zeroed — a hard cut would step the command by up to `TH_I_MAX`
  and kick the pendulum.
- `qlearning.*` — Q-table is `DMAMEM` (Teensy's second RAM bank) sized `QL_N_ALPHA * QL_N_ADOT *
  QL_N_ACT` floats (~42 kB) to keep it off the primary RAM used by the rest of the firmware.
  State discretization (`binAlpha`/`binAdot`) and the 7 discrete arm-velocity actions
  (`ACTION_W`) are config-driven; reward shaping lives in `reward()`. **`bestAction()` must break
  ties toward `ACT_NEUTRAL`** (the 0 rad/s action), not index 0 — index 0 is `-QL_W_MAX`, so a
  naive `best=0` start makes an untrained (all-zero) table command full reverse speed forever.
  **The state is `[alpha, alphaDot]` only — `theta` is not observed**, so the policy structurally
  cannot learn to keep the arm centered. Fixing that means adding a theta dimension to the table
  (and multiplying its size). Instead, arm drift is handled *episodically*: exceeding
  `QL_THETA_TURNS` is a **terminal state** (reward `QL_R_OUT_RANGE`, no bootstrap on the successor)
  that ends the episode and enters a `resetting` phase which drives the arm back to theta≈0 before
  the next episode starts — training continues rather than halting. **The reset drives the motor
  with a direct torque PD on theta (`QL_RESET_*`, applied in `controlTick`), deliberately bypassing
  `Motor::velocityStep`** — `KP_VEL`/`KI_VEL` are untuned, and a reset that fails to bring the arm
  home lets theta ratchet up episode after episode until `FAULT_THETA_RANGE`. Keep the reset
  independent of that inner loop. This is separate from
  `TurnsMax`, the system-wide runaway guard that faults out of the mode entirely.
  **Reward-shaping invariant**: the "pendulum up" bonuses are gated on low `|thetaDot|`. Without
  that gate a fast-spinning arm holds the pendulum up centrifugally and collects the bonus forever
  — a local optimum the agent never leaves. Any change to `reward()` must preserve that gate. `step()` does the Q-update
  for the *previous* transition before selecting the next action (standard online tabular
  Q-learning), and is a no-op update in greedy mode.
- `storage.*` — microSD (`BUILTIN_SDCARD`) for binary Q-table snapshots (`/q_current.bin`,
  `/q_best.bin`, versioned header with table dimensions to reject mismatched loads) and CSV
  training logs (`/logs/log_NNNN.csv`). Logging functions are documented as loop()-only, never
  call from the 1 kHz ISR.
- `ui.*` — GC9A01 SPI display + KY-040 menu encoder. Rendering functions do partial/differential
  redraws keyed off a `fullRedraw` flag to avoid flicker (fixed-width overwrites rather than
  clear+redraw); only call the `draw*` functions from `loop()` at the throttled 10 Hz rate.

**State machine** (`FurutaPendulum.ino`): `handleMenuSelect()` maps menu navigation to
`enterState()` calls; the `motorActive` set of states in `controlTick()` must stay in sync with
which states actually drive the motor — if you add a new active state, add it to that boolean AND
to the `switch` below it, otherwise Safety checks won't run for it.

## Hardware/tuning context (see README.md for the full bring-up procedure)

- All angle/rate/gain constants in `config.h` are expressed at the vertical axis, already
  compensated for the 2:1 arm-encoder and motor gear ratios (`ARM_ENC_RATIO`, `MOTOR_GEAR_RATIO`).
- `ARM_SIGN`, `PEND_SIGN`, and the physical pendulum params (`PEND_MASS`, `PEND_LCOM`, `PEND_LEN`)
  are unverified placeholders — the swing-up energy calc (`Settings::cfg.eTop()`/`pendJ()`)
  directly depends on the mass/length values being measured from the real part. These (and all
  gains) are now tunable live via the "Reglages" menu and saved to EEPROM, rather than requiring
  a recompile.
  `KE_SWING`, `K_ALPHA`, `K_ADOT`, `K_TH`, `K_THD`, `KP_VEL`/`KI_VEL` are starting
  points to be tuned empirically per the README's ordered bring-up steps — don't reorder that
  sequence (unpowered encoder check → motor sign test → recalibrate → balance-only → swing-up →
  Q-learning) since later steps assume earlier ones are already correct.
- **A slip ring is fitted on the main arm axis**, so the pendulum-encoder wiring never winds up and
  the arm may rotate indefinitely. `THETA_TURNS_MAX` is therefore *not* protecting hardware — it is
  an optional runaway guard and may be set to 0 (unlimited). Do not reintroduce advice about
  cable wrap. `K_TH` may likewise be 0 (no need to hold the arm near a home position), though a
  small value is still useful to keep the arm from wandering during balance.
- `QL_THETA_TURNS` (Q-learning) is unrelated to the slip ring: it bounds an *episode*, giving a
  consistent start state and a terminal penalty that discourages the spin exploit. Keep it
  non-zero even with unlimited mechanical rotation.
- `DUTY_LIMIT = 0.30` throttles peak current draw for the 15V/3A USB-PD supply; raise only
  gradually while watching for PD-supply cutout.
- Motor driver is a **DRV8871** (single H-bridge, IN1/IN2 sign-magnitude on pins 22/23, same
  scheme `motor.cpp` already drives). PWM runs at 20 kHz (`PWM_FREQ_HZ`) — inaudible, and the
  DRV8871 tolerates it (the earlier ZK-BM1 was capped at 2 kHz, hence audible whine).
