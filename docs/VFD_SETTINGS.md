# Mitsubishi FR-D700 VFD & Firmware Configuration Guide

This document describes the optimal parameter settings for the **Mitsubishi FR-D700** variable frequency drive (VFD) coupled with the mechanical holding brake and the non-linear square-root lift controller on the ESP32 (M5Stack ATOM).

---

## 1. VFD Parameter Settings (FR-D700)

| Parameter | Function | Value | Description |
| :--- | :--- | :--- | :--- |
| **`Pr. 13`** | Starting Frequency | **`2.0 Hz`** | Minimum output frequency when motion is commanded. Matched to firmware `LIFT_MIN_DRIVE_PCT = 4.0%` ($4\% \times 50\text{ Hz} = 2.0\text{ Hz}$). |
| **`Pr. 152`** | Zero Current Detection Level | **`2.0 %`** | Threshold current above which output signal **Y13** activates (releasing the mechanical brake under magnetic flux). |
| **`Pr. 153`** | Zero Current Detection Delay | **`0.05 s`** | Delay before signal **Y13** drops when current falls to zero. Setting this to $0.05\text{ s}$ ($50\text{ ms}$) ensures immediate, shock-free mechanical brake locking at standstill. |

---

## 2. Firmware Controller Parameters (`include/config.h` & `lib/Lift/Lift.h`)

```cpp
// Non-Linear Square-Root Controller
#define LIFT_SQRT_GAIN_DEFAULT    5.0f       // Square-root gain (% / sqrt(mm))
#define LIFT_DEADBAND_DEG_DEFAULT 0.05f      // Deadband angle tolerance (± 0.05 deg)
#define LIFT_MIN_DRIVE_PCT        4.0f       // Minimum moving speed floor (matches Pr. 13 @ 2.0 Hz)
#define LIFT_MAX_DRIVE_PCT        100.0f     // Maximum speed percentage (100% = 5V DAC)

// Inclinometer ADC Resolution (ADS1110 14-bit @ 60 SPS)
// 1 LSB = 250 uV -> ~0.029 deg per LSB (Window of ±0.05 deg contains ~3.5 discrete steps)
#define ADC_CONFIG_DEFAULT        ADC_CONFIG_60SPS_14BIT
#define ADC_MIN_CODE_DEFAULT      ADC_MIN_CODE_14BIT
```

---

## 3. Dynamic Sequence & Positioning Latch

```mermaid
sequenceDiagram
    participant ESP as ESP32 (M5Stack)
    participant VFD as Mitsubishi FR-D700
    participant Brake as Mechanical Brake
    participant Motor as Actuator Leadscrew

    Note over ESP,Motor: Motion Commanded (Outside Deadband > ±0.05°)
    ESP->>VFD: FORWARD/BACKWARD = HIGH, DAC >= 4% (2.0 Hz)
    VFD->>Motor: Magnetizing Current > Pr. 152 (2.0%)
    VFD->>Brake: Signal Y13 Closes -> Brake Releases under torque
    Motor->>Motor: Smooth constant-deceleration motion (v = 5.0 * sqrt(|e|))

    Note over ESP,Motor: Target Reached (Inside Deadband <= ±0.05°)
    ESP->>VFD: FORWARD/BACKWARD = LOW, DAC = 0 V
    VFD->>Motor: Output Current drops to 0 A
    Note over VFD,Brake: Pr. 153 Delay (0.05s)
    VFD->>Brake: Signal Y13 Opens -> Brake locks firmly at zero speed
    Note over ESP: liftTargetReached = true (Latched Stopped)

    Note over ESP,Motor: Subject Climbs / Sensor Noise (|error| > ±0.05°)
    Note over ESP: Drive remains strictly STOPPED & locked until setpoint changes!
```
