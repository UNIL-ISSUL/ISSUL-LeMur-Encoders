#pragma once
#include <Arduino.h>

// ============================================================================
// COMMUNICATION & BAUDRATES
// ============================================================================
#define DEBUG_BAUDRATE        115200UL   // USB Serial debug & Teleplot stream
#define MODBUS_BAUDRATE       19200UL    // RS485 Modbus RTU communication
#define TELEPLOT_BATCH_SIZE   10         // Batch size for 200Hz Teleplot stream (50ms)

// ============================================================================
// PIN DEFINITIONS (M5Stack ATOM)
// ============================================================================
// I2C Pins
#define I2C_SDA_PIN           G25
#define I2C_SCL_PIN           G21
#define I2C_FREQ_HZ           100000UL

// Modbus RS485 Pins
#define MODBUS_RX_PIN         33
#define MODBUS_TX_PIN         23

// Lift Direction Output Pins (Relay / Transistor drivers)
#define LIFT_UP_PIN           G22
#define LIFT_DOWN_PIN         G19

// ============================================================================
// I2C ADDRESSES & MULTIPLEXER (TCA9548)
// ============================================================================
#define I2C_MUX_ADDR          0x70       // TCA9548 Multiplexer address
#define I2C_ENC_ADDR          0x59       // M5Unit-ExtEncoder address (on PaHub ports 0 & 1)
#define I2C_ADC_ADDR          0x48       // ADS1110 ADC address (Lift angle on main bus)
#define I2C_LIFT_DAC_ADDR     0x5A       // GP8413 0-5V DAC (Lift on main bus, address: 0x5A / A0A1A2=010)
#define I2C_DAC_ADDR          0x58       // DFR0972 4-20mA DACs (on PaHub ports 2 & 3)

// TCA9548 Multiplexer Channels (PaHub Ports: 0 to 3)
#define MUX_CH_BELT_ENC       0          // Port 0: Belt encoder (0x59)
#define MUX_CH_STEPS_ENC      1          // Port 1: Steps encoder (0x59)
#define MUX_CH_SPEED_DAC      2          // Port 2: 4-20mA Speed DAC (0x58)
#define MUX_CH_INCL_DAC       3          // Port 3: 4-20mA Inclination DAC (0x58)
#define MUX_MAX_ENC_CHANNELS  2

// ============================================================================
// TIMING & REAL-TIME PARAMETERS
// ============================================================================
#define SAMPLING_FREQ_HZ      200.0f
#define UPDATE_PERIOD_US      5000       // 5ms tick (200 Hz)

// ============================================================================
// MECHANICAL & SENSOR PARAMETERS
// ============================================================================
#define BELT_PULSES_PER_REV   (10000 * 2)
#define STEPS_PULSES_PER_REV  (10000 * 2)
#define BELT_PERIMETER_MM     (100.0f * PI)    // Roller diameter 100 mm
#define STEPS_PERIMETER_MM    (126.7f * PI)    // Pulley diameter 126.7 mm

// 4-20mA Current Output Scaling
#define MAX_BELT_SPEED_MM_S   12000.0f   // 40 km/h
#define MAX_STEPS_SPEED_MM_S  3000.0f    // 1 m/s
#define CURRENT_MIN_MA        4.0f
#define CURRENT_MAX_MA        20.0f
#define DFR0972_DAC_4MA_RAW   654
#define DFR0972_DAC_20MA_RAW  3279

// ============================================================================
// LIFT PID REGULATION PARAMETERS
// ============================================================================
#define LIFT_PID_KP_DEFAULT       0.8f       // Proportional gain (% / mm)
#define LIFT_PID_KI_DEFAULT       0.0f       // Integral gain (0 = no integrator windup)
#define LIFT_PID_KD_DEFAULT       0.10f      // Derivative damping gain (seconds)
#define LIFT_PID_OUT_MIN          -100.0f    // -100% to +100%
#define LIFT_PID_OUT_MAX          100.0f
#define LIFT_MIN_OUTPUT_PCT       1.0f       // Minimum output threshold (< 1% => motor stopped)
#define LIFT_SENSOR_FILTER_ALPHA  0.20f      // Fast low-pass filter (tau = 25ms @ 200 Hz)
#define LIFT_ANGLE_MIN_DEG        0.0f
#define LIFT_ANGLE_MAX_DEG        90.0f

// ============================================================================
// MODBUS RTU CONFIGURATION
// ----------------------------------------------------------------------------
// COILS:
//   coils[0] : Encoder Mode Selection (false: Belt, true: Steps) [Master -> Slave]
//   coils[1] : Encoder Feedback Active (use_feedback: true when dac_value > 0) [Slave -> Master]
//   coils[2] : Lift PID Enable (false: Manual/Stop, true: Automatic) [Master -> Slave]
//
// HOLDING REGISTERS:
//   registers[0] : Measured speed in mm/s (Read-only, encoder_feedback_speed)
//   registers[1] : Lift setpoint in centidegrees 0.01° (Write by Master, lift_angle_SP)
//   registers[2] : Lift measured inclination in centidegrees 0.01° (Read-only, lift_angle_current)
// ============================================================================
#define MODBUS_SLAVE_ADDR     3
#define MODBUS_NUM_COILS      3
#define MODBUS_NUM_REGISTERS  3
