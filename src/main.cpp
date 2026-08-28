#include <Arduino.h>
#include <M5Atom.h>
#include <Wire.h>
#include "UNIT_EXT_ENCODER.h"
#include "TCA9548.h"
#include <RunningMedian.h>
#include <ModbusRTUSlave.h>
#include <math.h>
#include "config.h"
#include "filter.h"
#include "teleplot.h"
#include <DFRobot_GP8XXX.h>
#include <QuickPID.h>
#include "Lift.h"
#include "system_init.h"

// Lift instance & PID
Lift lift(LIFT_UP_PIN, LIFT_DOWN_PIN);
float liftSetpoint = 0.0f, liftInput = 0.0f, liftOutput = 0.0f;
float liftKp = LIFT_PID_KP_DEFAULT, liftKi = LIFT_PID_KI_DEFAULT, liftKd = LIFT_PID_KD_DEFAULT;
QuickPID liftPID(&liftInput, &liftOutput, &liftSetpoint);

// Encoders & I2C Multiplexer
UNIT_EXT_ENCODER encoders[MUX_MAX_ENC_CHANNELS];
uint32_t pulse[MUX_MAX_ENC_CHANNELS] = {BELT_PULSES_PER_REV, STEPS_PULSES_PER_REV};
float perimeter[MUX_MAX_ENC_CHANNELS] = {BELT_PERIMETER_MM, STEPS_PERIMETER_MM};
TCA9548 i2cMultiplexer(I2C_MUX_ADDR);
byte max_channel = MUX_MAX_ENC_CHANNELS;

// Median & IIR Low-pass Filters
RunningMedian belt_encoder_speed_median = RunningMedian(7);
RunningMedian steps_encoder_speed_median = RunningMedian(7);
IIRFilter belt_speed_filter;
IIRFilter encoder_speed_filter;

const int filter_order = 3;
const float b_coeffs[] = {
    0.0000954425f,
    0.0002863275f,
    0.0002863275f,
    0.0000954425f,
};
const float a_coeffs[] = {
    1.0000000000f,
    -2.8115736773f,
    2.6404834928f,
    -0.8281462754f,
};

// Modbus RTU Slave
ModbusRTUSlave slave(Serial2, MODBUS_SLAVE_ADDR);
bool coils[MODBUS_NUM_COILS] = {false, false};
uint16_t registers[MODBUS_NUM_REGISTERS] = {0, 0, 0};

// Hardware Timer & ISR flag
hw_timer_t *timer = NULL;
volatile bool flag_read_encoder = false;

void IRAM_ATTR onTimer() {
  flag_read_encoder = true;
}

// Helper: Extract float value from serial command (e.g. "Kp=0.5")
float getValue(const String &data, char separator = '=') {
  int separatorIndex = data.indexOf(separator);
  if (separatorIndex >= 0) {
    return data.substring(separatorIndex + 1).toFloat();
  }
  return 0.0f;
}

// Helper: Read encoder value via multiplexer
uint32_t readEncoder(uint8_t channel, TCA9548 *multiplexer, bool mm = false) {
  if (channel >= max_channel) {
    Serial.println("No encoder on requested channel : " + String(channel));
    return 0;
  }
  multiplexer->selectChannel(channel);
  if (mm) return encoders[channel].getMeterValue();
  else    return encoders[channel].getEncoderValue();
}

// Helper: Output 4-20mA current via DFR0972 DAC on multiplexer
void set_DFR0972_mA(uint8_t channel, float current_mA, uint16_t dac_4mA, uint16_t dac_20mA) {
  set_DFR0972_mA(i2cMultiplexer, channel, current_mA, dac_4mA, dac_20mA);
}

// Helper: Compute encoder speed in pulses per second
float compute_encoder_speed(int32_t delta_count, uint32_t delta_time) {
  if (delta_time == 0) return 0.0f;
  float encoder_speed = 1e6f * (float)delta_count / (float)delta_time;
  return fabs(encoder_speed);
}

void setup() {
  // 1. Initialize M5Atom core
  M5.begin(false, true, true);

  // 2. Perform full system diagnostics, sensor scan and hardware timer startup
  if (!system_hardware_init(i2cMultiplexer, encoders, max_channel, lift, slave, coils, registers, timer, onTimer)) {
    while (1) {
      delay(500);
    }
  }

  // 3. Configure Lift PID
  liftPID.SetTunings(liftKp, liftKi, liftKd);
  liftPID.SetSampleTimeUs(UPDATE_PERIOD_US);
  liftPID.SetOutputLimits(LIFT_PID_OUT_MIN, LIFT_PID_OUT_MAX);
  liftPID.SetDerivativeMode(QuickPID::dMode::dOnMeas);
  liftPID.SetMode(QuickPID::Control::manual);
  lift.stop();
  liftSetpoint = lift.getHeight_mm();
  liftInput = liftSetpoint;

  // 4. Initialize digital speed filters
  belt_speed_filter.init(filter_order, b_coeffs, a_coeffs);
  encoder_speed_filter.init(filter_order, b_coeffs, a_coeffs);
}

bool debug = false;

uint32_t belt_encoder_count = 0;
uint32_t steps_encoder_count = 0;
uint32_t last_belt_encoder_count = 0;
uint32_t last_steps_encoder_count = 0;
uint32_t time_us = 0;
uint32_t last_time_us = 0;

void loop() {
  M5.update();
  
  // --------------------------------------------------------------------------
  // 200 Hz SYNCHRONOUS CONTROL LOOP (Every 5 ms)
  // --------------------------------------------------------------------------
  if (flag_read_encoder) {
    // 1. Read quadrature encoders
    belt_encoder_count = readEncoder(MUX_CH_BELT_ENC, &i2cMultiplexer);
    steps_encoder_count = readEncoder(MUX_CH_STEPS_ENC, &i2cMultiplexer);
    time_us = micros();

    // 2. Compute speeds (pulses/s) with median filter
    belt_encoder_speed_median.add(compute_encoder_speed(belt_encoder_count - last_belt_encoder_count, time_us - last_time_us));
    steps_encoder_speed_median.add(compute_encoder_speed(steps_encoder_count - last_steps_encoder_count, time_us - last_time_us));
    float belt_encoder_speed = belt_encoder_speed_median.getMedian();
    float steps_encoder_speed = steps_encoder_speed_median.getMedian();
    last_belt_encoder_count = belt_encoder_count;
    last_steps_encoder_count = steps_encoder_count;
    last_time_us = time_us;

    // 3. Convert to linear speed (mm/s) and apply low-pass filter
    belt_encoder_speed = belt_encoder_speed * perimeter[0] / pulse[0];
    steps_encoder_speed = steps_encoder_speed * perimeter[1] / pulse[1];
    float filtered_belt_speed = belt_speed_filter.filter(belt_encoder_speed);
    float filtered_steps_speed = encoder_speed_filter.filter(steps_encoder_speed);

    // 4. Update Lift sensor & compute PID (Main I2C bus via 3-way Hub)
    i2cMultiplexer.disableAllChannels();
    delayMicroseconds(10);
    lift.update();
    liftInput = lift.getHeight_mm();
    if (liftPID.GetMode() == (uint8_t)QuickPID::Control::automatic) {
      liftPID.Compute();
      lift.move(liftOutput);
    }

    // 5. Select active speed feedback
    uint16_t dac_value;
    if (!coils[0]) dac_value = (int16_t)round(belt_encoder_speed);
    else          dac_value = (int16_t)round(steps_encoder_speed);

    // 6. Calculate 4-20mA currents
    float speed_mA;
    if (!coils[0]) speed_mA = CURRENT_MIN_MA + (16.0f * belt_encoder_speed / MAX_BELT_SPEED_MM_S);
    else          speed_mA = CURRENT_MIN_MA + (16.0f * steps_encoder_speed / MAX_STEPS_SPEED_MM_S);
    if (speed_mA > CURRENT_MAX_MA) speed_mA = CURRENT_MAX_MA;
    if (speed_mA < CURRENT_MIN_MA) speed_mA = CURRENT_MIN_MA;

    float incl_deg = lift.getInclinaison_deg();
    float incl_mA = CURRENT_MIN_MA + (incl_deg * 16.0f / LIFT_ANGLE_MAX_DEG);
    if (incl_mA > CURRENT_MAX_MA) incl_mA = CURRENT_MAX_MA;
    if (incl_mA < CURRENT_MIN_MA) incl_mA = CURRENT_MIN_MA;

    // 7. Output to 4-20mA DACs (Channels 2 and 3)
    set_DFR0972_mA(MUX_CH_SPEED_DAC, speed_mA, DFR0972_DAC_4MA_RAW, DFR0972_DAC_20MA_RAW);
    set_DFR0972_mA(MUX_CH_INCL_DAC, incl_mA, DFR0972_DAC_4MA_RAW, DFR0972_DAC_20MA_RAW);

    // 8. Update Modbus holding registers
    registers[0] = dac_value;
    registers[2] = (uint16_t)constrain(round(incl_deg * 100.0f), 0, 9000);

    // 9. Reset flag
    flag_read_encoder = false;

    // 10. Real-time Teleplot streaming with 10-sample batching (50ms packets @ 115200 baud)
    if (debug) {
      static uint32_t batch_timestamps[TELEPLOT_BATCH_SIZE];
      static float batch_belt_raw[TELEPLOT_BATCH_SIZE];
      static float batch_belt_filt[TELEPLOT_BATCH_SIZE];
      static float batch_steps_raw[TELEPLOT_BATCH_SIZE];
      static float batch_steps_filt[TELEPLOT_BATCH_SIZE];
      static uint8_t batch_count = 0;

      uint32_t now = millis();
      batch_timestamps[batch_count] = now;
      batch_belt_raw[batch_count] = belt_encoder_speed;
      batch_belt_filt[batch_count] = filtered_belt_speed;
      batch_steps_raw[batch_count] = steps_encoder_speed;
      batch_steps_filt[batch_count] = filtered_steps_speed;
      batch_count++;

      if (batch_count >= TELEPLOT_BATCH_SIZE) {
        // High-speed 200 Hz encoder batches (10 points every 50ms - 100% glitch resolution)
        teleplot_print_batch("Speed/Belt_Raw", batch_belt_raw, batch_timestamps, TELEPLOT_BATCH_SIZE, "mm/s");
        teleplot_print_batch("Speed/Belt_Filt", batch_belt_filt, batch_timestamps, TELEPLOT_BATCH_SIZE, "mm/s");
        teleplot_print_batch("Speed/Steps_Raw", batch_steps_raw, batch_timestamps, TELEPLOT_BATCH_SIZE, "mm/s");
        teleplot_print_batch("Speed/Steps_Filt", batch_steps_filt, batch_timestamps, TELEPLOT_BATCH_SIZE, "mm/s");

        // 20 Hz low-frequency curves (Lift, DACs, and Status)
        teleplot_print_group("Lift_Height", "Measured", liftInput, now, "mm");
        teleplot_print_group("Lift_Height", "Setpoint", liftSetpoint, now, "mm");
        teleplot_print_group("Lift_Angle", "Inclinaison", incl_deg, now, "deg");
        teleplot_print_group("Lift_Motor", "Output", liftOutput, now, "%");
        teleplot_print_group("DAC_4_20mA", "Speed_mA", speed_mA, now, "mA");
        teleplot_print_group("DAC_4_20mA", "Incl_mA", incl_mA, now, "mA");
        teleplot_print_text("Encoder_Mode", coils[0] ? "STEPS" : "BELT", now, "Status");
        teleplot_print_text("Lift_PID", (liftPID.GetMode() == (uint8_t)QuickPID::Control::automatic) ? "AUTO" : "MANUAL", now, "Status");

        batch_count = 0;
      }
    }
  }
  // --------------------------------------------------------------------------
  // ASYNCHRONOUS BACKGROUND TASKS (During the ~3.8 ms idle window)
  // --------------------------------------------------------------------------
  else {
    // 1. Modbus RTU Slave update
    slave.update();
    if (slave.hasException()) {
      if (!debug) Serial.println("MODBUS exception: " + String(slave.getExceptionMessage()));
      slave.clearException();
    }

    // Coil 0: Encoder Mode (Belt vs Steps)
    static bool last_coil0 = false;
    if (coils[0] != last_coil0) {
      last_coil0 = coils[0];
      if (!debug) Serial.println("Modbus Encoder Mode: " + String(coils[0] ? "Steps" : "Belt"));
    }

    // Coil 1: Lift PID Enable (Automatic vs Manual)
    static bool last_coil1 = false;
    if (coils[1] != last_coil1) {
      last_coil1 = coils[1];
      if (coils[1]) {
        liftPID.Reset();
        liftPID.SetMode(QuickPID::Control::automatic);
        if (!debug) Serial.println("Lift PID Auto mode enabled via Modbus (Coil 1 = 1)");
      } else {
        liftPID.SetMode(QuickPID::Control::manual);
        lift.stop();
        if (!debug) Serial.println("Lift PID Manual/Stop mode via Modbus (Coil 1 = 0)");
      }
    }

    // Register 1: Lift Setpoint Angle (0.01 deg)
    static uint16_t last_reg_setpoint = 0;
    if (registers[1] != last_reg_setpoint) {
      last_reg_setpoint = registers[1];
      float setpoint_deg = (float)registers[1] / 100.0f;
      setpoint_deg = constrain(setpoint_deg, LIFT_ANGLE_MIN_DEG, LIFT_ANGLE_MAX_DEG);
      liftSetpoint = lift.computeHeight(setpoint_deg);
      if (!debug) Serial.println("Lift Setpoint updated via Modbus: " + String(setpoint_deg) + " deg (" + String(liftSetpoint) + " mm)");
    }
  }

  // Button interaction: Toggle Teleplot debug mode
  if (M5.Btn.wasPressed()) {
    debug = !debug;
    if (debug) {
      M5.dis.drawpix(0, CRGB::Orange);
      Serial.println("Debug Teleplot stream enabled");
      for (int i = 0; i < max_channel; i++) {
        i2cMultiplexer.selectChannel(i);
        encoders[i].resetEncoder();
      }
    } else {
      M5.dis.drawpix(0, CRGB::Green);
      Serial.println("Debug Teleplot stream disabled");
    }
  }

  // Serial CLI: Runtime tuning and manual control
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.println("command:" + command);
    if (command == "up") {
      liftPID.SetMode(QuickPID::Control::manual);
      lift.moveUp();
    } else if (command == "down") {
      liftPID.SetMode(QuickPID::Control::manual);
      lift.moveDown();
    } else if (command == "stop") {
      liftPID.SetMode(QuickPID::Control::manual);
      lift.stop();
    } else if (command == "auto") {
      coils[1] = true;
      liftPID.Reset();
      liftPID.SetMode(QuickPID::Control::automatic);
      Serial.println("PID Auto mode enabled");
    } else if (command == "manual") {
      coils[1] = false;
      liftPID.SetMode(QuickPID::Control::manual);
      lift.stop();
      Serial.println("PID Manual mode enabled");
    } else if (command == "reset") {
      liftPID.Reset();
    } else if (command.startsWith("speed")) {
      float spd = getValue(command);
      liftPID.SetMode(QuickPID::Control::manual);
      Serial.println("speed:" + String(spd));
      lift.setSpeed(spd);
    } else if (command.startsWith("liftSP")) {
      liftSetpoint = getValue(command);
      Serial.println("liftSetPoint:" + String(liftSetpoint));
    } else if (command.startsWith("step")) {
      float stepVal = getValue(command);
      liftSetpoint += stepVal;
      Serial.println("liftSetPoint:" + String(liftSetpoint));
    } else if (command.startsWith("Kp")) {
      liftKp = getValue(command);
      liftPID.SetTunings(liftKp, liftKi, liftKd);
      Serial.println("Kp:" + String(liftKp));
    } else if (command.startsWith("Ki")) {
      liftKi = getValue(command);
      liftPID.SetTunings(liftKp, liftKi, liftKd);
      Serial.println("Ki:" + String(liftKi));
    } else if (command.startsWith("Kd")) {
      liftKd = getValue(command);
      liftPID.SetTunings(liftKp, liftKi, liftKd);
      Serial.println("Kd:" + String(liftKd));
    } else if (command.startsWith("getK")) {
      Serial.println("Kp:" + String(liftPID.GetKp()));
      Serial.println("Ki:" + String(liftPID.GetKi()));
      Serial.println("Kd:" + String(liftPID.GetKd()));
    } else if (command.startsWith("setAngle")) {
      float angle = getValue(command);
      angle = constrain(angle, LIFT_ANGLE_MIN_DEG, LIFT_ANGLE_MAX_DEG);
      liftSetpoint = lift.computeHeight(angle);
      registers[1] = (uint16_t)round(angle * 100.0f);
      Serial.println("setAngle:" + String(angle));
      Serial.println("liftSetPoint:" + String(liftSetpoint));
    }
  }
}


