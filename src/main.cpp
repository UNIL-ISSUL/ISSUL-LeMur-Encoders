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
  i2cMultiplexer.selectChannel(channel);
  delayMicroseconds(50);
  
  if (current_mA < CURRENT_MIN_MA) current_mA = CURRENT_MIN_MA;
  if (current_mA > CURRENT_MAX_MA) current_mA = CURRENT_MAX_MA;
  
  uint16_t dac_val = dac_4mA + ((current_mA - CURRENT_MIN_MA) * (float)(dac_20mA - dac_4mA) / 16.0f);
  
  Wire.beginTransmission(I2C_DAC_ADDR);
  Wire.write(0x02);
  Wire.write((dac_val << 4) & 0xFF);
  Wire.write((dac_val >> 4) & 0xFF);
  Wire.endTransmission();
}

// Helper: Compute encoder speed in pulses per second
float compute_encoder_speed(int32_t delta_count, uint32_t delta_time) {
  if (delta_time == 0) return 0.0f;
  float encoder_speed = 1e6f * (float)delta_count / (float)delta_time;
  return fabs(encoder_speed);
}

void setup() {
  // Initialize M5Atom
  M5.begin(false, true, true);
  Serial.begin(DEBUG_BAUDRATE);
  Serial.setTimeout(10); // Non-blocking short timeout
  Serial.flush();
  delay(100);
  Serial.println("M5Atom initialized @ " + String(DEBUG_BAUDRATE) + " baud");

  // Initialize Modbus RTU Slave
  Serial2.begin(MODBUS_BAUDRATE, SERIAL_8E1, MODBUS_RX_PIN, MODBUS_TX_PIN);
  slave.begin(MODBUS_BAUDRATE);
  slave.setCoils(coils, MODBUS_NUM_COILS);
  slave.setHoldingRegisters(registers, MODBUS_NUM_REGISTERS);
  
  delay(1000); // Avoid power-on bus glitches

  // I2C communication
  Wire.setClock(I2C_FREQ_HZ);
  uint8_t i2c_error = 0;

  // Initialize I2C Multiplexer
  if (i2cMultiplexer.begin()) {
    Serial.println("I2C Multiplexer initialized");
  } else {
    Serial.println("I2C Multiplexer initialization failed");
    i2c_error++;
  }

  // Detect and initialize encoders
  for (int i = 0; i < max_channel; i++) {
    i2cMultiplexer.selectChannel(i);
    bool connected = i2cMultiplexer.isConnected(I2C_ENC_ADDR);
    if (connected) {
      Serial.println("Encoder device found on channel " + String(i));
      encoders[i].begin(&Wire, I2C_ENC_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
      Serial.println("Encoder initialized");
    } else {
      Serial.println("Encoder device not found on channel " + String(i));
      i2c_error++;
    }
  }

  // Verify 4-20mA DAC devices
  for (int i = 2; i < 4; i++) {
    i2cMultiplexer.selectChannel(i);
    if (i2cMultiplexer.isConnected(I2C_DAC_ADDR)) {
      Serial.println("DAC device found on channel " + String(i));
    } else {
      Serial.println("DAC device not found on channel " + String(i));
      i2c_error++;
    }
  }

  // Initialize Lift hardware (GP8413 10V DAC + ADS1110 240fps ADC)
  if (lift.init() != 0) {
    Serial.println("Lift initialization failed");
    i2c_error++;
  } else {
    Serial.println("Lift initialized");
  }

  // Configure Lift PID
  liftPID.SetTunings(liftKp, liftKi, liftKd);
  liftPID.SetSampleTimeUs(UPDATE_PERIOD_US);
  liftPID.SetOutputLimits(LIFT_PID_OUT_MIN, LIFT_PID_OUT_MAX);
  liftPID.SetDerivativeMode(QuickPID::dMode::dOnMeas);
  liftPID.SetMode(QuickPID::Control::manual); // Start in safe manual mode
  lift.stop();

  // Warm up Lift sensor and initialize setpoint
  for (int i = 0; i < 50; i++) {
    lift.update();
    delay(2);
  }
  liftSetpoint = lift.getHeight_mm();
  liftInput = liftSetpoint;
  uint16_t init_angle_centideg = (uint16_t)constrain(round(lift.getInclinaison_deg() * 100.0f), 0, 9000);
  registers[1] = init_angle_centideg;
  registers[2] = init_angle_centideg;

  // Abort if I2C bus error detected
  if (i2c_error > 0) {
    Serial.println("I2C initialisation " + String(i2c_error) + " error detected : Stopping execution");
    M5.dis.drawpix(0, CRGB::Red);
    while (1);
  }
  
  // Initialize hardware timer @ 200 Hz (5ms)
  timer = timerBegin(3, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, UPDATE_PERIOD_US, true);
  timerAlarmEnable(timer);

  // Initialize LED display
  M5.dis.drawpix(0, CRGB::Green);

  // Initialize digital filters
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

    // 4. Update Lift sensor & compute PID
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

    // 10. Real-time Teleplot streaming @ 200 Hz
    if (debug) {
      uint32_t now = millis();
      // Speeds group
      teleplot_print_group("Speed", "Belt", belt_encoder_speed, now, "mm/s");
      teleplot_print_group("Speed", "Steps", steps_encoder_speed, now, "mm/s");
      
      // Lift Height & Control group
      teleplot_print_group("Lift_Height", "Measured", liftInput, now, "mm");
      teleplot_print_group("Lift_Height", "Setpoint", liftSetpoint, now, "mm");
      teleplot_print_group("Lift_Angle", "Inclinaison", incl_deg, now, "deg");
      teleplot_print_group("Lift_Motor", "Output", liftOutput, now, "%");

      // 4-20mA DAC Feedback group
      teleplot_print_group("DAC_4_20mA", "Speed_mA", speed_mA, now, "mA");
      teleplot_print_group("DAC_4_20mA", "Incl_mA", incl_mA, now, "mA");

      // System states (text telemetry)
      teleplot_print_text("Encoder_Mode", coils[0] ? "STEPS" : "BELT", now, "Status");
      teleplot_print_text("Lift_PID", (liftPID.GetMode() == (uint8_t)QuickPID::Control::automatic) ? "AUTO" : "MANUAL", now, "Status");
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


