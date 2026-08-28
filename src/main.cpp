#include <Arduino.h>
#include <M5Atom.h>
#include <Wire.h>
#include "UNIT_EXT_ENCODER.h"
#include "TCA9548.h"
#include <RunningMedian.h>
#include <ModbusRTUSlave.h>
#include <math.h>
#include "filter.h"
#include "teleplot.h"
#include <DFRobot_GP8XXX.h>
#include <QuickPID.h>
#include "Lift.h"

//lift definition & PID
#define LIFT_UP_PIN   G22
#define LIFT_DOWN_PIN G19
Lift lift(LIFT_UP_PIN, LIFT_DOWN_PIN);

float liftSetpoint = 0.0f, liftInput = 0.0f, liftOutput = 0.0f;
float liftKp = 0.4f, liftKi = 0.0f, liftKd = 0.0f;
QuickPID liftPID(&liftInput, &liftOutput, &liftSetpoint);

//helper function to extract float value from serial command (e.g. "Kp=0.5")
float getValue(const String &data, char separator = '=') {
  int separatorIndex = data.indexOf(separator);
  if (separatorIndex >= 0) {
    return data.substring(separatorIndex + 1).toFloat();
  }
  return 0.0f;
}

//i2c wire communication
#define I2C_FREQ 400000UL
#define SDA G25
#define SCL G21

//i2c ext-encoder communication
UNIT_EXT_ENCODER encoders[2]; //two encoders 0: belt, 1: steps
//encoder configuration
uint32_t pulse[2] = {10000*2,10000*2};  // belt and steps encoder pulse per revolution
float perimeter[2] = {100.0*PI,126.7*PI}; // belt roller and steps pulley diameter 123.7 mm
//speed computation function
float compute_encoder_speed(int32_t delta_count,uint32_t delta_time);
float compute_encoder_speed_2(uint32_t encoder_count,uint32_t time_us);

//i2c multiplexer communication
//two encoders on the i2c multiplexer at address 0x70
//one ADC directly connected to the i2c bus
TCA9548 i2cMultiplexer(0x70);
byte max_channel = 2;
#define ADC_ADDR 0x48
#define ENC_ADDR 0x59
#define UPDATE_PERIOD_US 5000 //5ms
#define DAC_ADDR 0x58 //DAC address GP8302

//running median filter
RunningMedian belt_encoder_speed_median = RunningMedian(7);
RunningMedian steps_encoder_speed_median = RunningMedian(7);

// IIR Butterworth Low-pass Filter for belt speed
IIRFilter belt_speed_filter;
IIRFilter encoder_speed_filter;
// Sampling Frequency (fs) = 200.0 Hz, Cutoff Frequency (fc) = 3.0 Hz
const int filter_order = 3;
const float b_coeffs[] = {
    0.0000954425,
    0.0002863275,
    0.0002863275,
    0.0000954425,
};
const float a_coeffs[] = {
    1.0000000000,
    -2.8115736773,
    2.6404834928,
    -0.8281462754,
};

// ============================================================================
// MODBUS RTU SLAVE CONFIGURATION (Serial2 @ 19200 baud, 8E1, Address 3)
// ----------------------------------------------------------------------------
// COILS (Bit-level Read/Write):
//   coils[0] : Encoder Mode Selection
//              - false (0) : Tapis roulant (Belt encoder)
//              - true  (1) : Marches d'escalier (Steps encoder)
//   coils[1] : Lift PID Enable
//              - false (0) : PID Désactivé / Manuel (Moteur Lift arrêté)
//              - true  (1) : PID Activé / Automatique (Régulation de hauteur)
//
// HOLDING REGISTERS (16-bit Word Read/Write):
//   registers[0] : Vitesse actuelle mesurée (mm/s) [Lecture seule par le maître]
//                  - Vitesse du tapis ou des marches selon coils[0]
//   registers[1] : Consigne d'inclinaison Lift (centidegrés, 0.01°) [Écriture par le maître]
//                  - Ex: 4500 = 45.00°, 7550 = 75.50° (convertie en mm)
//   registers[2] : Inclinaison actuelle Lift (centidegrés, 0.01°) [Lecture seule par le maître]
//                  - Ex: 4500 = 45.00° (mesurée via ADC ADS1110 240fps)
// ============================================================================
unsigned long baudrate = 19200UL;
#define RX 33
#define TX 23
ModbusRTUSlave slave(Serial2, 3); //modbus slave on serial2, address 3
#define NUM_COILS 2 //number of coils
#define NUM_REGISTERS 3 //number of holding registers
bool coils[NUM_COILS] = {false, false};  //coil 0: mode, coil 1: lift PID enable
uint16_t registers[NUM_REGISTERS] = {0, 0, 0}; //0: speed (mm/s), 1: lift setpoint (0.01 deg), 2: lift position (0.01 deg)

uint32_t readEncoder(uint8_t channel, TCA9548 *multiplexer,bool mm=false) {
  if (channel >= max_channel) {
    Serial.println("No encoder on requested channel : " + String(channel));
    return 0; //channel not available
  }
  multiplexer->selectChannel(channel);
  if(mm)  return encoders[channel].getMeterValue();
  else    return encoders[channel].getEncoderValue();
}

void set_DFR0972_mA(uint8_t channel, float current_mA, uint16_t dac_4mA, uint16_t dac_20mA) {
  // 1. Aiguillage sécurisé via le multiplexeur
  i2cMultiplexer.selectChannel(channel);
  delayMicroseconds(50); // Pause microscopique vitale
  
  // 2. Sécurisation des limites matérielles (4 à 20 mA)
  if (current_mA < 4.0) current_mA = 4.0;
  if (current_mA > 20.0) current_mA = 20.0;
  
  // 3. Calcul proportionnel avec les bornes spécifiques de CE module
  uint16_t dac_val = dac_4mA + ((current_mA - 4.0) * (float)(dac_20mA - dac_4mA) / 16.0);
  
  // 4. Envoi I2C "Bare-Metal" avec l'alignement 12-bits corrigé
  Wire.beginTransmission(0x58);
  Wire.write(0x02);                   // Registre de sortie DAC
  Wire.write((dac_val << 4) & 0xFF);  // Poids faible (décalé de 4 bits à gauche)
  Wire.write((dac_val >> 4) & 0xFF);  // Poids fort
  Wire.endTransmission();
}

//define timer to measure speed at regular interval
//https://deepbluembedded.com/esp32-timers-timer-interrupt-tutorial-arduino-ide/?utm_content=cmp-true
hw_timer_t *timer = NULL;
bool flag_read_encoder = false;
//ISR function
void IRAM_ATTR onTimer(){
  flag_read_encoder = true;
}


void setup() {
  
  //initialize the M5Atom
  M5.begin(false, true, true); // enable serial, enable I2C, enable display (led)
  Serial.begin(19200); //to avoid communication error du to low transmission and interuption
  Serial.flush();
  delay(100); //wait for serial to initialize
  Serial.println("M5Atom initialized");

  //initialize the modbus communication
  Serial2.begin(baudrate, SERIAL_8E1, RX, TX);
  slave.begin(baudrate);
  //associate coils and registers
  slave.setCoils(coils, NUM_COILS);
  slave.setHoldingRegisters(registers, NUM_REGISTERS); //set holding registers
  
  delay(1000); //avoid initialisation errors

  //I2C communication
  //Wire.begin(SDA, SCL);
  Wire.setClock(I2C_FREQ);  //update clock
  uint8_t i2c_error = 0;
  //initialize I2C multiplexer
  if(i2cMultiplexer.begin()){
    Serial.println("I2C Multiplexer initialized");
  }
  else{
    Serial.println("I2C Multiplexer initialization failed");
    i2c_error += 1;

  }
  //search and configure encoders
  for(int i = 0; i < max_channel; i++){
    i2cMultiplexer.selectChannel(i);
    bool connected = i2cMultiplexer.isConnected(ENC_ADDR);
    
    if(connected){
      Serial.println("Encoder device found on channel " + String(i));
      encoders[i].begin(&Wire, ENC_ADDR, SDA, SCL, I2C_FREQ);
      Serial.println("Encoder initialized");
    }
    else{
      Serial.println("Encoder device not found on channel " + String(i));
      i2c_error += 1;
      //decrease max channel
      //max_channel -= 1;
    }
  }

  //initialize DAC devices
  for(int i = 2; i < 4; i++){
    i2cMultiplexer.selectChannel(i);
    if(i2cMultiplexer.isConnected(DAC_ADDR)){
      Serial.println("DAC device found on channel "+String(i));
    }
    else{
      Serial.println("DAC device not found on channel "+String(i));
      i2c_error += 1;
    }
  }

  //initialize Lift & PID
  if(lift.init() != 0){
    Serial.println("Lift initialization failed");
    i2c_error += 1;
  }
  else{
    Serial.println("Lift initialized");
  }

  //configure lift PID
  liftPID.SetTunings(liftKp, liftKi, liftKd);
  liftPID.SetSampleTimeUs(UPDATE_PERIOD_US); // 5000us (200Hz)
  liftPID.SetOutputLimits(-100, 100);        // -100% to +100%
  liftPID.SetDerivativeMode(QuickPID::dMode::dOnMeas);
  liftPID.SetMode(QuickPID::Control::automatic);

  //warm up lift reading and init setpoint to current height
  for(int i = 0; i < 50; i++){
    lift.update();
    delay(2);
  }
  liftSetpoint = lift.getHeight_mm();
  liftInput = liftSetpoint;
  uint16_t init_angle_centideg = (uint16_t)constrain(round(lift.getInclinaison_deg() * 100.0f), 0, 9000);
  registers[1] = init_angle_centideg;
  registers[2] = init_angle_centideg;

  //stop execution if there is i2c error
  if(i2c_error > 0) {
    Serial.println("I2C initialisation " +String(i2c_error)+" error dectected : Stopping execution");
    M5.dis.drawpix(0, CRGB::Red);
    while(1);
  }
  
  //initialize timer to interupt @ 5ms
  timer = timerBegin(3, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, UPDATE_PERIOD_US, true);
  timerAlarmEnable(timer);

  //initialize display
  M5.dis.drawpix(0, CRGB::Green);

  //initialize the belt speed filter
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
  
  //M5 update for button
  M5.update();
  
  if(flag_read_encoder){
    //read encoder value
    belt_encoder_count = readEncoder(0, &i2cMultiplexer);
    steps_encoder_count = readEncoder(1, &i2cMultiplexer);
    //Serial.println("Belt encoder count: " + String(belt_encoder_count) + " Steps encoder count: " + String(steps_encoder_count));
    time_us = micros();
    //compute speed en pluse/sec
    belt_encoder_speed_median.add(compute_encoder_speed(belt_encoder_count-last_belt_encoder_count, time_us-last_time_us));
    steps_encoder_speed_median.add(compute_encoder_speed(steps_encoder_count-last_steps_encoder_count, time_us-last_time_us));
    float belt_encoder_speed = belt_encoder_speed_median.getMedian();
    float steps_encoder_speed = steps_encoder_speed_median.getMedian();
    last_belt_encoder_count = belt_encoder_count;
    last_steps_encoder_count = steps_encoder_count;
    last_time_us = time_us;
    //apply geometry to compute speed in mm/s
    belt_encoder_speed = belt_encoder_speed*perimeter[0]/pulse[0]; //mm/s
    steps_encoder_speed = steps_encoder_speed*perimeter[1]/pulse[1]; //mm/s
    //filter speeds
    float filtered_belt_speed = belt_speed_filter.filter(belt_encoder_speed);
    float filtered_steps_speed = encoder_speed_filter.filter(steps_encoder_speed);
    //update Lift & compute PID @ 200Hz
    lift.update();
    liftInput = lift.getHeight_mm();
    if(liftPID.GetMode() == (uint8_t)QuickPID::Control::automatic){
      liftPID.Compute();
      lift.move(liftOutput);
    }

    //update DAC output
    uint16_t dac_value;
    //if coil 0 is set to 1, the steps are used else this the belt
    //send raw data to dac 
    if(!coils[0]) dac_value = (int16_t)round(belt_encoder_speed);  
    else dac_value = (int16_t)round(steps_encoder_speed);
    //write DAC output
    // calculate current for speed DAC (Channel 2)
    float speed_mA;
    const uint16_t max_belt_speed_mm_s = 12000; // 40km/h
    const uint16_t max_steps_speed_mm_s = 3000; // 1m/s
    if (!coils[0]) speed_mA = 4.0 + (16.0 * belt_encoder_speed / max_belt_speed_mm_s);
    else speed_mA = 4.0 + (16.0 * steps_encoder_speed / max_steps_speed_mm_s);
    //Serial.println("Speed mA: " + String(speed_mA));
    if (speed_mA > 20.0) speed_mA = 20.0;
    if (speed_mA < 4.0) speed_mA = 4.0;

    // calculate current for inclinaison DAC (Channel 3) from Lift measurement
    float incl_deg = lift.getInclinaison_deg();
    float incl_mA = 4.0 + (incl_deg * 16.0 / 90.0);
    if (incl_mA > 20.0) incl_mA = 20.0;
    if (incl_mA < 4.0) incl_mA = 4.0;

    //send to DACs on I2C multiplexer channels 2 and 3
    set_DFR0972_mA(2, speed_mA, 654, 3279);
    set_DFR0972_mA(3, incl_mA, 654, 3279);
    //print mA sent value
    //Serial.println("Speed mA: " + String(speed_mA) + " Inclinaison mA: " + String(incl_mA));

    //update modbus holding registers
    registers[0] = dac_value; //update holding register with current feedback speed (mm/s)
    registers[2] = (uint16_t)constrain(round(incl_deg * 100.0f), 0, 9000); //current inclination (0.01 deg)

    //reset flag
    flag_read_encoder = false;

    // Teleplot output for debugging
    if (debug) {
      uint32_t now = millis();
      teleplot_print("belt_speed", belt_encoder_speed, now);
      //teleplot_print("belt_filtered_speed", filtered_belt_speed, now);
      teleplot_print("steps_speed", steps_encoder_speed, now);
      //teleplot_print("steps_filtered_speed", filtered_steps_speed, now);
      teleplot_print("inclinaison", incl_deg, now);
      teleplot_print("inclinaison_mA", incl_mA, now);
      teleplot_print("speed_mA", speed_mA, now);
      teleplot_print("lift_height", liftInput, now);
      teleplot_print("lift_setpoint", liftSetpoint, now);
      teleplot_print("lift_output", liftOutput, now);
      teleplot_print("lift_inclinaison", lift.getInclinaison_deg(), now);
    }
  }
  //if encoder are not updated read modbus coils & registers
  else {
    //update modbus slave
    slave.update();
    //check for exception on slave modbus
    if (slave.hasException()) {
      if (!debug) Serial.println("MODBUS exception: " + String(slave.getExceptionMessage()));
      slave.clearException();
    }
    //read coil 0: Encoder Mode (Belt vs Steps)
    static bool last_coil0 = false;
    if(coils[0] != last_coil0){
      last_coil0 = coils[0];
      if (!debug) Serial.println("Modbus Encoder Mode: " + String(coils[0] ? "Steps" : "Belt"));
    }
    //read coil 1: Lift PID Enable (Automatic vs Manual)
    static bool last_coil1 = false;
    if(coils[1] != last_coil1){
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
    //read register 1: Lift Setpoint Angle (0.01 deg)
    static uint16_t last_reg_setpoint = 0;
    if(registers[1] != last_reg_setpoint){
      last_reg_setpoint = registers[1];
      float setpoint_deg = (float)registers[1] / 100.0f;
      liftSetpoint = lift.computeHeight(setpoint_deg);
      if (!debug) Serial.println("Lift Setpoint updated via Modbus: " + String(setpoint_deg) + " deg (" + String(liftSetpoint) + " mm)");
    }
  }
 
  //change parameters with buttons
  if(M5.Btn.wasPressed()){
    debug = !debug;
    if(debug){
        M5.dis.drawpix(0, CRGB::Orange);
        Serial.println("Debug mode enabled");
        for(int i = 0; i < max_channel; i++){
          i2cMultiplexer.selectChannel(i);
          encoders[i].resetEncoder();
        }
    }
    else{
        M5.dis.drawpix(0, CRGB::Green);
    }
  }

  //read command on serial port to control lift: up, down, stop, auto, manual, reset, set speed, setpoint, Kp, Ki, Kd
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.println("command:" + command);
    if (command == "up") {
      liftPID.SetMode(QuickPID::Control::manual);
      lift.moveUp();
    } 
    else if (command == "down") {
      liftPID.SetMode(QuickPID::Control::manual);
      lift.moveDown();
    } 
    else if (command == "stop") {
      liftPID.SetMode(QuickPID::Control::manual);
      lift.stop();
    } 
    else if (command == "auto") {
      liftPID.Reset();
      liftPID.SetMode(QuickPID::Control::automatic);
      Serial.println("PID Auto mode enabled");
    } 
    else if (command == "manual") {
      liftPID.SetMode(QuickPID::Control::manual);
      lift.stop();
      Serial.println("PID Manual mode enabled");
    } 
    else if (command == "reset") {
      liftPID.Reset();
    } 
    else if (command.startsWith("speed")) {
      float spd = getValue(command);
      liftPID.SetMode(QuickPID::Control::manual);
      Serial.println("speed:" + String(spd));
      lift.setSpeed(spd);
    } 
    else if (command.startsWith("liftSP")) {
      liftSetpoint = getValue(command);
      Serial.println("liftSetPoint:" + String(liftSetpoint));
    } 
    else if (command.startsWith("step")) {
      float stepVal = getValue(command);
      liftSetpoint += stepVal;
      Serial.println("liftSetPoint:" + String(liftSetpoint));
    } 
    else if (command.startsWith("Kp")) {
      liftKp = getValue(command);
      liftPID.SetTunings(liftKp, liftKi, liftKd);
      Serial.println("Kp:" + String(liftKp));
    } 
    else if (command.startsWith("Ki")) {
      liftKi = getValue(command);
      liftPID.SetTunings(liftKp, liftKi, liftKd);
      Serial.println("Ki:" + String(liftKi));
    } 
    else if (command.startsWith("Kd")) {
      liftKd = getValue(command);
      liftPID.SetTunings(liftKp, liftKi, liftKd);
      Serial.println("Kd:" + String(liftKd));
    } 
    else if (command.startsWith("getK")) {
      Serial.println("Kp:" + String(liftPID.GetKp()));
      Serial.println("Ki:" + String(liftPID.GetKi()));
      Serial.println("Kd:" + String(liftPID.GetKd()));
    } 
    else if (command.startsWith("setAngle")) {
      float angle = getValue(command);
      liftSetpoint = lift.computeHeight(angle);
      Serial.println("setAngle:" + String(angle));
      Serial.println("liftSetPoint:" + String(liftSetpoint));
    }
  }
}


float compute_encoder_speed_2(uint32_t encoder_count,uint32_t time_us) {
  //define last values 
  static uint32_t last_time_us = 0;
  static uint32_t last_encoder_count = 0;
  //compute deltas
  //compute deltas count and time
  int32_t   delta_c = encoder_count - last_encoder_count;
  uint32_t  delta_t = time_us - last_time_us;
  //Serial.println("delta_c: " + String(delta_c) + " delta_t: " + String(delta_t));
  //compute speed
  float encoder_speed = 1e6 * delta_c / delta_t; //impulsions per second
  //Serial.println("encoder_speed: " + String(encoder_speed));
  //update last values
  last_encoder_count = encoder_count;
  last_time_us = time_us;
  //retunr speed pulses per second
  return encoder_speed;
}

float compute_encoder_speed(int32_t delta_count,uint32_t delta_time) {
  //compute speed
  float encoder_speed = 1e6 * delta_count / delta_time; //impulsions per second
  if(debug){
    //Serial.println("delta_pulses: " + String(delta_count) + " delta_time: " + String(delta_time));
    //Serial.println("encoder_speed_pulse/s: " + String(encoder_speed));
  }
  //return speed pulses per second
  return fabs(encoder_speed);
}

