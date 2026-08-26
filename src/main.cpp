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
//i2c wire communication
#define I2C_FREQ 400000UL
#define SDA 25
#define SCL 21

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
RunningMedian inclinaison_median = RunningMedian(10);

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

//modbus communication
unsigned long baudrate = 19200UL;
#define RX 33
#define TX 23
ModbusRTUSlave slave(Serial2,3); //modbus slave on serial2, addr to be checked
#define NUM_COILS 2 //number of coils
#define NUM_REGISTERS 1 //number of holding registers
bool coils[NUM_COILS] = {false,false};  //first coil is for steps status and second if for encoder_feedback status
uint16_t registers[NUM_REGISTERS] = {0}; //belt ot steps encoder speed in mm/s

float readADC() {
  int length = 2;
  Wire.requestFrom(ADC_ADDR, length);
  int16_t raw = (Wire.read() << 8) | Wire.read() ;
  int16_t min_code = 8192; //14bit 60fps
  float deltaV = raw * 2.048 / min_code; //conversion ADS1110 in datasheet
  return deltaV * 6.1; //m5stack ADCv1.1 has a tension diviser made by 510k and 100k resitor (510+100) / 100
}

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
  //associate coils
  slave.setCoils(coils, NUM_COILS);
  slave.setHoldingRegisters(registers,1); //set holding register to store current feedback speed
  
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
      //send configuration to encoder
      encoders[i].begin(&Wire, ENC_ADDR, SDA, SCL, I2C_FREQ);
      //encoders[i].setPulse(pulse[i]);
      //encoders[i].setPerimeter(perimeter[i]);
      //encoders[i].resetEncoder();
      Serial.println("Encoder initialized");
    }
    else{
      Serial.println("Encoder device not found on channel " + String(i));
      i2c_error += 1;
      //decrease max channel
      //max_channel -= 1;
    }
  }

  //initialize ADC device

  //test if ADC is connected
  Wire.beginTransmission(ADC_ADDR);
  if (Wire.endTransmission()== 0) {
    Serial.println("ADC device found");
    //send configuration to ADC
    const uint8_t config = 0x84; // 1000 0100 see ADC1110 datasheet : 60fps continuous mode
    Wire.beginTransmission(ADC_ADDR);
    Wire.write(0x84); // config register
    Wire.endTransmission();
    Serial.println("ADC initialized");
    }
  else {
    Serial.println("ADC device not found");
    i2c_error += 1;
  }

  //initialize DAC devices
  i2cMultiplexer.selectChannel(2);
  if(i2cMultiplexer.isConnected(DAC_ADDR)){
    Serial.println("DAC device found on channel 2");
  }
  else{
    Serial.println("DAC device not found on channel 2");
    i2c_error += 1;
  }

  i2cMultiplexer.selectChannel(3);
  if(i2cMultiplexer.isConnected(DAC_ADDR)){
    Serial.println("DAC device found on channel 3");
  }
  else{
    Serial.println("DAC device not found on channel 3");
    i2c_error += 1;
  }

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
    //read adc value
    inclinaison_median.add(readADC());
    uint32_t inclinaison= round(inclinaison_median.getMedian()*100);  

    //apply geometry to compute speed in mm/s
    belt_encoder_speed = belt_encoder_speed*perimeter[0]/pulse[0]; //mm/s
    steps_encoder_speed = steps_encoder_speed*perimeter[1]/pulse[1]; //mm/s
    //filter speeds
    float filtered_belt_speed = belt_speed_filter.filter(belt_encoder_speed);
    float filtered_steps_speed = encoder_speed_filter.filter(steps_encoder_speed);
    //update DAC output
    uint16_t dac_value;
    //if coil 0 is set to 1, the steps are used else this the belt
    //send raw data to dac 
    if(!coils[0]) dac_value = (int16_t)round(belt_encoder_speed);  
    else dac_value = (int16_t)round(steps_encoder_speed);
    //write DAC output
    // calculate current for speed DAC
    float speed_mA;
    const uint16_t max_belt_speed_mm_s = 12000; // 40km/h
    const uint16_t max_steps_speed_mm_s = 3000; // 1m/s
    if (!coils[0]) speed_mA = 4.0 + (16.0 * belt_encoder_speed / max_belt_speed_mm_s);
    else speed_mA = 4.0 + (16.0 * steps_encoder_speed / max_steps_speed_mm_s);
    //Serial.println("Speed mA: " + String(speed_mA));
    if (speed_mA > 20.0) speed_mA = 20.0;
    if (speed_mA < 4.0) speed_mA = 4.0;

    // calculate current for inclinaison DAC
    float incl_deg = (float)inclinaison * 90 / 1000000.0;
    float incl_mA = 4.0 + (incl_deg * 16.0 / 90.0);
    if (incl_mA > 20.0) incl_mA = 20.0;
    if (incl_mA < 4.0) incl_mA = 4.0;

    //speed_mA = 20;
    //incl_mA = 12;
    set_DFR0972_mA(2, speed_mA, 654, 3279);
    set_DFR0972_mA(3, incl_mA, 654, 3279);
    //print mA sent value
    //Serial.println("Speed mA: " + String(speed_mA) + " Inclinaison mA: " + String(incl_mA));

    //Check if dac_value is not null to enable encoder feedback
    if(dac_value > 0) coils[1] = true; //set encoder feedback coil to true
    else              coils[1] = false; //set encoder feedback coil to false
    //update modbus holding register with current feedback speed
    registers[0] = dac_value; //update holding register with current feedback speed

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
    }
  }
  //if encoder are not updated read modbus coils
  else {
    //update modbus slave
    slave.update();
    //Serial.println("Coil 0: " + String(coils[0]));
    //check for exception on slave modbus
    if (slave.hasException()) {
      if (!debug) Serial.println("MODBUS exception: " + String(slave.getExceptionMessage()));
      slave.clearException();
    }
    //read coils
    static bool last_coil = false;
    if(coils[0] != last_coil){
      last_coil = coils[0];
      if (!debug) Serial.println("Coil 0 changed to " + String(coils[0]));
    }
    //show change of encoder feedback state
    static bool last_coil1 = false;
    if(coils[1] != last_coil1){
      last_coil1 = coils[1];
      if (!debug) {
        if(coils[1]) Serial.println("Encoder feedback enabled");
        else         Serial.println("Encoder feedback disabled");
      }
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

  /*//Send Modbus request
  static uint32_t last_request = 0;
  if (master.isIdle()) {
    const float  max_pulse_sec = 2900.0 / 60 * 1504; //2900 tr/min at 100Hz with 1504 pulse per rotation
    const uint16_t max_voltage = 10000; //10V
    //compute encoder speed
    encoder_speed = compute_encoder_speed_2(encoder_count_filtered, micros());
    //filter encoder speed
    encoder_speed_filtered = encoder_speed_ewma.filter(encoder_speed);
    //scale speed to 0-10V
    outputRegistersVicon[0] = uint16_t(round(max_voltage * encoder_speed / max_pulse_sec));
    //scale adc on channel 0 to 0-10'000 mV  
    outputRegistersVicon[1] = uint16_t(1.0 * readADC(0, &i2cMultiplexer) * 10000 / pow(2, 14));
    //scale adc on channel 1 to 0-10'000 mV
    outputRegistersVicon[2] = uint16_t(1.0 * readADC(1, &i2cMultiplexer) * 10000 / pow(2, 14));

    //sinus test if test enabled on last modbus register
    if(true){
      const float f = 10; //Hz
      const float omega = 2 * PI * f;
      outputRegistersVicon[2] = sin(omega * micros() * 1e-6) * 1000;
    }
    //measure time between two modbus request
    if(debug) {
      Serial.println(">encoder_count:"+String(encoder_count)+"|np");
      Serial.println(">encoder_count_filtered:"+String(encoder_count_filtered)+"|np");
      //Serial.println(">encoder_pulse:"+String(encoder.getPulse())+"|np");
      //Serial.println(">encoder_count:"+String(readEncoder())+"|np");
      Serial.println(">encoder_speed:"+String(encoder_speed)+"|np");
      Serial.println(">encoder_speed_filtered:"+String(encoder_speed_filtered)+"|np");
      Serial.println(">adc0:"+String(readADC(0, &i2cMultiplexer))+"|np");
      Serial.println(">adc1:"+String(readADC(1, &i2cMultiplexer))+"|np");
    }

    //print last modbus request time every second
    if(millis() % 100 == 0){
      Serial.println("> modbus request delai [ms] : " + String((micros() - last_request)*1e-3) + "|np"); //time between two modbus request
    }
   
    //send write request
    if (!master.writeMultipleRegisters(VICON_ADDRESS, 0x0004, outputRegistersVicon, REGISTERS_TO_WRITE_VICON)) {
      Serial.println("Error writing to VICON");
      M5.dis.drawpix(0, CRGB::Red);
    }

    last_request = micros();
  }

  //add 1ms delay to avoid flooding the modbus
  //delay(1);

   //check for modbus response
  if (master.isWaitingResponse() ) {
    ModbusResponse response = master.available();
    if (response) {
      //Serial.println("Answer received from slave "+String(response.getSlave())+" : "+String(response.getFC()));
      if (response.hasError()) {
        // There is an error. You can get the error code with response.getErrorCode()
        Serial.println("Modbus error: " + String(response.getErrorCode()));
        M5.dis.drawpix(0, CRGB::Red);
      }
    }
  }

  //check for exception on master modbus
  static uint32_t last_exception = 0;
  if (master.hasException()) {
    Serial.println("MODBUS exception: " + String(master.getExceptionMessage()));
    M5.dis.drawpix(0, CRGB::Red);
    master.clearException();
    last_exception = millis();
  }
  //switch back led after 10s without exception
  if (millis() - last_exception > 1000) {
    if(!debug) M5.dis.drawpix(0, CRGB::Green);
    else M5.dis.drawpix(0, CRGB::Orange);
  }*/
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

