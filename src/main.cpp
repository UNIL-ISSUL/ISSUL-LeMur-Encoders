#include <Arduino.h>
#include <M5Atom.h>
#include <ArduinoModbus.h>
#include <ArduinoRS485.h>
#include "ATOM_DTU_NB.h"
#include <Wire.h>
#include "UNIT_EXT_ENCODER.h"
#include "TCA9548.h"

//modbus communication
RS485Class RS485(Serial2, ATOM_DTU_RS485_RX, ATOM_DTU_RS485_TX, -1, -1);
#define VICON_ADDRESS 4    //modbus address of the Vicon
//#define baudrate 57600UL
#define baudrate 9600UL

//register to write on Vicon
#define REGISTERS_TO_WRITE_VICON 3   //number of register to write : lift angle, belt speed
uint16_t outputRegistersVicon[REGISTERS_TO_WRITE_VICON] = {0, 0, 0}; //belt speed, lift angle, blet torque

//i2c ext-encoder communication
UNIT_EXT_ENCODER encoder;
float compute_encoder_speed();

//i2c multiplexer communication
TCA9548 i2cMultiplexer(0x70);
#define ADC_ADDR 0x48

static uint16_t readADC(uint8_t channel, TCA9548 *multiplexer) {
    multiplexer->selectChannel(channel);
    Wire.requestFrom(ADC_ADDR, (uint8_t)2);
    return (int16_t)((Wire.read() << 8) | Wire.read());
}

void setup() {
  //initialize the M5Atom
  M5.begin(true, true, true); // enable serial, enable I2C, enable display (led)
  Serial.begin(9600);
  Serial.println("M5Atom initialized");

  //initialize the modbus communication
  if (!ModbusRTUClient.begin(baudrate, SERIAL_8E1)) {
    Serial.println("Failed to start Modbus RTU Client!");
    while (1);
  }

  //initialize the i2c communication
  Wire.begin();
  Wire.setClock(400000);

  //initialize the encoder
  if(encoder.begin(&Wire, UNIT_EXT_ENCODER_ADDR, SDA, SCL, 400000UL)){ //400kHz
    Serial.println("Encoder initialized");
  }
  else{
    Serial.println("Encoder initialization failed");
  }

  //initialize the i2c multiplexer
  if(i2cMultiplexer.begin()){
    Serial.println("I2C Multiplexer initialized");
  }
  else{
    Serial.println("I2C Multiplexer initialization failed");
  }
  //search adc device
  for(int i = 0; i < 2; i++){
    i2cMultiplexer.selectChannel(i);
    bool connected = i2cMultiplexer.isConnected(ADC_ADDR);
    if(connected){
      Serial.println("ADC device found on channel " + String(i));
      //send configuration to ADC
      const uint8_t config = 0x84; // 1000 0100 see ADC1110 datasheet : 60fps continuous mode
      Wire.beginTransmission(ADC_ADDR);
      Wire.write(0x84); // config register
      Wire.endTransmission();
    }
    else{
      Serial.println("ADC device not found on channel " + String(i));
    }
  }
  M5.dis.drawpix(0, CRGB::Green);
}

bool debug = false;

void loop() {
  //M5 update for button
  M5.update();
  //change parameters with buttons
  if(M5.Btn.wasPressed()){
    debug = !debug;
    if(debug){
        M5.dis.drawpix(0, CRGB::Orange);
        Serial.println("Debug mode enabled");
    }
    else{
        M5.dis.drawpix(0, CRGB::Green);
    }
  }
  static uint32_t last_request = 0;
  if (!ModbusRTUClient.holdingRegisterWrite(VICON_ADDRESS, 0x0005, 0x0000)) {
        Serial.print("Failed to write coil! ");
        Serial.println(ModbusRTUClient.lastError());
  }
  if(debug) {
    Serial.println("time between modbus request : " + String(millis() - last_request) + " ms"); //time between two modbus request
  }
  delay(2);
  last_request = millis();
  /*
  //Send Modbus request
  static uint32_t last_request = 0;
  if (master.isIdle() && ((millis() - last_request) > 1000)) {
    const float  max_pulse_sec = 2900.0 / 60 * 1504; //2900 tr/min at 100Hz with 1504 pulse per rotation
    const uint16_t max_voltage = 10000; //10V
    //scale speed to 0-10V
    //outputRegistersVicon[0] = uint16_t(round(max_voltage * compute_encoder_speed() / max_pulse_sec));
    //scale adc on channel 0 to 0-10'000 mV  
    //outputRegistersVicon[1] = uint16_t(1.0 * readADC(0, &i2cMultiplexer) * 10000 / pow(2, 14));
    //scale adc on channel 1 to 0-10'000 mV
    //outputRegistersVicon[2] = uint16_t(1.0 * readADC(1, &i2cMultiplexer) * 10000 / pow(2, 14));

    //sinus test if test enabled on last modbus register
    if(debug){
      const int f = 10; //Hz
      const float omega = 2 * PI * f;
      outputRegistersVicon[2] = sin(omega * micros() * 1e-6) * 1000;
    }
    //measure time between two modbus request
    if(debug) {
      Serial.println("time between modbus request : " + String(millis() - last_request) + " ms"); //time between two modbus request
    }
   
    //send write request
    if (!master.writeMultipleRegisters(VICON_ADDRESS, 0x0004, outputRegistersVicon, REGISTERS_TO_WRITE_VICON)) {
      Serial.println("Error writing to VICON");
      M5.dis.drawpix(0, CRGB::Red);
    }

    last_request = millis();
  }
  
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
  if (master.hasException()) {
    Serial.println("MODBUS exception: " + String(master.getExceptionMessage()));
    M5.dis.drawpix(0, CRGB::Red);
    master.clearException();
  }
  */
}

float compute_encoder_speed() {
  //define last values 
  static uint32_t last_time_us = 0;
  static long last_encoder_count = 0;
  //compute deltas
  //read encoder value
  float encoder_count = encoder.getEncoderValue();
  uint32_t time_us = micros();
  //compute deltas count and time
  uint32_t delta_c = encoder_count - last_encoder_count;
  uint32_t delta_t = time_us - last_time_us;
  //compute speed
  float encoder_speed = delta_c * 1e6 / delta_t; //impulsions per second
  //update last values
  last_encoder_count = encoder_count;
  last_time_us = time_us;
  //retunr speed pulses per second
  return encoder_speed;
}