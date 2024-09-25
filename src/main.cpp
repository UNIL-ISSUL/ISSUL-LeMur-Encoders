#include <Arduino.h>
#include <M5Atom.h>
#include <ModbusRTUMaster.h>
#include <Wire.h>
#include "UNIT_EXT_ENCODER.h"
#include "TCA9548.h"
#include <RunningMedian.h>
#include "ewma.h"

//bluetooth
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

//bluetooth server
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
#include <BLE2902.h>  //client configuration descriptor
//#include <BLE2901.h>  //client characteristic configuration descriptor

#define SERVICE_UUID "1826" //fitness machine service
#define CHARACTERISTIC_UUID "2ACD" //treadmill data characteristic

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

//i2c wire communication
#define I2C_FREQ 400000UL

//i2c ext-encoder communication
UNIT_EXT_ENCODER encoder;
RunningMedian encoder_count_median = RunningMedian(5);
Ewma encoder_count_ewma = Ewma(0.1);
Ewma encoder_speed_ewma = Ewma(0.5);
float compute_encoder_speed();
float compute_encoder_speed_2(float encoder_count,uint32_t time_us);
float compute_encoder_speed_3(uint32_t encoder_count);
//define timer to measure speed
//https://deepbluembedded.com/esp32-timers-timer-interrupt-tutorial-arduino-ide/?utm_content=cmp-true
hw_timer_t *timer = NULL;

//i2c multiplexer communication
TCA9548 i2cMultiplexer(0x70);
#define ADC_ADDR 0x48

uint16_t readADC(uint8_t channel, TCA9548 *multiplexer) {
    multiplexer->selectChannel(channel);
    Wire.requestFrom(ADC_ADDR, (uint8_t)2);
    return (int16_t)((Wire.read() << 8) | Wire.read());
}

uint32_t readEncoder() {
    Wire.beginTransmission(0x59);
    Wire.write(0x00); // register 0
    Wire.endTransmission(true);
    Wire.requestFrom(UNIT_EXT_ENCODER_ADDR, (uint8_t)4);
    return (int32_t)((Wire.read() << 24) | (Wire.read() << 16) | (Wire.read() << 8) | Wire.read());
} 

void setup() {
  //initialize the M5Atom
  M5.begin(true, true, true); // enable serial, enable I2C, enable display (led)
  Serial.begin(115200);
  Serial.println("M5Atom initialized");

  //initialize the BLE
  BLEDevice::init("Le Mur");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID,BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  //start service
  pService->start();
  //start advertising
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE service started, waiting clien to notify");


  pCharacteristic->addDescriptor(new BLE2902());

  //initialize the timer
  //timer = timerBegin(2, 1, true); //timer 0, prescaler 80, count up
  //timerStart(timer);

  //initialize the i2c communication
  //Wire.setPins(25U, 21U); //default pins (21, 22)
  Wire.setClock(I2C_FREQ);
  //Wire.setTimeOut(1000); //1ms timeout
  Wire.begin(25U, 21U);


  //initialize the encoder
  if(encoder.begin(&Wire, UNIT_EXT_ENCODER_ADDR, 25U, 21U, I2C_FREQ)){
    Serial.println("Encoder initialized");
    encoder.setPulse(360);
    encoder.resetEncoder();
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

float encoder_count;
float encoder_count_filtered;
float encoder_speed;
float encoder_speed_filtered;

void loop() {
  //M5 update for button
  M5.update();

  //read encoder value
  encoder_count_median.add(encoder.getEncoderValue());
  static float encoder_cont_last_value = 0;
  encoder_count = encoder_count_median.getMedian();
  encoder_count_filtered = encoder_count_ewma.filter(encoder_count);

  //compute encoder speed
  encoder_speed = compute_encoder_speed_2(encoder_count_filtered, micros());

  //check if client is connected
  if (deviceConnected) {
    //send data to client
    //compute encoder speed
    pCharacteristic->setValue((uint8_t*)&encoder_speed, 4); //4byte float
    pCharacteristic->notify();
    delay(5); // bluetooth stack will go into congestion, if too many packets are sent
  }
  //disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  //connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
  
  //change parameters with buttons
  if(M5.Btn.wasPressed()){
    debug = !debug;
    if(debug){
        M5.dis.drawpix(0, CRGB::Orange);
        Serial.println("Debug mode enabled");
        encoder.resetEncoder();
    }
    else{
        M5.dis.drawpix(0, CRGB::Green);
    }
  }

  /*float delta_count = encoder_count - encoder_cont_last_value;
  encoder_cont_last_value = encoder_count;

  //compute encoder speed every 3ms
  static uint32_t last_time = 0;
  uint32_t time = micros();
  uint32_t delta_time = time - last_time;
  if(time - last_time >= 3000) {
    encoder_speed = compute_encoder_speed_2(delta_count,delta_time);
    last_time = time;
  }*/

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


float compute_encoder_speed_2(float encoder_count,uint32_t time_us) {
  //define last values 
  static uint32_t last_time_us = 0;
  static long last_encoder_count = 0;
  //compute deltas
  //compute deltas count and time
  int delta_c = encoder_count - last_encoder_count;
  uint32_t delta_t = time_us - last_time_us;
  Serial.println("delta_c: " + String(delta_c) + " delta_t: " + String(delta_t));
  //compute speed
  float encoder_speed = 1e6 * delta_c / delta_t; //impulsions per second
  Serial.println("encoder_speed: " + String(encoder_speed));
  //update last values
  last_encoder_count = encoder_count;
  last_time_us = time_us;
  //retunr speed pulses per second
  return encoder_speed;
}

float compute_encoder_speed() {
  //define last values 
  static uint32_t last_time_us = 0;
  static long last_encoder_count = 0;
  //compute deltas
  //read encoder value
  uint32_t encoder_count = encoder.getEncoderValue();
  uint32_t time_us = micros();
  //compute deltas count and time
  int delta_c = encoder_count - last_encoder_count;
  uint32_t delta_t = time_us - last_time_us;
  Serial.println("delta_c: " + String(delta_c) + " delta_t: " + String(delta_t));
  //compute speed
  float encoder_speed = 1e6 * delta_c / delta_t; //impulsions per second
  Serial.println("encoder_speed: " + String(encoder_speed));
  //update last values
  last_encoder_count = encoder_count;
  last_time_us = time_us;
  //retunr speed pulses per second
  return encoder_speed;
}