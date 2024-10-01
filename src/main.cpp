#include <Arduino.h>
#include <M5Atom.h>
#include <Wire.h>
#include "UNIT_EXT_ENCODER.h"
#include "TCA9548.h"

#include <BluetoothSerial.h>
//check if bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
//check if bluetooth spp is available
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

//create bluetooth object
BluetoothSerial SerialBT;
String myName = "LeMur streaming";
//const char* pin="1234";
//bool deviceConnected = false;

//define Union to store transmitted information over bluetooth
union {
  byte bytes[7];
  struct {
    uint16_t belt_encoder_speed ;
    uint16_t steps_encoder_speed ;
    uint16_t inclinaison ;
    byte stopbyte = 0x0A; //0x0A is line feed, //0x0D is carriage return
  };
} packet;

//i2c wire communication
#define I2C_FREQ 400000UL
#define SDA 25
#define SCL 21

//i2c ext-encoder communication
UNIT_EXT_ENCODER encoders[2]; //two encoders 0: belt, 1: steps
  //encoder configuration
  uint32_t pulse[2] = {8000,16000};
  uint32_t perimeter[2] = {80,123};
//speed computation function
float compute_encoder_speed();
float compute_encoder_speed_2(float encoder_count,uint32_t time_us);

//i2c multiplexer communication
//two encoders on the i2c multiplexer at address 0x70
//one ADC directly connected to the i2c bus
TCA9548 i2cMultiplexer(0x70);
byte max_channel = 2;
#define ADC_ADDR 0x48
#define ENC_ADDR 0x59

uint16_t readADC(uint8_t channel, TCA9548 *multiplexer) {
  multiplexer->selectChannel(channel);
  Wire.requestFrom(ADC_ADDR, (uint8_t)2);
  return (int16_t)((Wire.read() << 8) | Wire.read());
}
uint16_t readADC() {
  Wire.requestFrom(ADC_ADDR, (uint8_t)2);
  return (int16_t)((Wire.read() << 8) | Wire.read());
}

uint32_t readEncoder(bool mm=false) {
  int register_address = 0x00; // pulse value
  if(mm) register_address = 0x10; // meter value
  Wire.beginTransmission(ENC_ADDR);
  Wire.write(register_address); // register 0
  Wire.endTransmission(true);
  Wire.requestFrom(UNIT_EXT_ENCODER_ADDR, (uint8_t)4);
  return (int32_t)((Wire.read() << 24) | (Wire.read() << 16) | (Wire.read() << 8) | Wire.read());
} 

uint32_t readEncoder(uint8_t channel, TCA9548 *multiplexer,bool mm=false) {
  if (max_channel=0) return 0; //no encoder connected
  if (channel >= max_channel) return 0; //channel not available
  multiplexer->selectChannel(channel);
  if(mm)  return encoders[channel].getMeterValue();
  else    return encoders[channel].getEncoderValue();
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
  M5.begin(true, true, true); // enable serial, enable I2C, enable display (led)
  Serial.println("M5Atom initialized");
  
  //Bluetooth classic as slave
  SerialBT.begin(myName, false);
  Serial.println("The device \"%s\" started in slave mode");

  //I2C communication
  //Wire.begin(SDA, SCL);
  Wire.setClock(I2C_FREQ);  //update clock

  //initialize I2C multiplexer
  if(i2cMultiplexer.begin()){
    Serial.println("I2C Multiplexer initialized");
  }
  else{
    Serial.println("I2C Multiplexer initialization failed");
  }

  //search and configure encoders
  for(int i = 0; i < 2; i++){
    i2cMultiplexer.selectChannel(i);
    bool connected = i2cMultiplexer.isConnected(ENC_ADDR);
    if(connected){
      Serial.println("Encoder device found on channel " + String(i));
      //send configuration to encoder
      if (encoders[i].begin(&Wire, ENC_ADDR, SDA, SCL, I2C_FREQ)) {
        encoders[i].setPulse(pulse[i]);
        encoders[i].setPerimeter(perimeter[i]);
        encoders[i].resetEncoder();
        Serial.println("Encoder initialized");
      }
      else{
        Serial.println("Encoder initialization failed");
        //decrease max channel
        max_channel -= 1;
      }
    }
    else{
      Serial.println("Encoder device not found on channel " + String(i));
    }
  }

  //initialize ADC device

  //test if ADC is connected
  Wire.beginTransmission(ADC_ADDR);
  uint8_t error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("ADC device found");
    //send configuration to ADC
    const uint8_t config = 0x84; // 1000 0100 see ADC1110 datasheet : 60fps continuous mode
    Wire.beginTransmission(ADC_ADDR);
    Wire.write(0x84); // config register
    Wire.endTransmission();
    Serial.println("ADC initialized");
    }
  else Serial.println("ADC device not found");
  
  //initialize timer to interupt @ 5ms
  timer = timerBegin(3, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 5000, true);
  timerAlarmEnable(timer);

  //initialize packet
  packet.belt_encoder_speed = 0;
  packet.steps_encoder_speed = 0;
  packet.inclinaison = 0;
  //initialize display
  M5.dis.drawpix(0, CRGB::Green);
}

bool debug = false;

uint32_t belt_encoder_count = 0;
uint32_t steps_encoder_count = 0;

void loop() {
  
  //M5 update for button
  M5.update();
  
  if(flag_read_encoder){
    //read encoder value
    belt_encoder_count = readEncoder(0, &i2cMultiplexer);
    steps_encoder_count = readEncoder(1, &i2cMultiplexer);
    //compute speed en pluse/sec
    static float belt_encoder_speed = compute_encoder_speed_2(belt_encoder_count, micros());
    static float steps_encoder_speed = compute_encoder_speed_2(steps_encoder_count, micros());
    //read adc value
    static uint16_t inclinaison = readADC();  
    
    //update packet
    packet.belt_encoder_speed = round(100*belt_encoder_speed*perimeter[0]/pulse[0]);    // mm/s x100 rounded to uint16
    packet.steps_encoder_speed = round(100*steps_encoder_speed*perimeter[1]/pulse[1]);  // mm/s x100 rounded to uint16
    //mesure inclinaison
    packet.inclinaison = round(100.0 * inclinaison * 90 / pow(2, 14));                  // 0-90° x100 rounded to uint16
    //packet.belt_encoder_speed = 'a'<<8 | 'a';
    //packet.steps_encoder_speed = 'b'<<8 | 'b';
    //packet.inclinaison = 'c'<<8 | 'c';
    //stream speed value
    SerialBT.write(packet.bytes, 7);
    //print bytes
    /*for (int i = 0; i < 7; i++) {
      Serial.print(char(packet.bytes[i]));
    }*/
    //SerialBT.println(String(encoder_speed));
    flag_read_encoder = false;
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


float compute_encoder_speed_2(float encoder_count,uint32_t time_us) {
  //define last values 
  static uint32_t last_time_us = 0;
  static long last_encoder_count = 0;
  //compute deltas
  //compute deltas count and time
  int delta_c = encoder_count - last_encoder_count;
  uint32_t delta_t = time_us - last_time_us;
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

float compute_encoder_speed(int channel) {
  //define last values 
  static uint32_t last_time_us = 0;
  static long last_encoder_count = 0;
  //compute deltas
  //read encoder value
  uint32_t encoder_count = encoders[channel].getEncoderValue();
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