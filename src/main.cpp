#include <Arduino.h>
#include <M5Atom.h>
#include <Wire.h>
#include "UNIT_EXT_ENCODER.h"
#include "TCA9548.h"
#include <RunningMedian.h>
#include <DFRobot_GP8XXX.h>
#include <ModbusRTUSlave.h>

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
    int16_t belt_encoder_speed ;
    int16_t steps_encoder_speed ;
    int16_t inclinaison ;
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
#define DAC_ADDR 0x58 //DAC address GP8413

//define DAC object
DFRobot_GP8413 GP8413(DAC_ADDR);

//running median filter
RunningMedian belt_encoder_speed_median = RunningMedian(6);
RunningMedian steps_encoder_speed_median = RunningMedian(6);
RunningMedian inclinaison_median = RunningMedian(10); 

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
  //Serial.println("raw adc : "+String(raw));
  return raw*12440.0/8192; //max scale is 12450mV and max output 8192
}

/*uint32_t readEncoder(bool mm=false) {
  int register_address = 0x00; // pulse value
  if(mm) register_address = 0x10; // meter value
  Wire.beginTransmission(ENC_ADDR);
  Wire.write(register_address); // register 0
  Wire.endTransmission(true);
  Wire.requestFrom(UNIT_EXT_ENCODER_ADDR, (uint8_t)4);
  return (int32_t)((Wire.read() << 24) | (Wire.read() << 16) | (Wire.read() << 8) | Wire.read());
}*/

uint32_t readEncoder(uint8_t channel, TCA9548 *multiplexer,bool mm=false) {
  if (channel >= max_channel) {
    Serial.println("No encoder on requested channel : " + String(channel));
    return 0; //channel not available
  }
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

//function to scale encoder speed to 0-10V output
uint16_t scale_encoder_speed(float encoder_speed_mm_s) {
  const uint16_t max_voltage = 10000; //10V
  const float max_speed_mm_s = 40*1e6 / 3600; //40km/h
  float speed_mv = max_voltage * encoder_speed_mm_s / max_speed_mm_s;
  //return ouput scaled on 15bits
  return uint16_t(32767 * speed_mv / max_voltage);
}

void setup() {
  
  //initialize the M5Atom
  M5.begin(true, true, true); // enable serial, enable I2C, enable display (led)
  Serial.begin(115200);
  Serial.println("M5Atom initialized");

  //initialize the modbus communication
  Serial2.begin(baudrate, SERIAL_8E1, RX, TX);
  slave.begin(baudrate);
  //associate coils
  slave.setCoils(coils, NUM_COILS);
  slave.setHoldingRegisters(registers,1); //set holding register to store current feedback speed
  
  //Bluetooth classic as slave
  SerialBT.begin(myName, false);
  //Serial.println(ESP.getEfuseMac());
  Serial.printf("The device \"%s\" started in slave mode \n", myName.c_str());

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

  //initialize DAC device
  if (GP8413.begin() == 0) {
    Serial.println("GP8413 initialized");
  }
  else {
    //if DAC is not found, print error code
    Serial.println("DAC device not found");
    i2c_error += 1;
  }
  //set DAC output range to 0-10V
  GP8413.setDACOutRange(GP8413.eOutputRange10V);
  GP8413.setDACOutVoltage(0,0);//channel 0 output 0

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
    time_us = micros();
    //compute speed en pluse/sec
    belt_encoder_speed_median.add(compute_encoder_speed(belt_encoder_count-last_belt_encoder_count, time_us-last_time_us));
    steps_encoder_speed_median.add(compute_encoder_speed(steps_encoder_count-last_steps_encoder_count, time_us-last_time_us));
    float belt_encoder_speed = belt_encoder_speed_median.getMedian();
    float steps_encoder_speed = steps_encoder_speed_median.getMedian();
    //float belt_encoder_speed = compute_encoder_speed(belt_encoder_count-last_belt_encoder_count, time_us-last_time_us);
    //float steps_encoder_speed = compute_encoder_speed(steps_encoder_count-last_steps_encoder_count, time_us-last_time_us);
    last_belt_encoder_count = belt_encoder_count;
    last_steps_encoder_count = steps_encoder_count;
    last_time_us = time_us;
    //read adc value
    inclinaison_median.add(readADC());
    uint32_t inclinaison= round(inclinaison_median.getMedian()*100);  

    //Serial.println("belt_encoder_speed: " + String(round(100.0*belt_encoder_speed*perimeter[0]/pulse[0])) + " steps_encoder_speed: " + String(steps_encoder_speed));

    //update packet
    packet.belt_encoder_speed   = (int16_t)round(belt_encoder_speed*perimeter[0]/pulse[0]);      // mm/s  rounded to uint16
    packet.steps_encoder_speed  = (int16_t)round(steps_encoder_speed*perimeter[1]/pulse[1]);     // mm/s rounded to uint16
    packet.inclinaison          = (int16_t)round(inclinaison * 90 / 10000);             // 0-00° x100 rounded to uint16 coded on 10mV
    //packet.belt_encoder_speed = 'a'<<8 | 'a';
    //packet.steps_encoder_speed = 'b'<<8 | 'b';
    //packet.inclinaison = 'c'<<8 | 'c';
    //stream speed value
    SerialBT.write(packet.bytes, sizeof(packet.bytes));
    //update DAC output
    uint16_t dac_value;
    //if coil 0 is set to 1, the steps are used else this the belt
    if(!coils[0]) dac_value = packet.belt_encoder_speed;  
    else dac_value = packet.steps_encoder_speed;
    //write DAC output
    GP8413.setDACOutVoltage(scale_encoder_speed(dac_value),0);

    // Consolidate all Teleplot prints into a single string
    String teleplot_str = ">dac_value:" + String(dac_value) + "|np\r\n";
    if (debug) {
      teleplot_str += ">belt_speed:" + String(packet.belt_encoder_speed) + "|np\r\n";
      teleplot_str += ">step_speed:" + String(packet.steps_encoder_speed) + "|np\r\n";
      teleplot_str += ">belt_encoder_count:" + String(belt_encoder_count) + "|np\r\n";
      teleplot_str += ">steps_encoder_count:" + String(steps_encoder_count) + "|np\r\n";
    }
    Serial.print(teleplot_str);

    //Check if dac_value is not null to enable encoder feedback
    if(dac_value > 0) coils[1] = true; //set encoder feedback coil to true
    else              coils[1] = false; //set encoder feedback coil to false
    //update modbus holding register with current feedback speed
    registers[0] = dac_value; //update holding register with current feedback speed

    //reset flag
    flag_read_encoder = false;
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
  return encoder_speed;
}