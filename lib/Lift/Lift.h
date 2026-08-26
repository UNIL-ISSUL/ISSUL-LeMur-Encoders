#pragma once

#include <Arduino.h>
#include <DFRobot_GP8XXX.h>

//Constant physical and electrical parameters
#define ANALOG_TO_ANGLE_GAIN 1.071819024
#define ANALOG_TO_ANGLE_OFFSET -2.772616596
#define ANALOG_TO_ANGLE_GAIN_I2C 1
#define ANALOG_TO_ANGLE_OFFSET_I2C 0
#define BELT_LENGTH_MM 2630
#define BELT_HEIGHT_MM 185
#define SPEED_TO_ANALOG_GAIN 1.03
#define SPEED_TO_ANALOG_OFFSET 0
#define PCT_TRESHOLD 12
#define ADC_I2C_ADDRESS 0x48
#define THRESHOLD_ANGLE_DEG 78
#define SLOPE_MM_PER_DEG 6.3853

#ifndef ADC_ADDR
#define ADC_ADDR 0x48
#endif

#define ADC_CONFIG_240FPS_PGA1 0x80 // 0b10000000 : 240 SPS continuous mode, PGA = 1

#ifndef DAC_ADDR
#define DAC_ADDR 0x58 //DAC address GP8413
#endif

//define a class to manage the lift of the custom project le mur
class Lift {
    float inclinaison_deg;
    float height_mm;
    public :
        Lift(char upPin, char downPin, uint8_t dacAddr = DAC_ADDR);
        ~Lift();    
        int init(DFRobot_GP8XXX::eOutPutRange_t range = DFRobot_GP8XXX::eOutputRange10V);
        void update();
        float getInclinaison_deg();
        float getSensorValue(bool raw = true);
        float getHeight_mm();
        void setSpeed(float speed_pct);
        void moveUp();
        void moveDown();
        void stop();
        void move(float pid_output);
        float computeHeight(float angle_deg);
    private :
        //dac output from inclinometer
        float sensorValue;
        int sensorValueRaw;
        //pins
        char upPin;
        char downPin;
        //DAC model
        DFRobot_GP8413 GP8413;
        //Function to convert angle to height
        float horizontalPosition(float angle_deg);
        float readADC();
};