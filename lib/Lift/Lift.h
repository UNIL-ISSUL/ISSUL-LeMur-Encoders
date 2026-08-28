#pragma once

#include <Arduino.h>
#include <DFRobot_GP8XXX.h>

// Constant physical and electrical parameters (Calibration Baumer GIM500R via ADS1110)
// Regression R² = 0.999995 | RMSE = 0.066 deg | Max Error = 0.098 deg (lift-calibration.txt)
#define ANALOG_TO_ANGLE_GAIN         19.057528f   // deg/V
#define ANALOG_TO_ANGLE_OFFSET_STOP  -2.793855f   // deg (Belt stopped, nominal calibration)
#define ANALOG_TO_ANGLE_OFFSET_RUN   -2.603280f   // deg (Belt running, +0.191 deg compensation for 10mV drop)
#define ANALOG_TO_ANGLE_OFFSET       ANALOG_TO_ANGLE_OFFSET_STOP
#define ANALOG_TO_ANGLE_GAIN_I2C     1.0f
#define ANALOG_TO_ANGLE_OFFSET_I2C   0.0f
#define BELT_LENGTH_MM               2630.0f
#define BELT_HEIGHT_MM               185.0f
#define SPEED_TO_ANALOG_GAIN         1.03f
#define SPEED_TO_ANALOG_OFFSET       0.0f
#define PCT_TRESHOLD                 12.0f
#define ADC_I2C_ADDRESS              0x48
#define THRESHOLD_ANGLE_DEG          78.0f
#define SLOPE_MM_PER_DEG             6.3853f

#ifndef ADC_ADDR
#define ADC_ADDR 0x48
#endif

#define ADC_CONFIG_240FPS_PGA1 0x80 // 0b10000000 : 240 SPS continuous mode, PGA = 1

#ifndef DAC_ADDR
#define DAC_ADDR 0x59 // GP8413 0-10V DAC address (default: 0x59)
#endif

// Detailed diagnostic status codes
enum LiftStatus {
    LIFT_OK = 0,
    LIFT_ERR_DAC_NOT_FOUND = 1,       // GP8413 0-5V DAC (0x59) not responding
    LIFT_ERR_ADC_NOT_FOUND = 2,       // ADS1110 ADC (0x48) not responding
    LIFT_ERR_ADC_CONFIG_FAIL = 3,     // ADS1110 config write failed
    LIFT_ERR_ADC_READ_FAIL = 4        // ADS1110 reading returned invalid data
};

class Lift {
    float inclinaison_deg;
    float height_mm;
public:
    Lift(char upPin, char downPin);
    ~Lift();
    int init(DFRobot_GP8XXX::eOutPutRange_t range = DFRobot_GP8XXX::eOutputRange5V);
    int checkHardware(bool &dac_ok, bool &adc_ok);
    static const char* getStatusMessage(int code);
    void update(bool belt_running = false);
    float getInclinaison_deg();
    float getVoltage(bool raw = false);
    float getSensorValue(bool raw = false);
    float getHeight_mm();
    void setSpeed(float speed_pct);
    void moveUp();
    void moveDown();
    void stop();
    void move(float pid_output);
    float computeHeight(float angle_deg);
    float readADC();

private:
    float sensorValue;
    float sensorValueRaw;
    char upPin;
    char downPin;
    uint8_t dacAddress;
    DFRobot_GP8413 GP8413;
    float horizontalPosition(float angle_deg);
};
