#pragma once

#include <Arduino.h>
#include <DFRobot_GP8XXX.h>
#include <RunningMedian.h>

// Constant physical and electrical parameters (Calibration Baumer GIM500R via ADS1110)
// Regression R² = 0.999995 | RMSE = 0.066 deg | Max Error = 0.098 deg (lift-calibration.txt)
#define ANALOG_TO_ANGLE_GAIN         19.057528f   // deg/V
#define ANALOG_TO_ANGLE_OFFSET       -2.793855f   // deg (Nominal calibration R²=0.999995)
#define ANALOG_TO_ANGLE_GAIN_I2C     1.0f
#define ANALOG_TO_ANGLE_OFFSET_I2C   0.0f
#define BELT_LENGTH_MM               2630.0f
#define BELT_HEIGHT_MM               185.0f
#define SPEED_TO_ANALOG_GAIN         1.03f
#define SPEED_TO_ANALOG_OFFSET       0.0f
#define ADC_I2C_ADDRESS              0x48
#define THRESHOLD_ANGLE_DEG          78.0f
#define SLOPE_MM_PER_DEG             6.3853f

#ifndef ADC_ADDR
#define ADC_ADDR 0x48
#endif

#define ADC_CONFIG_240SPS_12BIT      0x80 // 12-bit (1 LSB = 1000 uV -> ~0.116 deg)
#define ADC_CONFIG_60SPS_14BIT       0x84 // 14-bit (1 LSB = 250 uV  -> ~0.029 deg)
#define ADC_CONFIG_30SPS_15BIT       0x88 // 15-bit (1 LSB = 125 uV  -> ~0.0145 deg)
#define ADC_CONFIG_15SPS_16BIT       0x8C // 16-bit (1 LSB = 62.5 uV -> ~0.0072 deg)

#define ADC_MIN_CODE_12BIT           2048.0f
#define ADC_MIN_CODE_14BIT           8192.0f
#define ADC_MIN_CODE_15BIT           16384.0f
#define ADC_MIN_CODE_16BIT           32768.0f

// Default ADS1110 configuration: 14-bit @ 60 SPS for maximum control bandwidth with ~0.029 deg resolution
#define ADC_CONFIG_DEFAULT           ADC_CONFIG_60SPS_14BIT
#define ADC_MIN_CODE_DEFAULT         ADC_MIN_CODE_14BIT

// Backward compatibility
#define ADC_CONFIG_240FPS_PGA1       ADC_CONFIG_240SPS_12BIT

#ifndef DAC_ADDR
#define DAC_ADDR 0x5A // GP8413 0-5V DAC address (configured with A0A1A2=010 -> 0x5A)
#endif

// Detailed diagnostic status codes
enum LiftStatus {
    LIFT_OK = 0,
    LIFT_ERR_DAC_NOT_FOUND = 1,       // GP8413 0-5V DAC (0x5A) not responding
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
    int init(DFRobot_GP8XXX::eOutPutRange_t range = DFRobot_GP8XXX::eOutputRange5V,
             uint8_t adc_config = ADC_CONFIG_DEFAULT,
             float adc_min_code = ADC_MIN_CODE_DEFAULT);
    int checkHardware(bool &dac_ok, bool &adc_ok);
    static const char* getStatusMessage(int code);
    void update();
    float getInclinaison_deg();
    float getVoltage(bool raw = false);
    float getSensorValue(bool raw = false);
    float getHeight_mm();
    void setSpeed(float speed_pct);
    void moveUp();
    void moveDown();
    void stop();
    void setMinOutputPct(float min_pct) { minOutputPct = min_pct; }
    float getMinOutputPct() const { return minOutputPct; }
    float computeHeight(float angle_deg);
    float readADC();

private:
    float sensorValue;
    float sensorValueRaw;
    float minOutputPct;
    float adcMinCode;
    uint8_t adcConfig;
    char upPin;
    char downPin;
    uint8_t dacAddress;
    DFRobot_GP8413 GP8413;
    RunningMedian adcMedian;
};
