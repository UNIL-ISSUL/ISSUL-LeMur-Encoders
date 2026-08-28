#include "Lift.h"
#include <Arduino.h>
#include <Ewma.h>
#include <Wire.h>
#include <math.h>

// Exponential Moving Average filters
Ewma ewmaFilterIn(0.01);
Ewma ewmaFilterOut(0.01);

float Lift::readADC() {
  const int length = 2;
  uint8_t received = Wire.requestFrom((int)ADC_ADDR, length);
  if (received < 2) {
    return 0.0f;
  }
  int16_t raw = (int16_t)((Wire.read() << 8) | Wire.read());
  const int16_t min_code = 2048; // 12-bit @ 240fps (PGA=1, full scale 2.048V)
  float deltaV = raw * 2.048f / min_code; // conversion ADS1110 in datasheet (1 LSB = 1mV)
  return deltaV * 6.1f; // m5stack ADCv1.1 voltage divider (510k + 100k) / 100k = 6.1
}

Lift::Lift(char upPin, char downPin) : GP8413(DAC_ADDR) {
    this->upPin = upPin;
    this->downPin = downPin;
    this->dacAddress = DAC_ADDR;
    sensorValue = 0.0f;
    sensorValueRaw = 0;
    inclinaison_deg = 0.0f;
    height_mm = 0.0f;
}

Lift::~Lift() {
    stop();
}

const char* Lift::getStatusMessage(int code) {
    switch (code) {
        case LIFT_OK:                 return "OK";
        case LIFT_ERR_DAC_NOT_FOUND:  return "GP8413 0-10V DAC NOT FOUND";
        case LIFT_ERR_ADC_NOT_FOUND:  return "ADS1110 ADC (0x48) NOT FOUND";
        case LIFT_ERR_ADC_CONFIG_FAIL:return "ADS1110 ADC config write failed";
        case LIFT_ERR_ADC_READ_FAIL:  return "ADS1110 ADC reading failed";
        default:                      return "Unknown error";
    }
}

int Lift::checkHardware(bool &dac_ok, bool &adc_ok) {
    Wire.beginTransmission(dacAddress);
    dac_ok = (Wire.endTransmission() == 0);

    Wire.beginTransmission(ADC_ADDR);
    adc_ok = (Wire.endTransmission() == 0);

    if (!dac_ok) return LIFT_ERR_DAC_NOT_FOUND;
    if (!adc_ok) return LIFT_ERR_ADC_NOT_FOUND;
    return LIFT_OK;
}

int Lift::init(DFRobot_GP8XXX::eOutPutRange_t range) {
    pinMode(upPin, OUTPUT);
    pinMode(downPin, OUTPUT);
    digitalWrite(upPin, LOW);
    digitalWrite(downPin, LOW);

    // 1. Initialize DAC GP8413
    Wire.beginTransmission(dacAddress);
    if (Wire.endTransmission() != 0) {
        return LIFT_ERR_DAC_NOT_FOUND;
    }
    int status = GP8413.begin();
    if (status != 0) {
        return LIFT_ERR_DAC_NOT_FOUND;
    }
    GP8413.setDACOutRange(range);
    setSpeed(0);

    // 2. Initialize and configure ADC ADS1110 (0x48) (240fps continuous mode, PGA=1)
    Wire.beginTransmission(ADC_ADDR);
    if (Wire.endTransmission() != 0) {
        return LIFT_ERR_ADC_NOT_FOUND;
    }
    Wire.beginTransmission(ADC_ADDR);
    Wire.write(ADC_CONFIG_240FPS_PGA1);
    if (Wire.endTransmission() != 0) {
        return LIFT_ERR_ADC_CONFIG_FAIL;
    }

    stop();
    update();
    return LIFT_OK;
}

void Lift::update() {
    sensorValueRaw = readADC();
    sensorValue = ewmaFilterIn.filter(sensorValueRaw);
    inclinaison_deg = sensorValue * ANALOG_TO_ANGLE_GAIN + ANALOG_TO_ANGLE_OFFSET;
    height_mm = computeHeight(inclinaison_deg);
}

float Lift::getInclinaison_deg() {
    return inclinaison_deg;
}

float Lift::getHeight_mm() {
    return height_mm;
}

float Lift::getVoltage(bool raw) {
    return raw ? sensorValueRaw : sensorValue;
}

float Lift::getSensorValue(bool raw) {
    return raw ? sensorValueRaw : sensorValue;
}

float Lift::computeHeight(float angle_deg) {
    auto angle2mm = [](float angle) {
        return BELT_LENGTH_MM * sin(radians(angle)) + BELT_HEIGHT_MM * sin(radians(90.0f - angle));
    };
    float verticalPosition_mm = angle2mm(angle_deg);
    if (angle_deg > THRESHOLD_ANGLE_DEG) {
        verticalPosition_mm = computeHeight(THRESHOLD_ANGLE_DEG) + SLOPE_MM_PER_DEG * (angle_deg - THRESHOLD_ANGLE_DEG);
    }
    return verticalPosition_mm;
}

float Lift::horizontalPosition(float angle_deg) {
    double angle = radians(angle_deg);
    return BELT_LENGTH_MM * cos(angle) - BELT_HEIGHT_MM * cos(PI/2 - angle);
}

void Lift::setSpeed(float speed_pct) {
    float pct = constrain(fabs(speed_pct), 0.0f, 100.0f);
    float dac_val_f = (pct / 100.0f) * 32767.0f * SPEED_TO_ANALOG_GAIN + SPEED_TO_ANALOG_OFFSET;
    uint16_t dac_val = (uint16_t)constrain(round(dac_val_f), 0, 32767);
    GP8413.setDACOutVoltage(dac_val, 0);
}

void Lift::moveUp() {
    digitalWrite(upPin, HIGH);
    digitalWrite(downPin, LOW);
}

void Lift::moveDown() {
    digitalWrite(upPin, LOW);
    digitalWrite(downPin, HIGH);
}

void Lift::stop() {
    digitalWrite(upPin, LOW);
    digitalWrite(downPin, LOW);
    setSpeed(0);
}

void Lift::move(float pid_output) {
    float speed = ewmaFilterOut.filter(int(pid_output * 10)) / 10.0f;
    setSpeed(speed);
    if (speed > 0.0f) {
        moveUp();
    } else if (speed < 0.0f) {
        moveDown();
    } else {
        stop();    
    }
}
