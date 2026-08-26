#include "Lift.h"
#include <Arduino.h>
#include <Ewma.h>
#include <Wire.h>
#include <math.h>

//define a EWMA filter to smooth the inclinometer value
Ewma ewmaFilterIn(0.01);
Ewma ewmaFilterOut(0.01);

//Read m5stack ADCv1.1 (ADS1110) in 12-bit @ 240fps mode
float Lift::readADC() {
  const int length = 2;
  Wire.requestFrom(ADC_ADDR, length);
  int16_t raw = (int16_t)((Wire.read() << 8) | Wire.read());
  const int16_t min_code = 2048; // 12-bit @ 240fps (PGA=1, full scale 2.048V)
  float deltaV = raw * 2.048f / min_code; // conversion ADS1110 in datasheet (1 LSB = 1mV)
  return deltaV * 6.1f; // m5stack ADCv1.1 has a voltage divider made by 510k and 100k resistors (510+100) / 100
}

Lift::Lift(char upPin, char downPin, uint8_t dacAddr) : GP8413(dacAddr) {
    this->upPin = upPin;
    this->downPin = downPin;
    sensorValue = 0.0f;
    sensorValueRaw = 0;
    inclinaison_deg = 0.0f;
    height_mm = 0.0f;
}

Lift::~Lift() {
    stop();
}

int Lift::init(DFRobot_GP8XXX::eOutPutRange_t range) {
    pinMode(upPin, OUTPUT);
    pinMode(downPin, OUTPUT);
    digitalWrite(upPin, LOW);
    digitalWrite(downPin, LOW);

    // 1. Initialize DAC GP8413
    int status = GP8413.begin();
    if (status != 0) {
        return status;
    }
    GP8413.setDACOutRange(range);

    // 2. Initialize and configure ADC ADS1110 (240fps continuous mode, PGA=1)
    Wire.beginTransmission(ADC_ADDR);
    Wire.write(ADC_CONFIG_240FPS_PGA1); // 0x80 (1000 0000)
    int adc_status = Wire.endTransmission();
    if (adc_status != 0) {
        return adc_status;
    }

    stop();
    update();
    return 0;
}

void Lift::update() {
    //read sensor value
    sensorValueRaw = readADC();
    //apply ewma filter
    sensorValue = ewmaFilterIn.filter(sensorValueRaw);
    //convert sensor value to angle
    inclinaison_deg = sensorValue * ANALOG_TO_ANGLE_GAIN + ANALOG_TO_ANGLE_OFFSET;
    height_mm = computeHeight(inclinaison_deg);
}

float Lift::getInclinaison_deg() {
    return inclinaison_deg;
}

float Lift::getHeight_mm() {
    return height_mm;
}

float Lift::getSensorValue(bool raw) {
    if (raw) {
        return sensorValueRaw;
    } else {
        return sensorValue;
    }
}

float Lift::computeHeight(float angle_deg) {
    auto angle2mm = [](float angle) {
        return BELT_LENGTH_MM * sin(radians(angle)) + BELT_HEIGHT_MM * sin(radians(90 - angle));
    };
    //the pivot point is not directly under the running surface of the belt (wheel on ground)
    float verticalPosition_mm = angle2mm(angle_deg);
    //Around 78°, there is an additional slope on the ground 6.3853 mm / deg, so we need to compensate the height calculation
    if (angle_deg > THRESHOLD_ANGLE_DEG) {
        verticalPosition_mm = computeHeight(THRESHOLD_ANGLE_DEG) + SLOPE_MM_PER_DEG * (angle_deg - THRESHOLD_ANGLE_DEG);
    }
    return verticalPosition_mm;
}

float Lift::horizontalPosition(float angle_deg) {
    double angle = radians(angle_deg);
    return BELT_LENGTH_MM * cos(angle) - BELT_HEIGHT_MM * cos(PI/2 - angle);
}

//set speed in a range 0 to 100%
void Lift::setSpeed(float speed_pct) {
    float pct = constrain(fabs(speed_pct), 0.0f, 100.0f);
    //GP8413 has 15-bit resolution (0 to 32767) corresponding to configured range (0-10V)
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
    //move up or down depending on pid output
    if (speed > 0.0f) {
        moveUp();
    } else if (speed < 0.0f) {
        moveDown();
    } else {
        stop();    
    }
}