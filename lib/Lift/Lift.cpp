#include <Lift.h>
#include <Arduino.h>
#include <Ewma.h>
#include <Wire.h>


//define a EWMA filter to smooth the inclinometer value
Ewma ewmaFilterIn(0.01);
Ewma ewmaFilterOut(0.01);

//SMA<50> average ;
//SMA<100, float, float> averageSensor;

static void writeRegister(uint8_t i2cAddress, uint8_t value)
{
    Wire.beginTransmission(i2cAddress);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
}

static uint16_t readRegister(uint8_t i2cAddress)
{
    Wire.requestFrom(i2cAddress, (uint8_t)2);
    return (int16_t)((Wire.read() << 8) | Wire.read());
}


Lift::Lift (char inclinometerPin, char upPin, char downPin, char speedPin, bool i2c) {
    this->i2c = i2c;
    if(i2c) {
        //init i2c
        Wire.begin();
        //send ADC configuration to inclinometer
        const uint8_t config = 0x84; // 1000 0100 see ADC1110 datasheet : 60fps continuous mode
        writeRegister(ADC_I2C_ADDRESS, config);
    }
    else {
        this->inclinometerPin = inclinometerPin;
    }
    this->upPin = upPin;
    this->downPin = downPin;
    this->speedPin = speedPin;
    //make sure lift is stopped
    stop();
    //measure current position
    update();
}

Lift::~Lift() {
    stop();
}

void Lift::update() {
    //read sensor value
    if(i2c) {
        sensorValueRaw = readRegister(ADC_I2C_ADDRESS);
    }
    else {
        sensorValueRaw  = analogRead(inclinometerPin);
    }
    //apply ewma filter
    sensorValue = ewmaFilterIn.filter(sensorValueRaw);
    //convert sensor value to angle
    float temp;
    if (i2c) {
        //map 14bit sensor value to 0-180° - 14bit @60fps
        temp = sensorValue * 180 / 4096;
        inclinaison_deg = temp * ANALOG_TO_ANGLE_GAIN_I2C + ANALOG_TO_ANGLE_OFFSET_I2C;
    } else {
        //map 10bit sensor value to 0-180°
        temp = sensorValue * 180 / 1023;
        inclinaison_deg = temp * ANALOG_TO_ANGLE_GAIN + ANALOG_TO_ANGLE_OFFSET;
    }
    //Serial.println("sensorValue:"+String(sensorValueRaw)+"\tsensorValueFiltered:"+String(sensorValue)+"\tangle:"+String(temp));
    inclinaison_deg = temp * ANALOG_TO_ANGLE_GAIN + ANALOG_TO_ANGLE_OFFSET;
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
    //the pivot point is not directly under the runnimg surafce of the belt (wheel on groud)
    float verticalPosition_mm = angle2mm(angle_deg);
    //Around 78°, there an additional slope on the ground 6.3853 mm / deg, so we need to compensate the height calculation
    if (angle_deg > THRESHOLD_ANGLE_DEG) {
        verticalPosition_mm = computeHeight(THRESHOLD_ANGLE_DEG) + SLOPE_MM_PER_DEG * (angle_deg - THRESHOLD_ANGLE_DEG);
    }
    return verticalPosition_mm;
}

float Lift::horizontalPosition(float angle_deg) {
    double angle = radians(angle_deg);
    return BELT_LENGTH_MM * cos(angle) - BELT_HEIGHT_MM * cos(PI/2 - angle);
}

//set speed in a range 0 to 100% depending on max frequency configured in header file
void Lift::setSpeed(float speed_pct) {
    //convert speed in % to pwm value
    //max pwm is 127 because of the motor driver analog signal fixed to 5V when controller output is 10V
    //convert decimal to int with a factor 10 to get 1 decimal precision
    long temp = abs(round(speed_pct*10));
    double pwm = map(temp, 0, 1000, 0, 127) * SPEED_TO_ANALOG_GAIN + SPEED_TO_ANALOG_OFFSET;
    analogWrite(speedPin, pwm);
    speed_pct = speed_pct;
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
    //float speed = averageOutput(pid_output);
    float speed = ewmaFilterOut.filter(int(pid_output*10))/10;
    //Serial.println(">motor output:"+String(speed));
    setSpeed(speed);
    //move up or down depending on pid output
    if (speed > 0) {
        moveUp();
    } else if (speed < -0) {
        moveDown();
    } else {
        stop();    
    }
}