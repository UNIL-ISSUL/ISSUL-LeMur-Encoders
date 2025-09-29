/**
 * @file HardwareManager.cpp
 * @brief Implementation of the HardwareManager class.
 */
#include "HardwareManager.h"

/**
 * @brief Construct a new HardwareManager:: Hardware Manager object
 *
 * Initializes the I2C device objects with their respective addresses from the config file.
 */
HardwareManager::HardwareManager() :
    _i2cMultiplexer(I2C_MULTIPLEXER_ADDR),
    _dac(DAC_ADDR)
{
    // The rest of the initialization is done in the begin() method
}

/**
 * @brief Initializes all hardware components.
 *
 * This method sets up the I2C bus and then initializes the I2C multiplexer,
 * both encoders, the ADC, and the DAC. It checks for errors during initialization.
 *
 * @return true if all components initialize successfully, false otherwise.
 */
bool HardwareManager::begin() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_FREQ);

    uint8_t i2c_error = 0;

    // Initialize I2C Multiplexer
    if (!_i2cMultiplexer.begin()) {
        Serial.println("I2C Multiplexer initialization failed");
        i2c_error++;
    }

    // Initialize Encoders on their respective multiplexer channels
    for (int i = 0; i < MAX_ENCODER_CHANNELS; i++) {
        _i2cMultiplexer.selectChannel(i);
        if (_i2cMultiplexer.isConnected(ENCODER_ADDR)) {
            _encoders[i].begin(&Wire, ENCODER_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
        } else {
            Serial.println("Encoder device not found on channel " + String(i));
            i2c_error++;
        }
    }

    // Initialize ADC
    Wire.beginTransmission(ADC_ADDR);
    if (Wire.endTransmission() == 0) {
        const uint8_t config = 0x84; // 60fps continuous mode
        Wire.beginTransmission(ADC_ADDR);
        Wire.write(config);
        Wire.endTransmission();
    } else {
        Serial.println("ADC device not found");
        i2c_error++;
    }

    // Initialize DAC
    if (_dac.begin() != 0) {
        Serial.println("DAC device not found");
        i2c_error++;
    }
    _dac.setDACOutRange(DFRobot_GP8413::eOutputRange10V);
    _dac.setDACOutVoltage(0, 0); // Set initial output to 0V

    return i2c_error == 0;
}

/**
 * @brief Resets the internal counters of all encoders.
 */
void HardwareManager::resetEncoders() {
    for (int i = 0; i < MAX_ENCODER_CHANNELS; i++) {
        _i2cMultiplexer.selectChannel(i);
        _encoders[i].resetEncoder();
    }
}

/**
 * @brief Reads the raw pulse count from a specific encoder.
 * @param channel The encoder channel to read (0 or 1).
 * @return The current encoder pulse count.
 */
uint32_t HardwareManager::readEncoder(uint8_t channel) {
    if (channel >= MAX_ENCODER_CHANNELS) {
        return 0; // Return 0 if the channel is invalid
    }
    _i2cMultiplexer.selectChannel(channel);
    return _encoders[channel].getEncoderValue();
}

/**
 * @brief Reads the raw value from the ADC.
 * @return The ADC reading, scaled to millivolts.
 */
float HardwareManager::readADC() {
    Wire.requestFrom(ADC_ADDR, 2);
    int16_t raw = (Wire.read() << 8) | Wire.read();
    // Scale the raw 14-bit value to millivolts (assuming a max of 12440mV)
    return raw * 12440.0 / 8192;
}

/**
 * @brief Sets the DAC output voltage.
 * @param voltage The voltage to set, in millivolts.
 */
void HardwareManager::setDACOutput(float voltage) {
    // The DAC function expects a value, not the channel. We assume channel 0.
    _dac.setDACOutVoltage(voltage, 0);
}

/**
 * @brief Provides access to the I2C multiplexer.
 * @return A reference to the TCA9548 object.
 */
TCA9548& HardwareManager::getMultiplexer() {
    return _i2cMultiplexer;
}