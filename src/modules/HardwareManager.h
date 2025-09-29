/**
 * @file HardwareManager.h
 * @brief Manages all hardware interactions for the LeMur project.
 *
 * This class abstracts the low-level details of initializing and communicating
 * with I2C devices, including the I2C multiplexer, encoders, ADC, and DAC.
 */
#ifndef HARDWARE_MANAGER_H
#define HARDWARE_MANAGER_H

#include <M5Atom.h>
#include <Wire.h>
#include "TCA9548.h"
#include "DFRobot_GP8XXX.h"
#include "UNIT_EXT_ENCODER.h"
#include "../config/config.h"

class HardwareManager {
public:
    /**
     * @brief Construct a new HardwareManager object.
     */
    HardwareManager();

    /**
     * @brief Initializes all I2C devices.
     * @return True if all devices are initialized successfully, false otherwise.
     */
    bool begin();

    /**
     * @brief Resets the counts of all encoders to zero.
     */
    void resetEncoders();

    /**
     * @brief Reads the raw value from a specified encoder channel.
     * @param channel The encoder channel to read (0 or 1).
     * @return The raw encoder count.
     */
    uint32_t readEncoder(uint8_t channel);

    /**
     * @brief Reads the value from the ADC and converts it to millivolts.
     * @return The ADC reading in mV.
     */
    float readADC();

    /**
     * @brief Sets the output voltage of the DAC.
     * @param voltage The desired output voltage in mV.
     */
    void setDACOutput(float voltage);

    /**
     * @brief Gets a reference to the I2C multiplexer object.
     * @return A reference to the TCA9548 object.
     */
    TCA9548& getMultiplexer();

private:
    TCA9548 _i2cMultiplexer; ///< The I2C multiplexer object.
    DFRobot_GP8413 _dac;     ///< The DAC object.
    UNIT_EXT_ENCODER _encoders[MAX_ENCODER_CHANNELS]; ///< An array of encoder objects.
};

#endif // HARDWARE_MANAGER_H